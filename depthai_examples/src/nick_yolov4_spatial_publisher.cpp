
#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

const std::vector<std::string> label_map = {
    "ok", "stop", "palm", "fist", "like", "peace", "peace_inv"
};

int downscaledWidth = 416;
int downscaleHeight = 416;

dai::Pipeline createPipeline(int fps, bool syncNN, bool subpixel, std::string nnPath, int confidence, int LRchecktresh, std::string resolution) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(downscaledWidth, downscaleHeight);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(fps);

    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(fps);

    /// setting node configs
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    /* ********************************* */
    /* STEREO CHANGES FOR 3d POINTCLOUDS */
    /// output size
    // ### iheight/iwidth will be decimate_filtered (4?) and the result has to be divisible by 16 cause of tinyyolo
    // ### the decimation filter is important, because it reduced what the filters do BEFORE resizing to output
    // ### so have hih computation time (slow and laggy camera images) when there is no decimation
    // ### ULTRA HIGH RESOLUTION
    // ###   16(needed) * 20(multiplier) * 4(decimation) = 1280?
    // ### HIGH RESOLUTION
    // ###   16(needed) * 10(multiplier) * 4(decimation) = 640?
    // ### LOW/MEDIUM RESOLUTION
    // ###   16(needed) * 5(multiplier) * 4(decimation) = 320?
    // ### VERY LOW RESOLUTION
    // ###   16(needed) * 2(multiplier) * 4(decimation) = 128?
    // ### multiplier is just a factor we can arbitrarily use for higher / lower result resolutions
    stereo->setOutputSize(320, 320);
    /// grab raw config
    auto config = stereo->initialConfig.get();
    /// median filter
    config.postProcessing.median = dai::MedianFilter::MEDIAN_OFF; // ?? KERNEL_7x7
    /// decimation filter
    config.postProcessing.decimationFilter.decimationMode = dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEDIAN;
    config.postProcessing.decimationFilter.decimationFactor = 4;
    /// spatial filter
    config.postProcessing.spatialFilter.enable = true;
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.alpha = 0.5;
    config.postProcessing.spatialFilter.delta = 20;
    config.postProcessing.spatialFilter.numIterations = 1;
    /// threshold filter
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 10000;
    /// speckle filter
    config.postProcessing.speckleFilter.enable = true;
    config.postProcessing.speckleFilter.speckleRange = 50;
    /// set raw config
    stereo->initialConfig.set(config);
    /* ********************************* */

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    /// nn pipeline
    if (!nnPath.empty()) {
        auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
        auto xoutNN = pipeline.create<dai::node::XLinkOut>();
        xoutNN->setStreamName("detections");
        spatialDetectionNetwork->setBlobPath(nnPath);
        spatialDetectionNetwork->setConfidenceThreshold(0.33f);
        spatialDetectionNetwork->input.setBlocking(false);
        spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
        spatialDetectionNetwork->setDepthLowerThreshold(100);
        spatialDetectionNetwork->setDepthUpperThreshold(5000);
        // yolo specific parameters
        spatialDetectionNetwork->setNumClasses(label_map.size());
        spatialDetectionNetwork->setCoordinateSize(4);
        spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
        spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
        spatialDetectionNetwork->setIouThreshold(0.33f);
        // Link plugins CAM -> NN -> XLINK
        colorCam->preview.link(spatialDetectionNetwork->input);
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);

        // if(syncNN)
        //     spatialDetectionNetwork->passthrough.link(xoutRgb->input);
        // else
        //     colorCam->preview.link(xoutRgb->input);
        spatialDetectionNetwork->out.link(xoutNN->input);
        stereo->depth.link(spatialDetectionNetwork->inputDepth);
        spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    }
    else{
        colorCam->preview.link(xoutRgb->input);
        stereo->depth.link(xoutDepth->input);
    }
    /// ...
    return pipeline;
}


std::string spatialFrameId;
std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub;
void spatialCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
    std::deque<vision_msgs::msg::Detection3DArray> deq;
    if(detConverter.get()){
        detConverter->toRosVisionMsg(inDet, deq);
        while(deq.size() > 0) {
            auto currMsg = deq.front();
            /// kikass13:
            /// change frame id to something more useful
            currMsg.header.frame_id = spatialFrameId;
            detPub->publish(currMsg);
            deq.pop_front();
        }

    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov4_spatial_node");

    std::string tfPrefix, nnPath;
    std::string camera_param_uri;
    bool syncNN, subpixel;
    int confidence = 200, LRchecktresh = 10;
    std::string monoResolution = "400p";
    int fps;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnPath", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("fps", 10);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("nnPath", nnPath);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("fps", fps);

    RCLCPP_INFO(node->get_logger(), "NICK depthai ros driver using nnPath = '%s'", nnPath.c_str());
    RCLCPP_INFO(node->get_logger(), "  - creating pipeline ... ");
    dai::Pipeline pipeline = createPipeline(fps, syncNN, subpixel, nnPath, confidence, LRchecktresh, monoResolution);
    RCLCPP_INFO(node->get_logger(), "  - connecting device ... ");
    dai::Device device(pipeline);
    RCLCPP_INFO(node->get_logger(), "  - defining queues ... ");
    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto calibrationHandler = device.readCalibration();

    int width, height;
    if(monoResolution == "720p") {
        width = 1280;
        height = 720;
    } else if(monoResolution == "400p") {
        width = 640;
        height = 400;
    } else if(monoResolution == "800p") {
        width = 1280;
        height = 800;
    } else if(monoResolution == "480p") {
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    RCLCPP_INFO(node->get_logger(), "  - configuring converters & publishers ... ");
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
                                                                                       node,
                                                                                       std::string("/oak/rgb/image_raw"),
                                                                                       std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                 &rgbConverter,  // since the converter has the same frame name
                                                                                                                 // and image type is also same we can reuse it
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2),
                                                                                       30,
                                                                                       rgbCameraInfo,
                                                                                       "rgb");
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        depthQueue,
        node,
        std::string("/oak/stereo/image_raw"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "stereo");
    
    depthPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    std::shared_ptr<dai::DataOutputQueue> detectionQueue;
    if (!nnPath.empty()) {
        int spatialWidth = downscaledWidth;
        int spatialHeight = downscaleHeight;
        if(syncNN){
            spatialWidth = width;
            spatialHeight = height;
        }
        spatialFrameId = tfPrefix + "_rgb_camera_optical_frame";
        detPub = node->template create_publisher<vision_msgs::msg::Detection3DArray>("/oak/nn/spatial_detections", 10);
        detectionQueue = device.getOutputQueue("detections", 30, false);
        /// DEFAULT SPATIAL DETECTION PUBLISHER USING DEPTHAI ROS TYPE
        /// ########################################################
        // dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", spatialWidth, spatialHeight, false);
        // dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        // detectionQueue,
        // node,
        // std::string("/oak/nn/spatial_detections"),
        // std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        // 30);
        // detectionPublish.addPublisherCallback();
        /// this is here, because we cannot create and in scope BridgePublisher ... because some people are lazy 
        /// and i dont want to remember some random pointers 
        // RCLCPP_INFO(node->get_logger(), "  - spinning ... ");
        // rclcpp::spin(node);
        /// ########################################################
        /// DEPTHAI CONVERTER TO GENERATE ROS2 vision_msgs type
        detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(tfPrefix + "_camera_optical_frame", width, height, false);
        detectionQueue->addCallback(std::bind(&spatialCB, std::placeholders::_1, std::placeholders::_2));
    }
    RCLCPP_INFO(node->get_logger(), "  - spinning ... ");
    rclcpp::spin(node);

    return 0;
}
