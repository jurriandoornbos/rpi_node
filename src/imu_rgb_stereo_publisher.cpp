#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>

#include <depthai_bridge/ImuConverter.hpp>

static constexpr int fps = 30;
static constexpr int hz = 200;
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;
static std::atomic<bool> downscaleColor{true};

dai::Pipeline createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel){
        // Create pipeline
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    if(downscaleColor) camRgb->setIspScale(1, 3);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->initialConfig.setConfidenceThreshold(245);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(lrcheck);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);


        // Add IMU To the pipeline

    auto imuSensor = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    
    // enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 100 hz rate
    imuSensor->enableIMUSensor({dai::IMUSensor::LINEAR_ACCELERATION, dai::IMUSensor::GYROSCOPE_CALIBRATED, dai::IMUSensor::ROTATION_VECTOR}, 200);
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imuSensor->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imuSensor->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imuSensor->out.link(xoutImu->input);

    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName, mode;
    std::string cameraParamUri;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;

    badParams += !pnh.getParam("camera_name", deviceName);
    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("lrcheck",  lrcheck);
    badParams += !pnh.getParam("extended",  extended);
    badParams += !pnh.getParam("subpixel",  subpixel);
    

    if (badParams > 0)
    {   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }


    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel);

    dai::Device device(pipeline);

    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto rgbQueue = device.getOutputQueue("rgb", 30, false);

    auto imuQueue = device.getOutputQueue("imu", hz, false);

    auto calibrationHandler = device.readCalibration();

    // this part would be removed once we have calibration-api
    /*     
     std::string leftUri = cameraParamUri +"/" + "left.yaml";

     std::string rightUri = cameraParamUri + "/" + "right.yaml";

     std::string stereoUri = cameraParamUri + "/" + "right.yaml";
    */
    std::cout << "USB SPEED: " << device.getUsbSpeed() << std::endl;

     

    dai::rosBridge::ImageConverter converter(deviceName + "_right_camera_optical_frame", true);
    auto rgbCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(rgbQueue,
                                                                                    pnh, 
                                                                                    std::string("color/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    rgbCameraInfo,
                                                                                    "color");

    rgbPublish.addPubisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &converter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rgbCameraInfo,
                                                                                     "stereo");

    depthPublish.addPubisherCallback();
    dai::rosBridge::ImuConverter imuConverter(deviceName + "_imu_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(imuQueue,
                                                                                    pnh,
                                                                                    std::string("imu"),
                                                                                    std::bind(static_cast<void(dai::rosBridge::ImuConverter::*)
                                                                                    (std::shared_ptr<dai::IMUData>, sensor_msgs::Imu&)
                                                                                    >(&dai::rosBridge::ImuConverter::toRosMsg),
                                                                                    &imuConverter,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2),
                                                                                    hz,
                                                                                    rgbCameraInfo,
                                                                                    "imu");

    imuPublish.addPubisherCallback();  

    ros::spin();
    

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    
    return 0;
}
