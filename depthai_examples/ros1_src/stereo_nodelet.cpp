#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <tuple>
#include <iostream>
#include <vector>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

using namespace std;
namespace depthai_examples{


 class StereoNodelet : public nodelet::Nodelet
{

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish,rgbPublish,rgbPublishOAK;
    std::unique_ptr<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>> detectionPublish,detectionPublishOAK;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter,rgbConverter,rgbConverterOAK;
    std::unique_ptr<dai::rosBridge::ImgDetectionConverter> detConverter,detConverterOAK;
    std::unique_ptr<dai::Device> _dev,_dev_mono,_dev_temp;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string tfPrefix, mode;
            std::string cameraParamUri;
            std::string monoResolution = "400p";
            int badParams = 0;
            bool lrcheck, extended, subpixel, enableDepth;
            int confidence = 200;
            int LRchecktresh = 5;

            std::string resourceBaseFolder("/home/rami/nr22-software/src/deps/depthai-ros/depthai_examples/resources");
            std::string nnPath;
            std::string nnName("mobilenet-ssd_openvino_2021.2_6shave.blob");
            bool syncNN(true);

            badParams += !pnh.getParam("tf_prefix", tfPrefix);
            badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck",  lrcheck);
            badParams += !pnh.getParam("extended",  extended);
            badParams += !pnh.getParam("subpixel",  subpixel);
            badParams += !pnh.getParam("confidence",  confidence);
            badParams += !pnh.getParam("LRchecktresh",  LRchecktresh);
            badParams += !pnh.getParam("monoResolution",  monoResolution);
            
            if (badParams > 0)
            {   
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find %d of the parameters");
            }

            if(mode == "depth"){
                enableDepth = true;
            }
            else{
                enableDepth = false;
            }

            dai::Pipeline pipeline,pipeline2;
            int monoWidth, monoHeight, rgbWidth, rgbHeight;

            // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
            if(pnh.hasParam("nnName")) {
                pnh.getParam("nnName", nnName);
            }

            if(resourceBaseFolder.empty()) {
                throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
            }

            nnPath = resourceBaseFolder + "/" + nnName;

            //std::vector<dai::DeviceInfo> devices= dai::Device::getAllAvailableDevices(); 

            // for(dai::DeviceInfo d : devices) {
            //     _dev_temp= std::make_unique<dai::Device>(pipeline,d);
                
            //     std::vector<dai::CameraBoardSocket> c= _dev_temp->getConnectedCameras();
            //     cout <<"CREATING DEV TEMP \n" << "SIZE " << c.size() <<endl;
            //     if(c.size()>1) {
            //         //_dev = std::make_unique<dai::Device>(pipeline,d,false); //error here
            //         _dev=std::move(_dev_temp);

            //         cout << "OAK-D detected";
            //     } else {
            //         _dev_mono=std::move(_dev_temp);
            //     }
            // }

            bool found; dai::DeviceInfo device_info,device_info2;


            // std::tie(pipeline2,rgbWidth,rgbHeight) = createMonoPipeline(syncNN, nnPath); //this line BAD
            // std::tie(found, device_info2) = dai::Device::getDeviceByMxId("1844301081F1670F00");
            // _dev_mono = std::make_unique<dai::Device>(pipeline2,device_info2,true); //OAK-1 MXID: 1844301081F1670F00

            std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution,syncNN, nnPath);
            std::tie(found, device_info) = dai::Device::getDeviceByMxId("1844301021693E0E00");
            _dev = std::make_unique<dai::Device>(pipeline,device_info,false); // Our OAK-D MXID: 1844301021693E0E00
           
        
        auto leftQueue = _dev->getOutputQueue("left", 15, false);
        auto rightQueue = _dev->getOutputQueue("right", 15, false);

        std::shared_ptr<dai::DataOutputQueue> stereoQueue;
        if (enableDepth) {
            stereoQueue = _dev->getOutputQueue("depth", 15, false);
        }else{
            stereoQueue = _dev->getOutputQueue("disparity", 15, false);
        }

        std::shared_ptr<dai::DataOutputQueue> previewQueue = _dev->getOutputQueue("preview", 15, false);
        std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = _dev->getOutputQueue("detections", 15, false);
        auto calibrationHandler = _dev->readCalibration();

        // std::shared_ptr<dai::DataOutputQueue> previewQueueRGB = _dev_mono->getOutputQueue("preview", 15, false);
        // std::shared_ptr<dai::DataOutputQueue> nNetDataQueueRGB = _dev_mono->getOutputQueue("detections", 15, false);
        // this part would be removed once we have calibration-api
        /*             
        std::string left_uri = camera_param_uri +"/" + "left.yaml";

        std::string right_uri = camera_param_uri + "/" + "right.yaml";

        std::string stereo_uri = camera_param_uri + "/" + "right.yaml"; 
        */

        auto boardName = calibrationHandler.getEepromData().boardName;
        if (monoHeight > 480 && boardName == "OAK-D-LITE") {
            monoWidth = 640;
            monoHeight = 480;
        }

        

        leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_left_camera_optical_frame", true);
        auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 

        leftPublish  = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                        (leftQueue,
                                                                                            pnh, 
                                                                                            std::string("left/image"),
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            leftConverter.get(),
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            15,
                                                                                            leftCameraInfo,
                                                                                            "left");

        // bridgePublish.startPublisherThread();
        leftPublish->addPublisherCallback();

        rightConverter = std::make_unique<dai::rosBridge::ImageConverter >(tfPrefix + "_right_camera_optical_frame", true);
        auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight); 

        rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                        (rightQueue,
                                                                                            pnh, 
                                                                                            std::string("right/image"),
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            rightConverter.get(), 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            15,
                                                                                            rightCameraInfo,
                                                                                            "right");

        rightPublish->addPublisherCallback();

        // dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame");
        depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                        (stereoQueue,
                                                                            pnh, 
                                                                            std::string("stereo/depth"),
                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                            rightConverter.get(), // since the converter has the same frame name
                                                                                            // and image type is also same we can reuse it
                                                                            std::placeholders::_1, 
                                                                            std::placeholders::_2) , 
                                                                            15,
                                                                            rightCameraInfo,
                                                                            "stereo");

        depthPublish->addPublisherCallback();

        //We can add the rectified frames also similar to these publishers. 
        //Left them out so that users can play with it by adding and removing

        
        //dai::Device device(pipeline);

        std::string color_uri = cameraParamUri + "/" + "color.yaml";

        //auto centercamerainfo=
        //dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

        rgbConverter= std::make_unique<dai::rosBridge::ImageConverter >(tfPrefix + "_rgb_camera_optical_frame", true);
        rgbPublish=  std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> (previewQueue,
                                                                                    pnh,
                                                                                    std::string("color/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                rgbConverter.get(),  // since the converter has the same frame name
                                                                                                                // and image type is also same we can reuse it
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2),
                                                                                    15,
                                                                                    color_uri,
                                                                                    "color");
        //intercept toRosmsg 

        //dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);;
        detConverter = std::make_unique<dai::rosBridge::ImgDetectionConverter> (tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);
        
        detectionPublish= std::make_unique<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>> (
            nNetDataQueue,
            pnh,
            std::string("color/mobilenet_detections"),
            std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, detConverter.get(), std::placeholders::_1, std::placeholders::_2),
            15);

        cout << "PUBLISHER CALLBACK" << endl;
        detectionPublish->addPublisherCallback();
        rgbPublish->addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.


        //OAK-1 PUBLISHERS
        
        // rgbConverterOAK= std::make_unique<dai::rosBridge::ImageConverter >(tfPrefix + "_camera_optical_frame", true);
        // rgbPublishOAK=  std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> (previewQueueRGB,
        //                                                                             pnh,
        //                                                                             std::string("OAK1/image"),
        //                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
        //                                                                                         rgbConverterOAK.get(),  // since the converter has the same frame name
        //                                                                                                         // and image type is also same we can reuse it
        //                                                                                         std::placeholders::_1,
        //                                                                                         std::placeholders::_2),
        //                                                                             15,
        //                                                                             "",
        //                                                                             "OAK1");
        // //intercept toRosmsg 

        // //dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);;
        // detConverterOAK = std::make_unique<dai::rosBridge::ImgDetectionConverter> (tfPrefix + "_camera_optical_frame", 300, 300, false);
        
        // detectionPublishOAK= std::make_unique<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>> (
        //     nNetDataQueueRGB,
        //     pnh,
        //     std::string("OAK1/mobilenet_detections"),
        //     std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, detConverterOAK.get(), std::placeholders::_1, std::placeholders::_2),
        //     15);

        // cout << "PUBLISHER CALLBACK" << endl;
        // detectionPublishOAK->addPublisherCallback();
        // rgbPublishOAK->addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.
        
        }

    void publishMono(dai::Device & _dev) {

    }
    std::tuple<dai::Pipeline,int,int> createMonoPipeline(bool syncNN, std::string nnPath) {
        dai::Pipeline pipeline;
        nnPath="/home/rami/nr22-software/src/deps/depthai-ros/depthai_examples/resources/mobilenet-ssd_openvino_2021.2_6shave_copy.blob";
        auto colorCam = pipeline.create<dai::node::ColorCamera>();
        auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
        auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
        auto nnOut = pipeline.create<dai::node::XLinkOut>();

        xlinkOut->setStreamName("preview");
        nnOut->setStreamName("detections");

        colorCam->setPreviewSize(300, 300);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(false);
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        colorCam->setFps(15);

        // testing MobileNet DetectionNetwork
        detectionNetwork->setConfidenceThreshold(0.5f);
        detectionNetwork->setBlobPath(nnPath);

        // Link plugins CAM -> NN -> XLINK
        colorCam->preview.link(detectionNetwork->input);
        if(syncNN)
            detectionNetwork->passthrough.link(xlinkOut->input);
        else
            colorCam->preview.link(xlinkOut->input);

        detectionNetwork->out.link(nnOut->input);
        int width= 1920;
        int height= 1080;
        return std::make_tuple(pipeline, width, height);
    }

    std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution,bool syncNN, std::string nnPath){
        dai::Pipeline pipeline;

        auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
        auto monoRight   = pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
        auto stereo      = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");

        if (withDepth) {
            xoutDepth->setStreamName("depth");
        }
        else {
            xoutDepth->setStreamName("disparity");
        }

        int width, height;
        dai::node::MonoCamera::Properties::SensorResolution monoResolution;
        if(resolution == "720p"){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
            width  = 1280;
            height = 720;
        }else if(resolution == "400p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
            width  = 640;
            height = 400;
        }else if(resolution == "800p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
            width  = 1280;
            height = 800;
        }else if(resolution == "480p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
            width  = 640;
            height = 480;
        }else{
            ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
            throw std::runtime_error("Invalid mono camera resolution.");
        }

        // MonoCamera
        monoLeft->setResolution(monoResolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(monoResolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // int maxDisp = 96;
        // if (extended) maxDisp *= 2;
        // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

        // StereoDepth
        stereo->initialConfig.setConfidenceThreshold(confidence);
        stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        if(withDepth){
            stereo->depth.link(xoutDepth->input);
        }
        else{
            stereo->disparity.link(xoutDepth->input);
        }

        auto colorCam = pipeline.create<dai::node::ColorCamera>(); //returns dai::node:ColorCamera
        auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
        auto xlinkIn = pipeline.create<dai::node::XLinkIn>();

        auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
        auto nnOut = pipeline.create<dai::node::XLinkOut>();
        
        xlinkIn->setStreamName("caminput");
        xlinkOut->setStreamName("preview");
        nnOut->setStreamName("detections");

        colorCam->setPreviewSize(300, 300);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(false);
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        colorCam->setFps(15);

        // testing MobileNet DetectionNetwork
        detectionNetwork->setConfidenceThreshold(0.5f);
        detectionNetwork->setBlobPath(nnPath);

        // Link plugins CAM -> NN -> XLINK
        colorCam->preview.link(detectionNetwork->input); //preview is an array containing the data types need to intercept this somehow
        if(syncNN)
            detectionNetwork->passthrough.link(xlinkOut->input);
        else
            colorCam->preview.link(xlinkOut->input);

        detectionNetwork->out.link(nnOut->input);

        return std::make_tuple(pipeline, width, height);
    }
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)
}   // namespace depthai_examples
