
/**
   @file SpinnakerCamera.hpp
   @author Parham Nooralishahi
   @date November 29, 2020
   @brief A wrapper for Spinnaker SDK as part of Multimodal imagery data platform for drone-enabled inspection of industrial sites
   @attention Copyright (C) 2020
   @attention MiViM
   @attention Université Laval
*/

#ifndef SPINNAKER_CAMERA_H
#define SPINNAKER_CAMERA_H

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "ros_flir_spinnaker/Camera.hpp"
#include "ros_flir_spinnaker/A700.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>            // ROS message header for Image
#include <sensor_msgs/image_encodings.h>  // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>

#include <map>
#include <mutex>
#include <memory>

#define __DETAILED_LOG__

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace phm {

    class SpinnakerCamera {
        private:
            SystemPtr system_;
            CameraList cameraList_;
            CameraPtr pCamera_;
            std::unique_ptr<Camera> camera_;
            std::unique_ptr<INodeMap> pNodeMap_;
            bool bCameraInitialized_;
            bool bAcquisitionStarted_;
            std::mutex mutex_; // The mutex is explicitly used for restricting grabbing access while reconfiguring.
            long timeOut_;
        public: 
            static Camera * createInstance (Spinnaker::GenApi::INodeMap *);
        private: // Configuration
            std::string serial;
        public:
            void configure(const ros_flir_spinnaker::phmSpinnakerConfig &, uint32_t);
        public:
            SpinnakerCamera();
            virtual ~SpinnakerCamera();
        public:
            void setSerial(std::string);
            std::string getSerial();
            void setTimeOut(long);
            long getTimeOut();
        public:
            bool isInitialized();
            bool isRunning();
        public:
            bool execute(const std::string &);
            bool executeOnOff(const std::string &, bool); 
            void connect();
            void disconnect();
            void start();
            void stop();
        public:
            int next (sensor_msgs::Image *, ImageStatus &);
    };
}

#endif // SPINNAKER_CAMERA_H