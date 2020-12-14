
/**
   @file Camera.hpp
   @author Parham Nooralishahi
   @date November 29, 2020
   @brief A core handler of the cameras supported by Spinnaker SDK
   @attention Copyright (C) 2020
   @attention MiViM
   @attention Université Laval
*/

#ifndef CAMERA_H
#define CAMERA_H

#include<ros/ros.h>
#include<string>
#include<exception>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "ros_flir_spinnaker/phmSpinnakerConfig.h"

#define RECONFIGURE_RUNNING 0
#define RECONFIGURE_STOP 1
#define RECONFIGURE_CLOSE 3

namespace phm {
   class Camera {
      protected:
         std::unique_ptr<Spinnaker::GenApi::INodeMap> pNodeMap;
         int iMaxHeight;
         int iMaxWidth;
         std::string strModelName;
      public:
         explicit Camera(Spinnaker::GenApi::INodeMap *);
         ~Camera();
      public:
         std::string getModelName();
         void initialize ();
         virtual void configure (const ros_flir_spinnaker::phmSpinnakerConfig &, uint32_t);
      public:
         bool executeCommand(const std::string &);
         Spinnaker::GenApi::CNodePtr getProperty(const Spinnaker::GenICam::gcstring);
         bool setProperty(const std::string &, const std::string &);
         bool setProperty(const std::string &, const double &);
         bool setProperty(const std::string &, const bool &);
         bool setProperty(const std::string &, const int &);
         bool setMaxInt(const std::string &);
   };
}

#endif // CAMERA_H