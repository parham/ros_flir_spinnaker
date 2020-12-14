
/**
   @file A700.hpp
   @author Parham Nooralishahi
   @date November 29, 2020
   @brief FLIR A700 handler supported by Spinnaker SDK
   @attention Copyright (C) 2020
   @attention MiViM
   @attention Université Laval
**/

#ifndef A700_H
#define A700_H

#include<ros/ros.h>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "ros_flir_spinnaker/Camera.hpp"

namespace phm {
    class A700 : public Camera {
        public: 
            explicit A700(Spinnaker::GenApi::INodeMap *);
            ~A700();
            void configure (const ros_flir_spinnaker::phmSpinnakerConfig &, uint32_t) override;
    };
}

#endif // A700_H