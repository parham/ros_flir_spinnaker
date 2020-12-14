/**
   @file exception.hpp
   @author Parham Nooralishahi
   @date December 09, 2020
   @attention Copyright (C) 2020
   @attention MiViM
   @attention Université Laval
*/

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <stdexcept>
#include <string>

/** Partially adopted from https://github.com/ros-drivers/flir_camera_driver/blob/kinetic-devel/spinnaker_camera_driver/include/spinnaker_camera_driver/camera_exceptions.h **/

namespace phm {
    class CameraTimeoutException : public std::runtime_error {
        public:
            CameraTimeoutException() : runtime_error("Image not found within timeout.") { } 
            explicit CameraTimeoutException(const std::string& msg) : runtime_error(msg.c_str()) { } 
    };

    class CameraNotRunningException : public std::runtime_error {
        public:
            CameraNotRunningException() : runtime_error("Camera is currently not running.  Please start the capture.") { } 
            explicit CameraNotRunningException(const std::string& msg) : runtime_error(msg.c_str()) { } 
    };

    class CameraImageNotReadyException : public std::runtime_error {
        public:
            CameraImageNotReadyException() : runtime_error("Image is currently not ready.") { } 
            explicit CameraImageNotReadyException(const std::string& msg) : runtime_error(msg.c_str()) { } 
    };
}

/********************************/

#endif // EXCEPTION_H