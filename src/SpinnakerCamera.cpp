
#include <sstream>
#include <pthread.h>
#include <iostream>
#include <string>

#include "ros_flir_spinnaker/SpinnakerCamera.hpp"

phm::SpinnakerCamera::SpinnakerCamera() 
    : serial(""),
     system_(Spinnaker::System::GetInstance()),
     cameraList_(system_->GetCameras()),
     pCamera_(static_cast<int>(NULL)),
     bAcquisitionStarted_(false),
     bCameraInitialized_(false),
     timeOut_(1000)
{
#ifdef __DETAILED_LOG__
    ROS_INFO("Spinnaker Environment is initialized ...");
    ROS_INFO("Camera list is initialized ...");
#endif // __DETAILED_LOG__
}

phm::SpinnakerCamera::~SpinnakerCamera() {
    cameraList_.Clear();
    system_->ReleaseInstance();
#ifdef __DETAILED_LOG__
    ROS_INFO("Spinnaker Environment is uninitialized ...");
    ROS_INFO("Camera list is uninitialized ...");
#endif // __DETAILED_LOG__
}

bool phm::SpinnakerCamera::isInitialized() {
    return bCameraInitialized_;
}

bool phm::SpinnakerCamera::isRunning() {
    return bAcquisitionStarted_;
}

phm::Camera * phm::SpinnakerCamera::createInstance (Spinnaker::GenApi::INodeMap * nodeMap) {
    // Determine camera model and create associated handler
    Spinnaker::GenApi::CStringPtr modelNameNode = nodeMap->GetNode("DeviceModelName");
    std::string modelName(modelNameNode->ToString());

    ROS_INFO_STREAM("Device Model: " << modelName);
    phm::Camera * ptrCamera;
    // Explore and determine the camera handler based on camera model.
    // ---> new camera handlers for other models can be added here to add the support.
    if (!modelName.compare("FLIR A700")) {
        ptrCamera = new phm::A700(nodeMap);
        ROS_INFO("Camera handler is initialized.");
    } else {
        throw std::runtime_error("The Camera model is not supported!");
    }

    return ptrCamera;
}

bool phm::SpinnakerCamera::execute(const std::string & cmd) {
    if (!isInitialized()) {
        ROS_WARN_STREAM("Camera is not initialized for executing " << cmd << " command!");
        return false;
    }

    ROS_INFO_STREAM(cmd << " command is executed!");
    return camera_->executeCommand(cmd);
}

bool phm::SpinnakerCamera::executeOnOff(const std::string & cmd, bool state) {
    if (!isInitialized()) {
        ROS_WARN_STREAM("Camera is not initialized for executing " << cmd << " command!");
        return false;
    }

    bool prev = camera_->getProperty(cmd.c_str());
    if (camera_->setProperty(cmd, state)) {
        ROS_INFO_STREAM(cmd << " turned " << (state ? "on" : "off") << "!");
    }
    return prev;
}

void phm::SpinnakerCamera::connect () {
#ifdef __DETAILED_LOG__
    ROS_INFO("It is connecting to the camera ...");
#endif // __DETAILED_LOG__
    unsigned int numCameras = cameraList_.GetSize();
    if (numCameras == 0) {
        throw std::runtime_error("No camera is detected!");
    }

    if (!bCameraInitialized_) {
        try{
            if (!serial.empty()) {
                ROS_INFO_STREAM("The camera with serial (" << serial << ") is used for connection.");
            } else {
                ROS_INFO("The first camera in the camera list is used for connection.");
            }
            pCamera_ = !serial.empty() ? 
                cameraList_.GetBySerial(serial) :
                cameraList_.GetByIndex(0);
        } catch (Spinnaker::Exception & e) {
            // throw std::runtime_error("[SpinnakerCamera::connect] " + "Is that camera plugged in? Error:" + std::to_string(e.what()));
            std::cerr << "Error: " << e.what() << std::endl;
        }

        if (!pCamera_ || !pCamera_->IsValid()) {
        throw std::runtime_error("[SpinnakerCamera::connect] Failed to obtain camera reference.");
        }
        
        try {
            //////////////////////////////////
            // Initialize Camera
            ROS_INFO("Initializing Camera ...");
            pCamera_->Init();
            bCameraInitialized_ = true;
            // Retrieve GenICam nodemap
            ROS_INFO("Extracting Camera's Node Map ...");
            Spinnaker::GenApi::INodeMap * nodeMap = & pCamera_->GetNodeMap();
            pNodeMap_ = std::unique_ptr<Spinnaker::GenApi::INodeMap>(nodeMap); // std::unique_ptr<Spinnaker::GenApi::INodeMap>();
            ROS_INFO("Camera model and handler is determining ...");
            // Determine camera model and create associated handler
            camera_ = std::unique_ptr<phm::Camera>(phm::SpinnakerCamera::createInstance(nodeMap));

            /////// Device Information /////////////////////
            if (serial.empty() && IsReadable(pCamera_->TLDevice.DeviceSerialNumber)) {
                serial = pCamera_->TLDevice.DeviceSerialNumber.ToString();
            } else {
                ROS_INFO("Camera serial number is not readable!");
            }
            // mapDeviceInfo["DeviceID"] = serial;
            // if (IsReadable(pCamera_->TLDevice.DeviceVendorName)) {
            //     mapDeviceInfo["Vendor"] = pCamera_->TLDevice.DeviceVendorName.ToString();
            // }
            // if (IsReadable(pCamera_->TLDevice.DeviceDisplayName)) {
            //     mapDeviceInfo["DeviceName"] = pCamera_->TLDevice.DeviceDisplayName.ToString();
            // }
            // if (IsReadable(pCamera_->TLDevice.DeviceDriverVersion)) {
            //     mapDeviceInfo["DriverVersion"] = pCamera_->TLDevice.DeviceDriverVersion.ToString();
            // }
            // if (IsReadable(pCamera_->TLDevice.GevDeviceMACAddress)) {
            //     mapDeviceInfo["DeviceMACAddress"] = pCamera_->TLDevice.GevDeviceMACAddress.ToString();
            // }
            // const LibraryVersion libvers = system_->GetLibraryVersion();
            // mapDeviceInfo["LibraryVersion"] = 
            //     std::to_string(libvers.major) + "." + 
            //     std::to_string(libvers.minor) + "." + 
            //     std::to_string(libvers.type) + "." + 
            //     std::to_string(libvers.build);
        } catch (Spinnaker::Exception & e) {
            throw std::runtime_error("Device Information is failed to retrieve : " + std::string(e.what()));
        }
    }

#ifdef __DETAILED_LOG__
    ROS_INFO("It is connected to the camera...");
#endif // __DETAILED_LOG__
}

void phm::SpinnakerCamera::disconnect() {
#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is disconnecting ...");
#endif // __DETAILED_LOG__
    try {
        stop();
        if (isInitialized()) {
            std::lock_guard<std::mutex> scopedLock(mutex_);
            
            if (pCamera_) {
                pCamera_->DeInit();
                pCamera_ = static_cast<int>(NULL);
                cameraList_.RemoveBySerial(serial);
            }
            bCameraInitialized_ = false;
        }
    } catch(const Spinnaker::Exception& e) {
        throw std::runtime_error("Camera is failed to disconnect with error: " + 
            std::string(e.what()));
    }
#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is disconnected ...");
#endif // __DETAILED_LOG__
}

void phm::SpinnakerCamera::configure(const ros_flir_spinnaker::phmSpinnakerConfig & config, uint32_t level) {
    //////////////////////////////////////
    // Configure Camera
    //////////////////////////////////////
    if (!pCamera_) {
        // Connect the camera if is not connected!
        connect();
    }

    // Activate mutex to prevent us from grabbing images during this time
    std::lock_guard<std::mutex> scopedLock(mutex_);

    if (level >= RECONFIGURE_STOP) {
        ROS_DEBUG("new configuration received : Reconfigure Stop.");
        bool tmpflag = isInitialized();
        stop();
        camera_->configure(config, level);
        if (tmpflag) {
            start();
        }
    } else {
        camera_->configure(config,level);
    }
}

void phm::SpinnakerCamera::start() {
#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is starting the acquisition process ...");
#endif // __DETAILED_LOG__
    if (!isInitialized()) {
        throw std::runtime_error("The connection is not initialized!");
    }

    if (!isRunning()) {
        //////////////////////////////////////
        // Start the acquisition process
        //////////////////////////////////////
        // Set acquisition mode to continuous
        if (!IsReadable(pCamera_->AcquisitionMode) || !IsWritable(pCamera_->AcquisitionMode)) {
            std::cerr << "Unable to set acquisition mode to continuous. Aborting..." << std::endl;
            return;
        }
        pCamera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
        ROS_INFO("Acquisition mode set to continuous...");
        // Begin acquiring images
        pCamera_->BeginAcquisition();
        bAcquisitionStarted_ = true;
    }

#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is started the acquisition process ...");
#endif // __DETAILED_LOG__
}

void phm::SpinnakerCamera::stop() {
#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is stopping the acquisition process ...");
#endif // __DETAILED_LOG__
    if(isRunning()) {
        try {
            bAcquisitionStarted_ = false;
            pCamera_->EndAcquisition();
        } catch(const Spinnaker::Exception & ex) {
            throw std::runtime_error("The acquisition process is failed to stop : " + std::string(ex.what()));
        }
    }
#ifdef __DETAILED_LOG__
    ROS_INFO("The camera is stopped the acquisition process ...");
#endif // __DETAILED_LOG__
}

void phm::SpinnakerCamera::setSerial(std::string s) {
    if (s.empty()) {
        throw new std::invalid_argument("Given serial is invalid!");
    }
    serial = s;
}

std::string phm::SpinnakerCamera::getSerial() {
    return serial;
}

void phm::SpinnakerCamera::setTimeOut(long to) {
    if (to < 0) {
        throw new std::invalid_argument("Time out is invalid!");
    }
    timeOut_ = to;
}

long phm::SpinnakerCamera::getTimeOut() {
    return timeOut_;
}

int phm::SpinnakerCamera::next(sensor_msgs::Image * frame, Spinnaker::ImageStatus & status) {
    int retcode = 0;
    if (isInitialized() && isRunning()) {
        std::lock_guard<std::mutex> scopedLock(mutex_);
        try {
            // Retrieve next frame
            ImagePtr img = pCamera_->GetNextImage(timeOut_);
            status = img->GetImageStatus();
            if (img->IsIncomplete()) {
                ROS_ERROR_STREAM("Image incomplete with image status : " << status);
                retcode = -3;
            } else {
                ////// Retrieve meta data
                // Set Image Time Stamp
                frame->header.stamp = ros::Time::now();
                frame->header.seq++;
                // frame->header.stamp.sec = img->GetTimeStamp() * 1e-9;
                frame->header.stamp.nsec = img->GetTimeStamp();
                // Check the bits per pixel
                size_t bitsPerPixel = img->GetBitsPerPixel();
                // Set the image encoding
                std::string imageEncoding;
                switch(img->GetPixelFormat()) {
                    case PixelFormat_Mono8:
                        imageEncoding = sensor_msgs::image_encodings::MONO8;
                        break;
                    case PixelFormat_Mono16:
                        imageEncoding = sensor_msgs::image_encodings::MONO16;
                        break;
                    default:
                        throw std::runtime_error("The pixel format is not supported!");
                }

                int width = img->GetWidth();
                int height = img->GetHeight();
                int stride = img->GetStride();
                // Fill the image
                sensor_msgs::fillImage(* frame, imageEncoding, 
                    height, width, stride, img->GetData());
                frame->header.frame_id = std::to_string(img->GetFrameID());
            }
            // Release the collected frame
            img->Release();
        } catch (Spinnaker::Exception &e) {
            std::cerr << "Error: " << e.what() << std::endl;
            retcode = -2;
        }
    }

    return retcode;
}
