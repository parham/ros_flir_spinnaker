
#include "ros_flir_spinnaker/A700.hpp"

phm::A700::A700(Spinnaker::GenApi::INodeMap * node) : phm::Camera(node) {
    // Empty body
}

phm::A700::~A700() {
    // Empty body
}

void phm::A700::configure (const ros_flir_spinnaker::phmSpinnakerConfig & config, uint32_t level) {
    try {
        // CLOSE & STOP Configuration
        ROS_INFO(">>> FLIR A700 CONFIGURATIONS");
        if (level > RECONFIGURE_RUNNING) {
            ROS_INFO("STOP CONFIGURATIONS ...");
            setProperty("AcquisitionFrameRate", config.acquisition_frame_rate);
            ROS_INFO_STREAM("AcquisitionFrameRate" << " : " << config.acquisition_frame_rate);
            // Image Format Control
            ROS_INFO("IMAGE FORMAT CONTROL CONFIGURATIONS ...");
            setProperty("Width", config.frame_width);
            ROS_INFO_STREAM("Width" << " : " << config.frame_width);
            setProperty("Height", config.frame_height);
            ROS_INFO_STREAM("Height" << " : " << config.frame_height);
            setProperty("OffsetX", config.frame_x_offset);
            ROS_INFO_STREAM("OffsetX" << " : " << config.frame_x_offset);
            setProperty("OffsetY", config.frame_y_offset);
            ROS_INFO_STREAM("OffsetY" << " : " << config.frame_y_offset);
            setProperty("PixelFormat", config.frame_pixel_format);
            ROS_INFO_STREAM("PixelFormat" << " : " << config.frame_pixel_format);
            setProperty("ImageMode", config.image_mode);
            ROS_INFO_STREAM("ImageMode" << " : " << config.image_mode);
            // Chunk Data Control
            ROS_INFO("CHUNK DATA CONTROL CONFIGURATIONS ...");
            setProperty("ChunkModeActive", config.chunk_mode_active);
            ROS_INFO_STREAM("ChunkModeActive" << " : " << config.chunk_mode_active);
            setProperty("ChunkEnable", config.chunk_enable);
            ROS_INFO_STREAM("ChunkEnable" << " : " << config.chunk_enable);
        }

        ROS_INFO("RUNNING CONFIGURATIONS ...");
        // Running Configuration
        // Focus Control
        ROS_INFO("FOCUS CONTROL CONFIGURATIONS ...");
        setProperty("FocusDirection", config.focus_direction);
        ROS_INFO_STREAM("FocusDirection" << " : " << config.focus_direction);
        setProperty("AutoFocusMethod", config.focus_method);
        ROS_INFO_STREAM("AutoFocusMethod" << " : " << config.focus_method);
        setProperty("ImageAdjustMode", config.adjust_mode);
        ROS_INFO_STREAM("ImageAdjustMode" << " : " << config.adjust_mode);
        setProperty("Torch", config.lens_torch);
        ROS_INFO_STREAM("Torch" << " : " << config.lens_torch);
        
        // executeCommand("AutoFocus");
    } catch (const Spinnaker::Exception & e) {
        throw std::runtime_error("[Camera::setNewConfiguration] Failed to set configuration: " + std::string(e.what()));
    }
}
