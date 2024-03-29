
#include <ros/ros.h>
#include <string>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "Spinnaker.h"
#include "ros_flir_spinnaker/SpinnakerCamera.hpp"
#include "ros_flir_spinnaker/phmSpinnakerConfig.h"
#include "ros_flir_spinnaker/ExecuteCommand.h"
#include "ros_flir_spinnaker/ExecuteOnOff.h"

phm::SpinnakerCamera cameraHandler;
bool rosServiceStatus = true;

void reconfigureCallBack(ros_flir_spinnaker::phmSpinnakerConfig & config, uint32_t level) {
    cameraHandler.configure(config,level);
}

bool executeCommand (
    ros_flir_spinnaker::ExecuteCommand::Request & req,
    ros_flir_spinnaker::ExecuteCommand::Response & res) {
    res.state = cameraHandler.execute(req.command);
    return true;
}

bool executeTurnOnOff(
    ros_flir_spinnaker::ExecuteOnOff::Request & req,
    ros_flir_spinnaker::ExecuteOnOff::Response & res) {
    res.prev = cameraHandler.executeOnOff(req.command, req.state);
    return true;
}

void initializeParams (ros::NodeHandle & nodeHandle) {
    std::string serial;
    if (nodeHandle.getParam("serial", serial)) {
        ROS_INFO_STREAM("serial " << serial << " is initiated!");
        cameraHandler.setSerial(serial);
    }
}

void interrupt_signal (int sig) {
    ROS_WARN_STREAM("Node is going to be shutdown ...");
    rosServiceStatus = false;
    // All the default SIGINT handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char ** argv) {

    std::string nodeName = "phm_flir_spinnaker";
    // Initialize ROS Node
    ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
    // Initialize the reconfiguration server
    ROS_INFO("Initializing the reconfiguration service ...");
    dynamic_reconfigure::Server<ros_flir_spinnaker::phmSpinnakerConfig> server;
    dynamic_reconfigure::Server<ros_flir_spinnaker::phmSpinnakerConfig>::CallbackType func;
    // Register the callback for configuration changes
    func = boost::bind(&reconfigureCallBack, _1, _2);
    server.setCallback(func);

    ros::NodeHandle nodeHandle;

    signal(SIGINT, interrupt_signal);
    // Initialize Service server
    ROS_INFO("Initializing the service server ...");
    ros::ServiceServer serviceExecuteCommand = nodeHandle.advertiseService(nodeName + "/execute", executeCommand);
    ros::ServiceServer serviceExecuteOnOff = nodeHandle.advertiseService(nodeName + "/on_off", executeTurnOnOff);
    // Define the publisher type and configurations
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Publisher pub = imageTransport.advertise("thermal_camera/image", 1);

    // Initialize parameters of the camera core
    initializeParams(nodeHandle);

    cameraHandler.connect();
    cameraHandler.start();

    while(ros::ok() && rosServiceStatus) {
        sensor_msgs::Image msg;
        Spinnaker::ImageStatus status;
        if (cameraHandler.next(&msg, status) == 0) {
            pub.publish(msg);
        }
        ros::spinOnce();
        if (!rosServiceStatus) {
            break;
        }
    }

    cameraHandler.stop();
    cameraHandler.disconnect();

    ROS_INFO("Good bye ... PHM !!");

    return 0;
}
