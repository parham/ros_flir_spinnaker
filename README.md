<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/parham/ros_flir_spinnaker">
    <img src="https://www.dsv.ulaval.ca/wp-content/uploads/2018/02/RTEmagicC_06867499dd.gif.gif" alt="Logo" width="320" height="120">
  </a>

  <h3 align="center">PHM FLIR Spinnaker Driver for ROS</h3>

  <p align="center">
    <a href="http://mivim.gel.ulaval.ca/?Lang=1"><strong>ULAVAL -- MiViM</strong></a>
    <br/>
    <br/>
  </p>
</p>

#Parent project: @Le Manchot

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [FLIR A700 Thermal Camera](#flir-a700)
  * [Built With](#built-with)
* [Prerequisites](#prerequisites)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

## About The Project

The work is a ros driver for Gige-V FLIR thermal cameras (especially FLIR A700) supported by Spinnaker SDK.

For prepare the node:
``` catkin_make ```

### FLIR A700 Thermal Camera

FLIR A400 and A700 Science Kits offer researchers and engineers a streamlined solution for accurate temperature measurement. Simplified yet robust connections help you set up and start testing quickly; then easily view, acquire, and analyze data using included FLIR Research Studio software. The Standard kit includes a FLIR A400 or A700 Image Streaming camera, 24° lens with automatic/remote and manual focusing, as well as FLIR Macro Mode. The Professional kit has the added benefits of MSX® image enhancement, radiometric data transmission over Wi-Fi, and a close-up lens for accurate thermal measurements on small components.

### FLIR ROS Node's Configuration

In order to change the configurations of FLIR node, the following command can be used:

``` 
rosrun rqt_gui rqt_gui -s reconfigure
```

### Built With
* [ROS](https://www.ros.org)
* camera_info_manager
* dynamic_reconfigure
* image_proc
* cv_bridge
* image_transport
* nodelet
* rospy
* roscpp
* roslaunch
* sensor_msgs
* message_generation

## Contact
Parham Nooralishahi - parham.nooralishahi@gmail.com | [@phm](https://www.linkedin.com/in/parham-nooralishahi/) <br/>




