<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/parham/ros_flir_spinnaker">
    <img src="https://www.dsv.ulaval.ca/wp-content/uploads/2018/02/RTEmagicC_06867499dd.gif.gif" alt="Logo" width="320" height="120">
  </a>

  <h3 align="center">LeManchot - FLIR ROS Node for A700 Camera</h3>

  <p align="center">
    <a href="http://mivim.gel.ulaval.ca/?Lang=1"><strong>ULAVAL -- MiViM</strong></a>
    <br/>
    <br/>
  </p>
</p>


<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [FLIR A700 Thermal Camera](#flir-a700)
  * [Built With](#built-with)
* [Prerequisites](#prerequisites)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

## About The Project

LeManchot FLIR Node is a ROS driver for communicating with a Gige-V FLIR thermal cameras (especially FLIR A700) supported by Spinnaker SDK.

### Citation

In case of any use or reference, please cite:

```
@article{nooralishahi2022drone,
  title={Drone-Enabled Multimodal Platform for Inspection of Industrial Components},
  author={Nooralishahi, Parham and L{\'o}pez, Fernando and Maldague, Xavier PV},
  journal={IEEE Access},
  volume={10},
  pages={41429--41443},
  year={2022},
  publisher={IEEE}
}
```

### FLIR A700 Thermal Camera

> FLIR A400 and A700 Science Kits offer researchers and engineers a streamlined solution for accurate temperature measurement. Simplified yet robust connections help you set up and start testing quickly; then easily view, acquire, and analyze data using included FLIR Research Studio software. The Standard kit includes a FLIR A400 or A700 Image Streaming camera, 24° lens with automatic/remote and manual focusing, as well as FLIR Macro Mode. The Professional kit has the added benefits of MSX® image enhancement, radiometric data transmission over Wi-Fi, and a close-up lens for accurate thermal measurements on small components.

## Installation and Initialization

For preparing the node:
``` catkin_make ```

### Node's Configuration

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




