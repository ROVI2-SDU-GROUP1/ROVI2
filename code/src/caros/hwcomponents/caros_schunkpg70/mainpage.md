\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_schunkpg70](#carosschunkpg70)
- [Interfaces - how to use this node](#interfaces---how-to-use-this-node)
    - [Gripper Service Interface](#gripper-service-interface)
- [Requirements](#requirements)
- [Launching the node](#launching-the-node)
    - [Parameters](#parameters)
    - [Using debug verbosity](#using-debug-verbosity)
- [Notes](#notes)

<!-- markdown-toc end -->

# caros_schunkpg70 #
caros_schunkpg70 is a ROS node for controlling the (physical) SCHUNK 2-finger parallel gripper PG 70.

# Interfaces - how to use this node #
The node can be controlled through the @ref caros::GripperServiceInterface.

For @ref caros::GripperServiceInterface::setForceQ q values between 0.0 and 200.0 N are supported. The relationship between the specified force and the actual applied force, is only approximate (see the source code for more specific information).

## Gripper Service Interface ##
The @ref caros::GripperServiceInterface interface is supported to some extent. The functionalities that are not supported can be seen below:
| Service | Not Supported |
| ------- | :-------------: |
| set_velocity_q | x |

# Requirements #
- RobWorkHardware with the *schunkpg70* component enabled, is required and can be obtained from http://www.robwork.dk

# Launching the node #
The CAROS schunkpg70 node can be launched by using the following:

    roslaunch caros_schunkpg70 caros_schunkpg70.launch

## Parameters ##
The following parameters are supported:
| Parameter | Description |
| --------- | ----------- |
| serial_device | The device to use for the serial communication |

## Using debug verbosity ##
To enable debug verbosity and thus hopefully make it easier to diagnose issues, then a rosconsole debug configuration file has to be present and specified - see https://gitlab.com/caro-sdu/caros/wikis/Tests#example-rosconsole_debug-conf for more information.

# Notes #
Currently when using the service _move\_q_, the node will become unresponsive until the physical movement has finished. This is due to how the driver is implemented in RobWorkHardware.
