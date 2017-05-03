\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_robotiq2/3](#carosrobotiq2/3)
- [Interfaces - how to use this node](#interfaces---how-to-use-this-node)
    - [Gripper Service Interface](#gripper-service-interface)
- [Requirements](#requirements)
- [Launching the node](#launching-the-node)
    - [Parameters](#parameters)
- [Small demo(s)](#small-demos)
    - [Available demo(s)](#available-demos)
    - [Launching the demo(s)](#launching-the-demos)
        - [Using debug verbosity](#using-debug-verbosity)

<!-- markdown-toc end -->

# caros_robotiq2/3 #
The caros_robotiq2 and caros_robotiq3 ROS nodes are for controlling a physical Robotiq2 or Robotiq3 hand.

While we would not be surprised if these nodes work for other robotics hands we have only tested them with "2-Finger Adaptive Robot Gripper, C-Model, controller version 3.0" and "3-Finger Adaptive Robot Gripper, version S-6". 

# Interfaces - how to use this node #

## Gripper Service Interface ##
The @ref caros::GripperServiceInterface interface is supported by the nodes but units are not. THIS NODE DOES NOT COMPLY TO THE UNITS SPECIFIED IN caros::GripperServiceInterface. See below for values used.

The Robotiq2 hand has one joint, the Robotiq3 has four. Therefore all service calls expecting input of type Q need to be provided one or four dimensional Qs. All Q values should be between 0 and 255 both values inclusive (see Robotiq datasheets, SDU's robotiq2 controller version 3.0 - http://support.robotiq.com/download/attachments/1181172/2-FINGER-85-130206.pdf?version=1&modificationDate=1407264261000 robotiq3 version S-6 - http://support.robotiq.com/download/attachments/1181172/IMB-311012.pdf?version=1&modificationDate=1351707795000 ).

Clients can make use of the @ref caros::GripperSIProxy to communicate with this node.

# Requirements #
RobWorkHardware with the *robotiq* component enabled, is required and can be obtained from http://www.robwork.dk

# Launching the node #
The CAROS Robotiq nodes can be launched by using one of the following:

    roslaunch caros_robotiq caros_robotiq3.launch
    roslaunch caros_robotiq caros_robotiq2.launch

The launch scripts are using the default ip and port used in the marvin setup. This can be changed in the hwcomponents/caros_robotiq/launch/caros_robotiq3_param.xml and hwcomponents/caros_robotiq/launch/caros_robotiq2_param.xml files.

## Parameters ##
The following parameters are supported:
| Parameter | Description | Default |
| --------- | ----------- | ------- |
| ip | IP of the hand to control | 192.168.100.21 for robotiq3 and 192.168.100.22 for robotiq2|
| port | The modbus communication port used | 502 |

# Small demo(s) #
To quickly and easily verify that the communication with one of the hands is working, there is one simple demo for each device that can be run. The expected behaviour should be both observed and verified by the user.

## Available demo(s) ##
| Demo | Expected behaviour | Notes |
| ---- | ------------------ | ----- |
| simple_robotiq2_demo | Initialising the Robotiq2 hand and moving the fingers to a position. | None |
| simple_robotiq3_demo | Initialising the Robotiq3 hand and moving the fingers to a position. | None |

## Launching the demo(s) ##
In order to make ROS properly find the demos, then the <your_catkin_workspace>/devel/setup.bash file should be sourced. If standing in your catkin workspace then it's as simple as (if you are using BASH or similar shell - default on Ubuntu):

    source devel/setup.bash

To launch the demos:

    roslaunch caros_robotiq <demo name>.test

For example to launch the simple_robotiq3_demo:

    roslaunch caros_robotiq simple_robotiq3_demo.test

### Using debug verbosity ###
To enable debug verbosity and thus hopefully make it easier to diagnose issues, then a rosconsole debug configuration file has to be present (see https://gitlab.com/caro-sdu/caros/wikis/Tests#example-rosconsole_debug-conf):

    ROSCONSOLE_CONFIG_FILE=/path/to/rosconsole_debug.conf roslaunch simple_robotiq3_demo.test
