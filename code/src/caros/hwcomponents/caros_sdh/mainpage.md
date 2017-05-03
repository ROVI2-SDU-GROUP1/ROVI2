\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_sdh](#carossdh)
- [Interfaces - how to use this node](#interfaces---how-to-use-this-node)
- [Requirements](#requirements)
- [Launching the node](#launching-the-node)
    - [Parameters](#parameters)
        - [Status of RS232 support](#status-of-rs232-support)
    - [Using debug verbosity](#using-debug-verbosity)

<!-- markdown-toc end -->

# caros_sdh #
caros_sdh is a ROS node for controlling the (physical) 3-finger SCHUNK dexterous hand.

# Interfaces - how to use this node #
The node can be controlled through the @ref caros::GripperServiceInterface (**However please notice that not all the service callbacks comply with the units specified in that interface**).

Clients can make use of the @ref caros::GripperSIProxy to communicate with this node.

# Requirements #
- RobWorkHardware with the *sdh* component enabled, is required and can be obtained from http://www.robwork.dk
    - SDH library. Currently this library doesn't appear to be officially publicly available.
    - PEAK CAN driver and library, which can be obtained from http://www.peak-system.com/fileadmin/media/linux/index.htm

# Launching the node #
The CAROS SDH node can be launched by using the following:

    roslaunch caros_sdh caros_sdh.launch

## Parameters ##
The following parameters are supported:
| Parameter | Description |
| --------- | ----------- |
| interface_type | The communication interface / protocol to use (currently CAN and RS232 are supported) |
| rs232_device | The device-file to use on the Linux system for RS232 communication |
| rs232_port | The port (e.g. 1 to 4) |
| rs232_baudrate | The baudrate |
| rs232_timeout | Timeout (e.g. 0.5) |
| can_device | The device-file to use on the Linux system for CAN communication |
| can_baudrate | The baudrate |
| can_timeout | Timeout (e.g. 0.5) |

### Status of RS232 support ###
The node has not yet been tested using RS232 communication.

## Using debug verbosity ##
To enable debug verbosity and thus hopefully make it easier to diagnose issues, then a rosconsole debug configuration file has to be present and specified - see https://gitlab.com/caro-sdu/caros/wikis/Tests#example-rosconsole_debug-conf for more information.
