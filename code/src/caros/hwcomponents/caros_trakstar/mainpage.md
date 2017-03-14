\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-generate-toc again -->
**Table of Contents**

- [caros_trakstar](#carostrakstar)
- [Interfaces - how to use this node](#interfaces---how-to-use-this-node)
- [Requirements](#requirements)
- [Launching the node](#launching-the-node)
    - [Parameters](#parameters)
- [Calibration](#calibration)

<!-- markdown-toc end -->

# caros_trakstar #
The Trakstar component is a ros wrapper for the magnetic pose tracker system from TrakStar. It consists of 4 pose trackers and some digital io inputs.

# Interfaces - how to use this node #
The node can be controlled through the @ref caros::PoseSensorServiceInterface.

Clients can make use of the @ref caros::PoseSensorSIProxy to communicate with this node.

# Requirements #
IMPORTANT: the driver needs to be running. This is done by making sure ATCdaemon64 is running. Start it in the terminal and test if the connection is find using the APITest64 which is normally found in "/opt/3DGuidance.Rev.E.64/Binaries".

# Launching the node #

The trakstar node is implemented as a nodelet and needs to be started as such. A standalone node is also available using the executable "caros_trakstar" which wraps the nodelet functionality. The trakstar node implements the Pose sensor interface, button interface and the caros node interface.

To launch the node as a standalone nodelet run:

    roslaunch caros_trakstar caros_trakstar.launch

If the ATCdaemon64 is not running then the node will fail its initialization sequence and enter a caros fatal node error state, a restart of the node and make sure ATCdaemon is running, is required.

After successful launch you can check the rostopic to make sure that the topics are announced:

    rostopic list

The list should include (node name caros_trakstar):

    /caros_trakstar/caros_button_sensor_service_interface/buttons
    /caros_trakstar/caros_node/caros_node_state
    /caros_trakstar/caros_pose_sensor_service_interface/poses

Running the following command writes out the poses of the sensors to the screen:

    rostopic echo /caros_trakstar/caros_pose_sensor_service_interface/poses -c

## Parameters ##
There are a few parameters that matters to the trakstar driver
The following parameters are supported:
| Parameter | Description |
| --------- | ----------- |
|rate| The rate in which pose samples are published with. The trakstar has a limitation depending on version and how many receivers that are used, however when using 4 receivers the fastest sampling rate is approximately 240hz. Default value is 100.|
|frame|The name of the frame in which the samples are described. Default value is TrakstarBase.|

# Calibration  #
TODO: The calibration is still not enabled in this implementation.
