\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-generate-toc again -->
**Table of Contents**

- [caros_netft](#carosnetft)
- [Interfaces - how to use this node](#interfaces---how-to-use-this-node)
    - [Force Torque Sensor Service Interface](#force-torque-sensor-service-interface)
- [Requirements](#requirements)
- [Launching the node](#launching-the-node)
    - [Parameters](#parameters)
- [Small demo(s)](#small-demos)

<!-- markdown-toc end -->

# caros_netft #
This component provides access to a Schunk NetFT Force/Torque sensor.

The Force/Torque sensors measures forces and torques in its local frame of reference.
Units are Newton and Newton Meter, respectively.

**NOTICE**: The sensor is not calibrated to have a zero offset. Furthermore the sensor has a tendency to drift over time. The users of the components need to take care of this themselves.

# Interfaces - how to use this node #
This node publishes sensor readings using the @ref caros::FTSensorServiceInterface interface.

Clients can make use of the @ref caros::FTSensorSIProxy to communicate with this node.

# Requirements #
RobWorkHardware with the *netft* component enabled, is required and can be obtained from http://www.robwork.dk

# Launching the node #
The CAROS NetFT node can be launched by using the following:

    roslaunch caros_netft caros_netft.launch

## Parameters ##
The following parameters are supported:
| Parameter | Description | Default |
| --------- | ----------- | ------- |
| ip | IP of the netft to read from | 168.192.100.2 |
| port | The modbus communication port used | 49152 |
| rate | Rate (Hz) with which data is published | 100 |

# Small demo(s) #
To quickly and easily verify that the component start up the component using the launch script (see above) and use rqt to view the topic *wrench* and the *caros_node_state*.
