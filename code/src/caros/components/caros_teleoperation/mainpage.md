\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-generate-toc again -->
**Table of Contents**

- [caros_teleoperation](#carosteleoperation)
    - [Parameters](#parameters)
- [Launching the node](#launching-the-node)

<!-- markdown-toc end -->

# caros_teleoperation #
This component can be used for controlling a robot using teleoperation. The component should be initialized with the 
available workcell descriptions of robot and the environment calibrated in order to avoid collision during movements.
This component connects a rosnode providing pose information, e.g. caros_trakstar, to a serial device node, e.g. caros_universalrobot.


First you have to change the parameters in the launchfile: caros_teleoperation/launch/caros_teleopetartion.launch

\verbatim
	<launch>
	  <group>
	    <node pkg="caros_teleoperation" type="teleoperation" name="caros_teleoperation" output="screen">
	      <param name="robot" value="UR5"/>
	      <param name="rate" value="100"/>
	      <param name="PushSensor" value="/caros_trakstar/caros_button_sensor_service_interface/buttons"/>
	      <param name="PoseArray" value="/caros_trakstar/caros_pose_sensor_service_interface/poses"/>
	      <param name="PoseIdx" value="3"/>
	    </node>
	  </group>
	</launch>
\endverbatim
	
## Parameters ##
There are a few parameters that matters to the trakstar driver
The following parameters are supported:
 Parameter | Description | Default |
| --------- | ----------- | ------- |
| robot | Name of the serial device in the workcellfile | UR5 |
| rate | Maximum rate of the control loop | 100 |
| PushSensor | Topic name of the deadmans button enabling the teleoperation | /caros_trakstar/caros_button_sensor_service_interface/buttons |
| PoseArray | Topic name of pose data array| /caros_trakstar/caros_pose_sensor_service_interface/poses |
| PoseIdx | Indexnumber of the pose data array used for control | 3 last sensors of the trakstar |

# Launching the node #
Preconditions:
You have to launch a robot e.g. caros_universalrobot and a pose sensor e.g. caros_trakstar. 
When these nodes are running you can launch this node: 

    roslaunch caros_teleoperation caros_teleoperation.launch

A successful launch can be checked by rqt_graph. If you are using trakstar sensor to control an UR robot, the graph should look like this:
<img src="../../../assets/teleoperation.png" width="600" height="400" />

As seen in the image from rqt_graph, 3 nodes have been launched. These are a caros_trakstar node, caros_universalrobot node (UR5) and a caros_teleopration node.
The caros_teleoperation node is connected to the caros_trakstar node via two topics: /caros_trakstar/caros_button_sensor_service_interface/buttons and /caros_trakstar/caros_pose_sensor_service_interface/poses. It is connected to the caros_universalrobot topic: /UR5/caros_serial_device_service_interface/robot_state.
