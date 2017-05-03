\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_sensor](#carossensor)
- [Interfaces](#interfaces)
    - [Button Sensor Service Interface](#button-sensor-service-interface)
    - [Force/Torque (FT) Sensor Service Interface](#forcetorque-ft-sensor-service-interface)
    - [Pose Sensor Service Interface](#pose-sensor-service-interface)

<!-- markdown-toc end -->

# caros_sensor #
This component contains interfaces for communicating with various types of sensors. Examples hereof are camera sensors, RGB-D sensors, force/torque sensors and tactile array sensors.

# Interfaces #
All interfaces in here are currently just shells. Nothing is implemented close to hardware.

## Button Sensor Service Interface ##
The @ref caros::ButtonSensorServiceInterface represents a simple button sensor for an array of analog or digital buttons. It has one output topic:
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| buttons  | @ref caros_sensor_msgs::ButtonSensorState | State of all analog/digital buttons |

Clients can make use of the @ref caros::ButtonSensorSIProxy to communicate with nodes implementing this service interface.

## Force/Torque (FT) Sensor Service Interface ##
The @ref caros::FTSensorServiceInterface uses an output topic to publish the current 6D FT reading, encoded as a wrench.
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| wrench  | @ref geometry_msgs::WrenchStamped | 6D vector of current force and torque reading |

Clients can make use of the @ref caros::FTSensorSIProxy to communicate with nodes implementing this service interface.

## Pose Sensor Service Interface ##
The @ref caros::PoseSensorServiceInterface uses a physical device to track one or more poses of objects. Tracked poses are published in a topic.
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| poses  | @ref caros_sensor_msgs::PoseSensorState | Vector of poses, each encoded as a geometry_msgs::Transform |

Clients can make use of the @ref caros::PoseSensorSIProxy to communicate with nodes implementing this service interface.

## Camera Service Interface ##
The @ref caros::CameraServiceInterface uses one or two camera image publishers, to publish the images from the camera to a ROS topic or ROS topics.
Furthermore it uses a PointCloud publisher to publish pointclouds.
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| namespace/image_raw  | @ref sensor_msgs::Image | Image, encoded as described here [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) |
| left/image_raw  | @ref sensor_msgs::Image | Image, encoded as described here [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) |
| right/image_raw  | @ref sensor_msgs::Image | Image, encoded as described here [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) |
| left/pointcloud  | @ref sensor_msgs::PointCloud2 | PointCloud, encoded as described here [sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) |
