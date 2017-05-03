# Represents the state of a pose sensor.

# Header containing information about time and frameid
Header header

# Poses of the sensor
geometry_msgs/Transform[] poses

# IDs of the poses
uint32[] ids

# Quality of the sensor pose measurements [0;1] 0=invalid, 1=high quality.
float32[] qualities
