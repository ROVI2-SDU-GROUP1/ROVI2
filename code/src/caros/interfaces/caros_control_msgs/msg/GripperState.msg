# GripperState represents the state of a single gripper in the setup.

# Header containing information about time and frameid
Header header

# Joint configuration.
# Angles should be represented as radians and distances in meters.
caros_common_msgs/Q q

# Velocities should be represented as radians per sec
caros_common_msgs/Q dq

# Approximate force applied by the gripper joints (most implementations simply output the measured current)
caros_common_msgs/Q force

# Is the gripper trying to move to a target (notice it can still be blocked even if isMoving is true)
bool is_moving

# If the gripper is moving then it may be blocked.
bool is_blocked

# If the gripper reached its target then isStopped is true
bool is_stopped

# Is Emergency Stopped
bool e_stopped

