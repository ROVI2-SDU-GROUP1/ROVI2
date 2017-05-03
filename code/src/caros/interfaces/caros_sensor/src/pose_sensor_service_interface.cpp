#include <caros/pose_sensor_service_interface.h>
#include <caros_sensor_msgs/PoseSensorState.h>
#include <caros/common_robwork.h>

#include <vector>

using namespace rw::common;
using namespace caros;

PoseSensorServiceInterface::PoseSensorServiceInterface(const ros::NodeHandle& nodehandle)
    : nodehandle_(nodehandle, POSE_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Nothing specific to do */
}

PoseSensorServiceInterface::~PoseSensorServiceInterface()
{
}

bool PoseSensorServiceInterface::configureInterface()
{
  if (pose_publisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more PoseSensorServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  pose_publisher_ =
      nodehandle_.advertise<caros_sensor_msgs::PoseSensorState>("poses", POSE_SENSOR_POSE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!pose_publisher_, "The PoseSensor poses publisher is empty!");

  /* Verify that the various ROS services have actually been created properly */
  if (pose_publisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM("All PoseSensorServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The PoseSensor service could not be properly initialised - one or more ros services or publishers failed to "
        "be properly initialised.");
    return false;
  }

  return true;
}

void PoseSensorServiceInterface::publishPoses(const std::vector<rw::math::Transform3D<>>& poses,
                                              const std::vector<int>& ids, const std::vector<float>& qualities)
{
  caros_sensor_msgs::PoseSensorState pstate;
  pstate.poses.resize(poses.size());
  pstate.ids.resize(poses.size());
  pstate.qualities.resize(poses.size());

  for (size_t i = 0; i < poses.size(); i++)
  {
    pstate.poses[i] = caros::toRos(poses[i]);
    if (ids.size() <= i)
      pstate.ids[i] = i;
    else
      pstate.ids[i] = ids[i];

    if (qualities.size() <= i)
      pstate.qualities[i] = i;
    else
      pstate.qualities[i] = qualities[i];
  }

  pose_publisher_.publish(pstate);
}
