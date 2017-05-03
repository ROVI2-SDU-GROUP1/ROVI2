#include <caros/ft_sensor_service_interface.h>
#include <caros/common.h>

#include <geometry_msgs/WrenchStamped.h>

#include <string>

using namespace rw::common;
using namespace caros;

FTSensorServiceInterface::FTSensorServiceInterface(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle, FT_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Nothing specific to do */
}

FTSensorServiceInterface::~FTSensorServiceInterface()
{
  /* Nothing specific to do */
}

bool FTSensorServiceInterface::configureInterface()
{
  if (wrench_data_publisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more FTSensorServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  wrench_data_publisher_ =
      nodehandle_.advertise<geometry_msgs::WrenchStamped>("wrench", FT_SENSOR_WRENCH_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!wrench_data_publisher_, "The FTSensor wrench publisher is empty!");

  /* Verify that the various ROS services have actually been created properly */
  if (wrench_data_publisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM("All FTSensorServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The FTSensor service could not be properly initialised - one or more ros services or publishers failed to be "
        "properly initialised.");
    return false;
  }

  return true;
}

void FTSensorServiceInterface::publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe)
{
  geometry_msgs::WrenchStamped wdata;

  wdata.header.frame_id = refframe;
  wdata.header.stamp = ros::Time::now();

  wdata.wrench.force.x = wrench(0);
  wdata.wrench.force.y = wrench(1);
  wdata.wrench.force.z = wrench(2);

  wdata.wrench.torque.x = wrench(3);
  wdata.wrench.torque.y = wrench(4);
  wdata.wrench.torque.z = wrench(5);

  wrench_data_publisher_.publish(wdata);
}
