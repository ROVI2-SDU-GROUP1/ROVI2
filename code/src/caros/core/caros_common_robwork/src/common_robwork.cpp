#include <caros/common_robwork.h>

#include <rw/math.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <caros_common_msgs/Q.h>
#include <caros_common_robwork_msgs/GetRwState.h>
#include <caros_common_robwork_msgs/RwState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>

#include <string>

namespace caros
{
/************************************************************************
 * Notes:
 * These conversion functions can be templated to work with other scalar
 * types than the default 'double'.
 * But sticking with the default to keep things simple.
 ************************************************************************/

/************************************************************************
 * State
 ************************************************************************/
caros_common_robwork_msgs::RwState toRos(const rw::kinematics::State& state)
{
  ROS_DEBUG_STREAM("RobWork state to ROS RwState -begin-");
  caros_common_robwork_msgs::RwState res;
  /* Preallocate the size */
  auto state_data_size = state.getStateStructure()->getStateData().size();
  ROS_DEBUG_STREAM("state_data_size: " << state_data_size);
  res.state_data.reserve(state_data_size);

  for (const auto& data : state.getStateStructure()->getStateData())
  {
    caros_common_robwork_msgs::RwStateData res_data;
    res_data.name = data->getName();
    /* This hardcoded sizeof should be provided by the 'data->' object, as the type used for storing the state data is
     * highly implementation specific */
    res_data.size = data->size() * sizeof(double);
    ROS_DEBUG_STREAM("res_data name: " << res_data.name);
    ROS_DEBUG_STREAM("res_data size: " << res_data.size << " where data->size = " << data->size()
                                       << " and sizeof(double) = " << sizeof(double));
    /* Preallocate the size */
    res_data.data.reserve(res_data.size);

    for (std::size_t index = 0; index < res_data.size; ++index)
    {
      /* Converting to uint8_t because the data in the state should just be considered a memory space and not
       * necessarily consist of doubles, as it could as well be booleans and other sort of types */
      std::uint8_t binary_data = reinterpret_cast<const std::uint8_t*>(data->getData(state))[index];
      ROS_DEBUG_STREAM("binary_data[" << index << "] = " << std::hex << std::setw(2)
                                      << static_cast<std::uint32_t>(binary_data));
      res_data.data.push_back(binary_data);
    }
    res.state_data.push_back(res_data);
  }

  ROS_DEBUG_STREAM("RobWork state to ROS RwState -end-");
  return res;
}

void toRw(const caros_common_robwork_msgs::RwState& state_ros, rw::kinematics::State& state)
{
  ROS_DEBUG_STREAM("ROS RwState to RobWork state -begin-");
  for (const auto& state_ros_data : state_ros.state_data)
  {
    auto data = state.getStateStructure()->findData(state_ros_data.name);
    if ((data->size() * sizeof(double)) == state_ros_data.data.size())
    {
      ROS_DEBUG_STREAM("data name: " << state_ros_data.name);
      ROS_DEBUG_STREAM("data size: " << state_ros_data.data.size());
      // Place the state_ros_data.data in a continuous memory space
      std::uint8_t* raw_data = new std::uint8_t[state_ros_data.data.size()];
      for (std::size_t index = 0; index < state_ros_data.data.size(); ++index)
      {
        raw_data[index] = state_ros_data.data.at(index);
        ROS_DEBUG_STREAM("raw_data[" << index << "] = " << std::hex << std::setw(2)
                                     << static_cast<std::uint32_t>(raw_data[index]));
      }
      data->setData(state, reinterpret_cast<const double*>(raw_data));
      delete[] raw_data;
    }
    else
    {
      ROS_ERROR_STREAM(
          "Mismatch in data (i.e. payload) lengths between current state and the serialised state from ROS: "
          "ros_state_name="
          << state_ros_data.name << " ros_state_size=" << state_ros_data.data.size() * sizeof(double)
          << " current_state_size=" << data->size());
    }
  }
  ROS_DEBUG_STREAM("ROS RwState to RobWork state -end-");
}

rw::kinematics::State toRw(const caros_common_robwork_msgs::RwState& state_ros, const rw::models::WorkCell::Ptr wc)
{
  rw::kinematics::State state = wc->getDefaultState();

  toRw(state_ros, state);

  return state;
}

/************************************************************************
 * Q
 ************************************************************************/
rw::math::Q toRw(const caros_common_msgs::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

caros_common_msgs::Q toRos(const rw::math::Q& q)
{
  caros_common_msgs::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

/************************************************************************
 * Transform
 ************************************************************************/
geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform)
{
  rw::math::Quaternion<> q(transform.R());
  geometry_msgs::Transform result;
  result.translation.x = transform.P()(0);
  result.translation.y = transform.P()(1);
  result.translation.z = transform.P()(2);

  result.rotation.x = q(0);
  result.rotation.y = q(1);
  result.rotation.z = q(2);
  result.rotation.w = q(3);

  return result;
}

rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform)
{
  rw::math::Vector3D<> p(transform.translation.x, transform.translation.y, transform.translation.z);
  rw::math::Quaternion<> q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  return rw::math::Transform3D<>(p, q);
}

/************************************************************************
 * Pose
 *
 * Because the RobWork representation of a ROS pose is also a transform
 * then the function to convert to a ROS pose has the type appended i.e.
 * toRosPose
 * This may change in the future if a better way to streamline the
 * conversion functions is found with acceptable tradeoffs.
 ************************************************************************/
geometry_msgs::Pose toRosPose(const rw::math::Transform3D<>& transform)
{
  rw::math::Quaternion<> q(transform.R());
  geometry_msgs::Pose result;
  result.position.x = transform.P()(0);
  result.position.y = transform.P()(1);
  result.position.z = transform.P()(2);

  result.orientation.x = q(0);
  result.orientation.y = q(1);
  result.orientation.z = q(2);
  result.orientation.w = q(3);

  return result;
}

rw::math::Transform3D<> toRw(const geometry_msgs::Pose& pose)
{
  rw::math::Vector3D<> p(pose.position.x, pose.position.y, pose.position.z);
  rw::math::Quaternion<> q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  return rw::math::Transform3D<>(p, q);
}

/************************************************************************
 * Wrench
 ************************************************************************/
geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w)
{
  geometry_msgs::Wrench wrench;
  wrench.force.x = w.force()[0];
  wrench.force.y = w.force()[1];
  wrench.force.z = w.force()[2];
  wrench.torque.x = w.torque()[0];
  wrench.torque.y = w.torque()[1];
  wrench.torque.z = w.torque()[2];
  return wrench;
}

rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench)
{
  rw::math::Wrench6D<> w(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y,
                         wrench.torque.z);
  return w;
}

/************************************************************************
 * Twist / VelocityScrew
 ************************************************************************/
rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist)
{
  rw::math::VelocityScrew6D<> vs(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y,
                                 twist.angular.z);
  return vs;
}

geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs)
{
  geometry_msgs::Twist twist;
  twist.linear.x = vs(0);
  twist.linear.y = vs(1);
  twist.linear.z = vs(2);
  twist.angular.x = vs(3);
  twist.angular.y = vs(4);
  twist.angular.z = vs(5);
  return twist;
}

/************************************************************************
 * Double
 ************************************************************************/
double toRw(const double value)
{
  return value;
}

/************************************************************************
 * Float
 ************************************************************************/
float toRw(const float value)
{
  return value;
}

/************************************************************************
 * Boolean
 ************************************************************************/
bool toRw(const bool value)
{
  return value;
}

/************************************************************************
 * Convenience
 ************************************************************************/
rw::models::WorkCell::Ptr getWorkCell()
{
  return getWorkCell("/caros/workcell");
}

rw::models::WorkCell::Ptr getWorkCell(const std::string& paramname)
{
  static rw::models::WorkCell::Ptr wc;
  if (wc == NULL)
  {
    ros::NodeHandle node("~");
    std::string workcell_file;
    bool found_param;
    found_param = node.getParam(paramname, workcell_file);
    if (!found_param)
    {
      ROS_ERROR_STREAM("No such parameter on the parameter server: " << paramname);
      return NULL;
    }
    else if (workcell_file.empty())
    {
      ROS_ERROR_STREAM("The value of the parameter is empty!");
      return NULL;
    }
    ROS_DEBUG_STREAM("loading file: " << workcell_file);
    try
    {
      wc = rw::loaders::WorkCellFactory::load(workcell_file);
    }
    catch (const std::exception& exp)
    {
      ROS_ERROR_STREAM("Unable to open workcell file ");
      ROS_ERROR_STREAM("Error: " << exp.what());
      return NULL;
    }
  }
  return wc;
}

rw::common::Ptr<rw::kinematics::State> getState()
{
  // currently we always get the state from a service and not a topic
  ros::NodeHandle node("~");
  if (!ros::service::exists("/caros/get_workcell_state", true))
  {
    ROS_ERROR_STREAM("There are no registered state sources!");
    return NULL;
  }

  rw::models::WorkCell::Ptr wc = getWorkCell();

  ros::ServiceClient global = node.serviceClient<caros_common_robwork_msgs::GetRwState>("/caros/get_workcell_state");

  caros_common_robwork_msgs::GetRwState service;

  global.call(service);

  rw::kinematics::State state = caros::toRw(service.response.state, wc);

  return ownedPtr(new rw::kinematics::State(state));
}

}  // namespace caros
