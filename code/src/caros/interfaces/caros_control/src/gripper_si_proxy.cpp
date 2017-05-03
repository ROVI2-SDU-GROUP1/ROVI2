#include <caros/gripper_si_proxy.h>

/* Provides GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE and the different msg and srv types*/
#include <caros/gripper_service_interface.h>

#include <caros/common.h>
#include <caros/common_robwork.h>
#include <caros/exceptions.h>

#include <caros_control_msgs/GripperState.h>

/* TODO:
 * Find a good way to handle the situation where a user tries to get some data e.g. getQ and no data has ever been
 *received yet - currently it will just return the default initialised values.
 *
 * Provide synchronous versions of the service calls such as moveQ and gripQ. This would make a blocking call waiting
 *for the reported gripper state to show the proper values, but handleGripperState would never be run with the current
 *setup (where the user has to invoke "ros spin") - requiring the subscription callback to be handled in its own thread.
 */

using namespace caros;

GripperSIProxy::GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                               const bool use_persistent_connections)
    : nodehandle_(nodehandle),
      use_persistent_connections_(use_persistent_connections),
      ros_namespace_("/" + devname + "/" + GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE),
      srv_move_q_(nodehandle_, "move_q", ros_namespace_, use_persistent_connections_),
      srv_grip_q_(nodehandle_, "grip_q", ros_namespace_, use_persistent_connections_),
      srv_set_force_q_(nodehandle_, "set_force_q", ros_namespace_, use_persistent_connections_),
      srv_set_velocity_q_(nodehandle_, "set_velocity_q", ros_namespace_, use_persistent_connections_),
      srv_stop_movement_(nodehandle_, "stop_movement", ros_namespace_, use_persistent_connections_)
{
  /* TODO:
   * Make the queue size into a parameter that can be configured - (hardcoded to 1 here)
   */
  sub_gripper_state_ =
      nodehandle_.subscribe(ros_namespace_ + "/gripper_state", 1, &GripperSIProxy::handleGripperState, this);
}

GripperSIProxy::~GripperSIProxy()
{
  /* Empty */
}

bool GripperSIProxy::moveQ(const rw::math::Q& q)
{
  caros_control_msgs::GripperMoveQ srv;
  srv.request.q = caros::toRos(q);

  srv_move_q_.call<caros_control_msgs::GripperMoveQ>(srv);

  return srv.response.success;
}

bool GripperSIProxy::gripQ(const rw::math::Q& q)
{
  caros_control_msgs::GripperGripQ srv;
  srv.request.q = caros::toRos(q);

  srv_grip_q_.call<caros_control_msgs::GripperGripQ>(srv);

  return srv.response.success;
}

bool GripperSIProxy::setForceQ(const rw::math::Q& q)
{
  caros_control_msgs::GripperSetForceQ srv;
  srv.request.force = caros::toRos(q);

  srv_set_force_q_.call<caros_control_msgs::GripperSetForceQ>(srv);

  return srv.response.success;
}

bool GripperSIProxy::setVelocityQ(const rw::math::Q& q)
{
  caros_control_msgs::GripperSetVelocityQ srv;
  srv.request.velocity = caros::toRos(q);

  srv_set_velocity_q_.call<caros_control_msgs::GripperSetVelocityQ>(srv);

  return srv.response.success;
}

bool GripperSIProxy::stopMovement()
{
  caros_control_msgs::GripperStopMovement srv;

  srv_stop_movement_.call<caros_control_msgs::GripperStopMovement>(srv);

  return srv.response.success;
}

/* Hardcoded since the connections are not added to a collection that can easily be iterated */
void GripperSIProxy::closePersistentConnections()
{
  srv_move_q_.shutdown();
  srv_grip_q_.shutdown();
  srv_set_force_q_.shutdown();
  srv_set_velocity_q_.shutdown();
  srv_stop_movement_.shutdown();
}

rw::math::Q GripperSIProxy::getQ()
{
  return caros::toRw(psv_gripper_state_.q);
}

rw::math::Q GripperSIProxy::getQd()
{
  return caros::toRw(psv_gripper_state_.dq);
}

rw::math::Q GripperSIProxy::getForce()
{
  return caros::toRw(psv_gripper_state_.force);
}

ros::Time GripperSIProxy::getTimeStamp()
{
  return psv_gripper_state_.header.stamp;
}

void GripperSIProxy::handleGripperState(const caros_control_msgs::GripperState& state)
{
  psv_gripper_state_ = state;
}
