#include <caros/test/gripper_service_interface_dummy.h>
#include <gtest/gtest.h>

GripperServiceInterfaceDummy::GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool return_value,
                                                           const bool cause_error)
    : caros::GripperServiceInterface(nodehandle),
      return_value_(return_value),
      cause_error_(cause_error),
      causing_error_msg_("Intentionally causing error - please ignore it")
{
  /* make ROS publish the services */
  if (!GripperServiceInterface::configureInterface())
  {
    /* Invalid object, since the services weren't published within this constructor */
    throw std::runtime_error("The service interface could not be configured!");
  }
}

GripperServiceInterfaceDummy::~GripperServiceInterfaceDummy()
{
  /* Nothing to clean up */
}

const std::string& GripperServiceInterfaceDummy::getMostRecentFunctionCalled() const
{
  return most_recent_function_called_;
}

bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q& q)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q& q)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q& q)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q& q)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool GripperServiceInterfaceDummy::stopMovement(void)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}
