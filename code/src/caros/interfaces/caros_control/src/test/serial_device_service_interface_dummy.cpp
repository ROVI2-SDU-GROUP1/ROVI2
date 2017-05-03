#include <caros/test/serial_device_service_interface_dummy.h>
#include <gtest/gtest.h>

SerialDeviceServiceInterfaceDummy::SerialDeviceServiceInterfaceDummy(ros::NodeHandle nodehandle,
                                                                     const bool return_value, const bool cause_error)
    : caros::SerialDeviceServiceInterface(nodehandle),
      return_value_(return_value),
      cause_error_(cause_error),
      causing_error_msg_("Intentionally causing error - please ignore it")
{
  /* make ROS publish the services */
  if (!SerialDeviceServiceInterface::configureInterface())
  {
    /* Invalid object, since the services weren't published within this constructor */
    throw std::runtime_error("The service interface could not be configured!");
  }
}

SerialDeviceServiceInterfaceDummy::~SerialDeviceServiceInterfaceDummy()
{
  /* Nothing to clean up */
}

const std::string& SerialDeviceServiceInterfaceDummy::getMostRecentFunctionCalled() const
{
  return most_recent_function_called_;
}

bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t& targets)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::movePtp(const QAndSpeedContainer_t& targets)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::movePtpT(const TransformAndSpeedContainer_t& targets)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q& q_vel)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t& targets)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t& targets)
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}

bool SerialDeviceServiceInterfaceDummy::moveStop()
{
  most_recent_function_called_ = __PRETTY_FUNCTION__;
  if (cause_error_)
  {
    throw std::runtime_error(causing_error_msg_);
  }
  return return_value_;
}
