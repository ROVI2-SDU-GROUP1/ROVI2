#ifndef CAROS_TEST_GRIPPER_SERVICE_INTERFACE_DUMMY_H
#define CAROS_TEST_GRIPPER_SERVICE_INTERFACE_DUMMY_H

#include <caros/gripper_service_interface.h>

#include <string>
#include <stdexcept>

class GripperServiceInterfaceDummy : public caros::GripperServiceInterface
{
 public:
  GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool return_value, const bool cause_error = false);
  virtual ~GripperServiceInterfaceDummy();

  const std::string& getMostRecentFunctionCalled() const;

  bool moveQ(const rw::math::Q& q);
  bool gripQ(const rw::math::Q& q);
  bool setForceQ(const rw::math::Q& q);
  bool setVelocityQ(const rw::math::Q& q);
  bool stopMovement(void);

 private:
  bool return_value_;
  bool cause_error_;
  std::string causing_error_msg_;
  std::string most_recent_function_called_;
};

#endif  // CAROS_TEST_GRIPPER_SERVICE_INTERFACE_DUMMY_H
