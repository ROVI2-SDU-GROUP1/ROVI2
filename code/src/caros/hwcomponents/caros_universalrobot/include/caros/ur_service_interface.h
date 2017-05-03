#ifndef CAROS_UR_SERVICE_INTERFACE_H
#define CAROS_UR_SERVICE_INTERFACE_H

#include <caros_universalrobot/UrServiceServoQ.h>
#include <caros_universalrobot/UrServiceServoT.h>
#include <caros_universalrobot/UrServiceEmpty.h>
#include <caros_universalrobot/UrServiceForceModeUpdate.h>
#include <caros_universalrobot/UrServiceForceModeStart.h>
#include <caros_universalrobot/UrServiceForceModeStop.h>
#include <caros_universalrobot/UrServicePayload.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#define URSERVICEINTERFACE_SUB_NAMESPACE "ur_service_interface"

class URServiceInterface
{
 public:
  /**
   * @brief constructor
   * @param[in] nodehandle the nodehandle to use for services.
   */
  explicit URServiceInterface(const ros::NodeHandle& nodehandle);

  /**
   * @brief virtual destructor
   */
  virtual ~URServiceInterface();

  /**
   * @brief setup the ROS services for this interface
   */
  bool configureInterface();

  /**
   * @brief move robot using a pose as target (requires inverse kinematics)
   */
  virtual bool urServoT(const rw::math::Transform3D<>& target) = 0;

  /**
   * @brief move robot using an Q-configuration as target
   */
  virtual bool urServoQ(const rw::math::Q& target) = 0;

  /**
   * @brief activate forcemode
   *
   * @general 6D vector = [x, y, z, rx, ry, rz], C = compliant, NC = Non-Compliant
   * @param[in] ref_t_offset Pose vector that defines the force frame relative to the base frame.
   * @param[in] selection 6D vector, setting C in the axis direction (=1) or NC (=0)
   * @param[in] wrench_target C: Adjusts position to achieve the force/torque,
   *                         NC: accounts for an external force/torque of the specified value
   * @param[in] limits 6D vector, C: max tcp speed, NV: max deviation from tcp pose
   */
  virtual bool urForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                                const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& limits) = 0;
  virtual bool urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target) = 0;
  virtual bool urForceModeStop() = 0;

  /**
   * @brief Sets the payload of the robot.
   *
   * @param[in] mass The mass of the payload in kg.
   * @param[in] com The dispalacement of the Center of Mass from the robot TCP.
   */
  virtual bool urSetPayload(const double& mass, const rw::math::Vector3D<>& com) = 0;

 private:
  bool urServoTHandle(caros_universalrobot::UrServiceServoT::Request& request,
                      caros_universalrobot::UrServiceServoT::Response& response);

  bool urServoQHandle(caros_universalrobot::UrServiceServoQ::Request& request,
                      caros_universalrobot::UrServiceServoQ::Response& response);

  bool urForceModeStartHandle(caros_universalrobot::UrServiceForceModeStart::Request& request,
                              caros_universalrobot::UrServiceForceModeStart::Response& response);

  bool urForceModeUpdateHandle(caros_universalrobot::UrServiceForceModeUpdate::Request& request,
                               caros_universalrobot::UrServiceForceModeUpdate::Response& response);

  bool urForceModeStopHandle(caros_universalrobot::UrServiceForceModeStop::Request& request,
                             caros_universalrobot::UrServiceForceModeStop::Response& response);

  bool urSetPayloadHandle(caros_universalrobot::UrServicePayload::Request& request,
                          caros_universalrobot::UrServicePayload::Response& response);

 protected:
  ros::NodeHandle nodehandle_;

  ros::ServiceServer srv_ur_servo_t_;
  ros::ServiceServer srv_ur_servo_q_;
  ros::ServiceServer srv_ur_force_mode_start_;
  ros::ServiceServer srv_ur_force_mode_update_;
  ros::ServiceServer srv_ur_force_mode_stop_;
  ros::ServiceServer srv_ur_set_payload_;
};

#endif  // CAROS_UR_SERVICE_INTERFACE_H
