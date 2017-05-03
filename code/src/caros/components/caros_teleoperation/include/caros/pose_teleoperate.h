#ifndef CAROS_POSE_TELEOPERATE_H
#define CAROS_POSE_TELEOPERATE_H

#include <caros/common.h>
#include <caros/serial_device_si_proxy.h>
#include <caros/pose_sensor_si_proxy.h>
#include <caros/button_sensor_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros/pose_sensor_service_interface.h>
#include <caros/caros_node_service_interface.h>

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models.hpp>
#include <rw/common/Ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <memory>

namespace caros
{
/**
 * @brief this ROS node teleoperates a Robot arm and a gripper, using a multiple PoseSensors as input.
 *
 * The posesensors should be organized and attached to a glove.
 */
class PoseTeleoperate : public caros::CarosNodeServiceInterface
{
 public:
  /**
   * @brief Constructor
   */
  explicit PoseTeleoperate(const ros::NodeHandle& nh, const std::string& name = "/teleoperate");

  //! destructor
  virtual ~PoseTeleoperate();
  enum TELEOPERATE_ERRORCODE
  {
    TELEOPERATE_MISSING_ROSPARAM = 1,
    TELEOPERATE_MISSING_WORKCELL,
    TELEOPERATE_MISSING_DEVICE_IN_WORKCELL,
    TELEOPERATE_SUBSCRIPTION_FAILED,
    TELEOPERATE_MISSING_ROSPARAM_RUNTIME,
    TELEOPERATE_INTERNAL_ERROR
  };

 protected:
  //! initializes the node handle
  bool initNode();

  void handleButtonSensor(caros_sensor_msgs::ButtonSensorState btn_state);
  void handlePoseArraySensor(caros_sensor_msgs::PoseSensorState array);

  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

 protected:
  ros::NodeHandle nh_;

  // these are initialized and deinitialized on start and stop
  ros::Subscriber pose_sensor_subscriber_;
  ros::Subscriber button_sensor_state_;
  ros::Subscriber pose_array_state_;

  std::string button_sensor_name_;
  std::string pose_array_name_;

  std::shared_ptr<caros::SerialDeviceSIProxy> device_sip_;
  std::shared_ptr<caros::PoseSensorSIProxy> pose_sip_;

  ros::ServiceServer srv_start_, srv_stop_, srv_pause_;

  // last recorded data
  std::string frame_ref_;
  ros::Time stamp_;
  std::vector<rw::math::Transform3D<>> poses_;
  std::vector<int> ids_;
  std::vector<float> qualities_;

  rw::math::Transform3D<> baseTsensor_base_, initial_sensor_pose_, initial_robotTtool_, current_poseTtool_, rel_move_;

  int pose_sensor_id1_;

  rw::kinematics::State tmp_state_;

 private:
  /**
   * @brief starts the teleoperation mode: starts listening for posesensor.
   */
  bool startListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
   * @brief stop listening for posesensor data. need to run start again.
   */
  bool stopListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool pauseListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool runLoop();

  // flags
  bool do_teleoperate_;  // running the code

  //! is called by run
  void doTeleoperate();
  // devices etc.
  rw::models::Device::Ptr dev_;
  rw::models::WorkCell::Ptr p_workcell_;

  // RobWork
  rw::kinematics::State tele_state_;

  // doTeleoperate stuff
  rw::math::Transform3D<double> offset_Zpos_, sensor_offset_;

  bool analog_button_, analog_button_pushed_;

  rw::math::Vector3D<double> relative_motion_pos;
  rw::math::Transform3D<double> base_sensor_orig;
  rw::math::Rotation3D<double> relative_motion_rot;
  rw::math::Transform3D<double> robot_tool_orig_transform;
  rw::math::Transform3D<double> target_transform;
  rw::math::Transform3D<double> trakstar_sensor_transform;

  rw::math::Transform3D<> last_target_pose_;
};
}  // namespace caros

#endif  // CAROS_POSE_TELEOPERATE_H
