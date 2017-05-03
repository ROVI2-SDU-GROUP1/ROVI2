#include <caros/pose_teleoperate.h>

#include <rw/math/LinearAlgebra.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>

using rw::math::Deg2Rad;
using rw::math::RPY;
using rw::math::Rotation3D;
using rw::math::Transform3D;
using rw::math::Vector3D;
using rw::math::Q;
using rw::math::Quaternion;

namespace caros
{
PoseTeleoperate::PoseTeleoperate(const ros::NodeHandle& nh, const std::string& name)
    : caros::CarosNodeServiceInterface(nh, 100), nh_(nh), do_teleoperate_(false)
{
  pose_sensor_id1_ = -1;
  analog_button_pushed_ = false;
  analog_button_ = false;
}

bool PoseTeleoperate::activateHook()
{
  std_srvs::Empty::Request request;
  std_srvs::Empty::Response response;

  if (!initNode())
  {
    return false;
  }

  if (!startListening(request, response))
  {
    return false;
  }

  return true;
}
bool PoseTeleoperate::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  std_srvs::Empty::Request request;
  std_srvs::Empty::Response response;

  bool resolved = false;

  switch (error_code)
  {
    case TELEOPERATE_MISSING_ROSPARAM_RUNTIME:
      if (startListening(request, response))
      {
        ROS_DEBUG_STREAM("Subscribing to ButtonSensor topic");
        button_sensor_state_ = nh_.subscribe(button_sensor_name_, 1, &PoseTeleoperate::handleButtonSensor, this);
        if (!button_sensor_state_)
        {
          CAROS_FATALERROR("Subscribing to ButtonSensor topic failed from recoverhook - FATALERROR",
                           TELEOPERATE_SUBSCRIPTION_FAILED);
        }
        resolved = true;
      }
      if (!pose_array_state_)
      {
        CAROS_FATALERROR(
            "Not able to properly recover from the error condition 'missing rosparam at runtime' - going into "
            "FATALERROR",
            TELEOPERATE_MISSING_ROSPARAM_RUNTIME);
        resolved = false;
        ROS_DEBUG_STREAM("Subscribing to pose topic");
        pose_array_state_ = nh_.subscribe(pose_array_name_, 1, &PoseTeleoperate::handlePoseArraySensor, this);
        if (!button_sensor_state_)
        {
          CAROS_FATALERROR("Subscribing to pose topic failed from recoverhook - FATALERROR",
                           TELEOPERATE_SUBSCRIPTION_FAILED);
        }
        resolved = true;
      }
      else
      {
        CAROS_FATALERROR(
            "Not able to properly recover from the error condition 'failed subscription' - going into FATALERROR",
            TELEOPERATE_SUBSCRIPTION_FAILED);
        resolved = false;
      }
      break;
    default:
      CAROS_FATALERROR("The provided error code '" << error_code << "' has no recovery functionality! "
                                                   << "- this should be considered a bug!",
                       TELEOPERATE_INTERNAL_ERROR);
      resolved = false;
      break;
  }

  if (resolved)
  {
    do_teleoperate_ = true;
  }

  return resolved;
}
void PoseTeleoperate::errorLoopHook()
{
  ROS_ERROR_STREAM("Something is wrong. Try calling recoverHook()");
  do_teleoperate_ = false;
}

void PoseTeleoperate::fatalErrorLoopHook()
{
  ROS_ERROR_STREAM("Fatal error. Shutting down node...");
  std_srvs::Empty::Request request;
  std_srvs::Empty::Response response;
  stopListening(request, response);
}

void PoseTeleoperate::runLoopHook()
{
  if (do_teleoperate_)
  {
    doTeleoperate();
  }
}

bool PoseTeleoperate::initNode()
{
  // the stuff we need to listen for
  std::string dev_name, button_name, pose_sensor_name;
  double rate, zoffset, zoffset_tcp;

  if (!nh_.getParam("device_name", dev_name))
  {
    CAROS_FATALERROR("The parameter '" << nh_.getNamespace()
                                       << "/device_name' was not present on the parameter server! "
                                       << "This parameter has to be specified for this node to work properly.",
                     TELEOPERATE_MISSING_ROSPARAM);
    return false;
  }

  nh_.param("rate", rate, 100.0);
  nh_.param("zoffset_tcp", zoffset_tcp, 0.0);
  nh_.param("zoffset", zoffset, 0.0);

  offset_Zpos_.P()[2] = zoffset_tcp;
  sensor_offset_.P()[2] = zoffset;

  setLoopRateFrequency(rate);

  // add some control interface
  srv_start_ = nh_.advertiseService("start", &PoseTeleoperate::startListening, this);
  srv_stop_ = nh_.advertiseService("stop", &PoseTeleoperate::stopListening, this);
  srv_pause_ = nh_.advertiseService("pause", &PoseTeleoperate::pauseListening, this);

  // get the workcell
  p_workcell_ = caros::getWorkCell();
  if (p_workcell_ == NULL)
  {
    ROS_ERROR("No workcell added to the parameter server!");
    CAROS_FATALERROR("No workcell added to the parameter server!", TELEOPERATE_MISSING_ROSPARAM);
    return false;
  }

  // device
  dev_ = p_workcell_->findDevice(dev_name);
  tmp_state_ = p_workcell_->getDefaultState();
  if (dev_ == NULL)
  {
    ROS_ERROR_STREAM("No device by name " << dev_name << " Possible devices are:");
    CAROS_FATALERROR("No device by name found in workcell.", TELEOPERATE_MISSING_DEVICE_IN_WORKCELL);
    for (rw::models::Device::Ptr dev : p_workcell_->getDevices())
    {
      ROS_ERROR_STREAM("dev: " << dev->getName());
    }
    return false;
  }
  return true;
}

PoseTeleoperate::~PoseTeleoperate()
{
  /* Nothing specific to do */
}

bool PoseTeleoperate::startListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_DEBUG_STREAM("start: ");
  if (do_teleoperate_)
  {
    ROS_DEBUG_STREAM("Stopping the running teleoperation");
    stopListening(request, response);
  }

  // get pose sensor name

  if (!nh_.getParam("PoseArray", pose_array_name_))
  {
    CAROS_ERROR("No pose sensor topic name defined in parameter server!", TELEOPERATE_MISSING_ROSPARAM_RUNTIME);
    return false;
  }

  if (!nh_.getParam("PushSensor", button_sensor_name_))
  {
    CAROS_ERROR("No button sensor topic name defined in parameter server!", TELEOPERATE_MISSING_ROSPARAM_RUNTIME);
    return false;
  }

  nh_.param("PoseIdx", pose_sensor_id1_, 0);

  // initialize robot arm proxy
  ROS_INFO_STREAM("Subscribing to Device proxy, with:" << dev_->getName());
  device_sip_ = std::make_shared<caros::SerialDeviceSIProxy>(nh_, dev_->getName());
  ROS_INFO_STREAM("Subscribing to Pose Sensor proxy, with: " << pose_array_name_);
  pose_sip_ = std::make_shared<caros::PoseSensorSIProxy>(nh_, pose_array_name_);

  ROS_INFO_STREAM("Subscribing to ButtonSensor topic, with: " << button_sensor_name_);
  button_sensor_state_ = nh_.subscribe(button_sensor_name_, 1, &PoseTeleoperate::handleButtonSensor, this);
  if (!button_sensor_state_)
  {
    CAROS_ERROR("Subscribing to ButtonSensor topic, with: " << button_sensor_name_ << " failed!",
                TELEOPERATE_SUBSCRIPTION_FAILED);
    return false;
  }

  pose_array_state_ = nh_.subscribe(pose_array_name_, 1, &PoseTeleoperate::handlePoseArraySensor, this);
  ROS_INFO_STREAM("Subscribing to pose topic, with: " << pose_array_name_);
  if (!pose_array_state_)
  {
    CAROS_ERROR("Subscribing to pose topic, with: " << pose_array_name_ << " failed!", TELEOPERATE_SUBSCRIPTION_FAILED);
    return false;
  }

  // initialize doTeleoperate stuff
  do_teleoperate_ = true;

  return true;
}

void PoseTeleoperate::handleButtonSensor(caros_sensor_msgs::ButtonSensorState btn_state)
{
  if (btn_state.analog[0] > 0)
    analog_button_pushed_ = true;
  else
    analog_button_pushed_ = false;
}

void PoseTeleoperate::handlePoseArraySensor(caros_sensor_msgs::PoseSensorState array)
{
  poses_.clear();
  for (const geometry_msgs::Transform pose : array.poses)
  {
    Quaternion<> quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
    Vector3D<> pos(pose.translation.x, pose.translation.y, pose.translation.z);
    // Quaternion<> quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    // Vector3D<> pos(pose.position.x, pose.position.y, pose.position.z);
    poses_.push_back(Transform3D<>(pos, quat.toRotation3D()));
  }
}

bool PoseTeleoperate::pauseListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  /* TODO(any): */
  ROS_WARN_STREAM("This method is not implemented!");
  return true;
}

bool PoseTeleoperate::stopListening(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (!do_teleoperate_)
  {
    ROS_DEBUG_STREAM("Already stopped!");
    return true;
  }
  do_teleoperate_ = false;
  device_sip_ = NULL;
  pose_sip_ = NULL;
  return true;
}

void PoseTeleoperate::doTeleoperate()
{
  // WARNING SHOULD NOT BLOCK, this should be called from another loop

  if (static_cast<signed int>(poses_.size()) <= pose_sensor_id1_)
  {
    ROS_WARN("no poses yet!");
    return;
  }

  Transform3D<> robotbaseTtrans(RPY<>(180 * Deg2Rad, 0, 0).toRotation3D());

  Transform3D<> pose1 = robotbaseTtrans * poses_[pose_sensor_id1_] * sensor_offset_;
  Q robQ = device_sip_->getQ();
  Q robQd = device_sip_->getQd();
  // std::cout << robQ << std::endl;
  // servoing of the robot device
  if (!analog_button_pushed_)
  {
    if (analog_button_)
    {
      ROS_INFO("Released BTN");
    }
    analog_button_ = false;
  }
  else if (robQ.size() == 0)
  {
    ROS_WARN("robQ.size() is 0.... Probably due to read error in Robot State");
  }
  else
  {
    dev_->setQ(robQ, tmp_state_);
    // if button was not pushed down before then save the tool/sensor transform
    if (!analog_button_)
    {
      ROS_INFO("Pushed BTN");
      analog_button_ = true;
      initial_sensor_pose_ = pose1;  // pose1 * offsetZpos
      initial_robotTtool_ = dev_->baseTend(tmp_state_) * offset_Zpos_;
      last_target_pose_ = pose1;
    }

    // calculate the change from initial pose to current pose
    // Transform3D<> initialSensorTcurrent = inverse(initialSensorPose_) * pose1;

    Vector3D<> relativeMotionPos = pose1.P() - initial_sensor_pose_.P();
    Rotation3D<> relativeMotionRot = pose1.R() * inverse(initial_sensor_pose_.R());

    Transform3D<> baseTtool_target =
        Transform3D<>(initial_robotTtool_.P() + relativeMotionPos, relativeMotionRot * initial_robotTtool_.R());

    if (!device_sip_->moveServoT(baseTtool_target * inverse(offset_Zpos_)))
    {
      ROS_WARN("deviceSIP_->moveServoT(baseTtool_target * inverse(offsetZpos))) failed. Move command to robot failed.");
    }

    last_target_pose_ = pose1;
  }
}
}  // namespace caros
