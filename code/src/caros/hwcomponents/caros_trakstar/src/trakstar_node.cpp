#include <caros/trakstar_node.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

#include <rw/math.hpp>
#include <string>
#include <vector>

#include <caros/trakstar.h>
// using namespace rw::math;
// using namespace caros;
// using namespace marvin_common;
// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_EXPORT_CLASS(caros::TrakstarNode, nodelet::Nodelet)

caros::TrakstarNode::TrakstarNode()
    : Nodelet(),
      CarosNodeServiceInterface(ros::NodeHandle("~"), 100),
      ButtonSensorServiceInterface(ros::NodeHandle("~")),
      PoseSensorServiceInterface(ros::NodeHandle("~")),
      nodehandle_(ros::NodeHandle("~"))

{
  ROS_DEBUG_STREAM("create a new Trakstar driver");
  max_pub_frequency_ = 100;
  t_driver_ = std::unique_ptr<caros::Trakstar>(new caros::Trakstar);
  // t_driver_->initialize(true);
  ROS_DEBUG_STREAM("exit constructor");
}

caros::TrakstarNode::TrakstarNode(const ros::NodeHandle& nodehandle)
    : CarosNodeServiceInterface(nodehandle, 100),
      ButtonSensorServiceInterface(nodehandle),
      PoseSensorServiceInterface(nodehandle),
      nodehandle_(nodehandle)
{
  // create a new Trakstar driver
  ROS_DEBUG_STREAM("create a new Trakstar driver");
  max_pub_frequency_ = 100;
  t_driver_ = std::unique_ptr<caros::Trakstar>(new caros::Trakstar);
  // t_driver_->initialize(true);
}

caros::TrakstarNode::~TrakstarNode()
{
  if (t_driver_)
  {
    if (t_driver_->isPolling())
    {
      ROS_DEBUG_STREAM("Still polling data - going to stop the device and disconnect.");
      t_driver_->stopPolling();
    }
    t_driver_ = 0;
  }
  else
  {
    ROS_DEBUG_STREAM("There was no trakstar device to destroy.");
  }
}
void caros::TrakstarNode::onInit()
{
  pub_thread_.reset(new boost::thread(boost::bind(&caros::TrakstarNode::start, this)));
  ROS_DEBUG_STREAM("Entering PREINIT");
  while (getState() == PREINIT)
  {
    ros::spinOnce();
  };
  ROS_DEBUG_STREAM("Leaving PREINIT");
  // now wait until node is finished initializing
}
bool caros::TrakstarNode::activateHook()
{
  /************************************************************************
   * Parameters
   ************************************************************************/
  // if (! nodehandle_.getParam("device_name", deviceName)) {
  //    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace()
  // << "/device_name' was not present on the parameter server!
  // This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
  //    return false;
  // }
  nodehandle_.param("rate", max_pub_frequency_, 100.0);
  nodehandle_.param("frame", frame_id_, std::string("TrakstarBase"));
  nodehandle_.param("calibration_data", calibration_data_, std::string(""));

  analog_buttons = {{"analog_btn_0", 0}, {"analog_btn_1", 0}, {"analog_btn_2", 0}};
  digital_buttons = {{"digital_btn_0", 0}, {"digital_btn_1", 0}, {"digital_btn_2", 0}};

  // if calibration enabled then also publish raw data
  // _posearray_publisher = _node_handle.advertise<geometry_msgs::PoseArray>("poses_raw", 5);

  ROS_DEBUG_STREAM("Initialising TrakStar sensor system, this might take some time...");
  t_driver_->initialize(true);

  ROS_DEBUG_STREAM("Sensors Initialised");
  while (t_driver_->getInitStatus() == Trakstar::TRAKSTAR_STATUS_INITIALIZING)
  { /* WAIT FOR INITIALIZATION TO FINISH */
    ROS_INFO_STREAM("Initialising.... ");
  }

  if (t_driver_->getInitStatus() == Trakstar::TRAKSTAR_STATUS_STARTED)
  {
    ROS_DEBUG_STREAM("Sensors Initialised: Start polling ..");
    if (!t_driver_->startPolling())
    {
      ROS_DEBUG_STREAM("failed!");
    }
    else
      ROS_DEBUG_STREAM("succeed. Is polling " << t_driver_->isPolling());
  }
  else
  {
    CAROS_FATALERROR("Unable to initialise trakStar", TRAKSTAR_DRIVER_INITIALIZATION_FAIL);
    return false;
  }

  if (!PoseSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The pose sensor service could not be configured correctly.", TRAKSTAR_POSESENSOR_CONFIGURE_FAIL);
    return false;
  }

  if (!ButtonSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The buttonservice could not be configured correctly.", TRAKSTAR_BUTTONSERVICE_CONFIGURE_FAIL);
    return false;
  }

  setLoopRateFrequency(max_pub_frequency_);

  return true;
}

void caros::TrakstarNode::runLoopHook()
{
  // tjek status of driver, also make sure it is initialized
  // if(t_driver_->inError() ) { }
  if (!t_driver_->isInitialized())
  {
    ROS_ERROR_STREAM("Driver is not initialized! restart node...");
  }

  const std::vector<Trakstar::PoseData> pd = t_driver_->getData();
  geometry_msgs::PoseArray array;

  std::vector<rw::math::Transform3D<>> transforms;
  std::vector<int> ids;
  std::vector<float> qualities;

  for (unsigned int i = 0; i < pd.size(); i++)
  {
    ids.push_back(i);
    transforms.push_back(rw::math::Transform3D<>(pd[i].pos * 0.001, pd[i].rot.toRotation3D()));
    qualities.push_back(pd[i].quality);

    if (pd[i].analog_button_on)
    {
      analog_buttons[0].second = 1;
    }
    else
    {
      analog_buttons[0].second = 0;
    }
  }
  PoseSensorServiceInterface::publishPoses(transforms, ids, qualities);
  ButtonSensorServiceInterface::publishButtons(digital_buttons, analog_buttons);
}
void caros::TrakstarNode::errorLoopHook()
{
  ROS_DEBUG_STREAM("Entering errorLoopHook");
  if (!t_driver_->isInitialized())
  {
    ROS_ERROR_STREAM("Driver is not initialized! restart node...");
  }
  else
  {
    ROS_ERROR_STREAM("Stop polling...");
    t_driver_->stopPolling();
    ROS_ERROR_STREAM("Polling stopped.");
  }
}
void caros::TrakstarNode::fatalErrorLoopHook()
{
  ROS_DEBUG_STREAM("Entering fatalErrorLoopHook");
  if (t_driver_)
  {
    ROS_ERROR_STREAM("Stop polling...");
    t_driver_->stopPolling();
    t_driver_ = NULL;
    ROS_ERROR_STREAM("Polling stopped.");
  }
  else
  {
    ROS_DEBUG_STREAM("The TrakStar device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
}
