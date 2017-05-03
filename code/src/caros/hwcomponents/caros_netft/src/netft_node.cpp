#include <caros/netft_node.h>
#include <rw/rw.hpp>
#include <string>

using namespace caros;
using namespace rw;
using namespace rwhw;

NetFTNode::NetFTNode(const ros::NodeHandle& node_handle)
    : caros::CarosNodeServiceInterface(node_handle),
      caros::FTSensorServiceInterface(node_handle),
      node_handle_(node_handle)
{
  /* Nothing specific to do */
}

NetFTNode::~NetFTNode()
{
  if (netft_ != 0)
  {
    netft_->stop();
  }
}

bool NetFTNode::activateHook()
{
  /* Fetch parameters (if any) or use the defaults */

  std::string param;
  param = "ip";
  lookupParamWithDefaultDebugMsg(param);
  node_handle_.param(param, ip_, std::string("192.168.100.2"));

  param = "port";
  lookupParamWithDefaultDebugMsg(param);
  node_handle_.param(param, port_, 49152);

  param = "rate";
  lookupParamWithDefaultDebugMsg(param);
  node_handle_.param(param, publish_rate_, 50);

  setLoopRateFrequency(publish_rate_);
  netft_ = rw::common::ownedPtr(new NetFTLogging(ip_, port_));

  try
  {
    netft_->start();
  }
  catch (const std::exception& exp)
  {
    CAROS_FATALERROR("Unable to start communication with the NetFT sensor (exception msg: '" << exp.what() << "')",
                     NETFT_UNABLE_TO_START_COMMUNICATION);
    return false;
  }

  if (!FTSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS FTSensorServiceInterface could not be configured correctly.",
                     NETFT_CAROS_FT_SENSOR_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool NetFTNode::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* TODO: */

  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented yet!");
  ROS_BREAK();

  return false;
}

void NetFTNode::runLoopHook()
{
  try
  {
    if (netft_ == 0)
    {
      CAROS_FATALERROR("The NetFT device is not configured", NETFT_INTERNAL_ERROR);
      return;
    }

    NetFTLogging::NetFTData data = netft_->getAllData();
    /* Should the refframe "FT" be configurable? */
    publish(rw::math::Wrench6D<>(data.data.first, data.data.second), "FT");
  }
  catch (const rw::common::Exception& exp)
  {
    /* Could look at the kind of error/exception and handle them differently than just going into the error state (see
     * the Boost documentation for the exceptions/error conditions of unique_lock (which is used as of this writing)) */

    CAROS_ERROR(exp.what(), NETFT_INTERNAL_ERROR);
    return;
  }
}

void NetFTNode::errorLoopHook()
{
  /* Stop the NetFTNode's current action(s) */
  if (netft_ == 0)
  {
    ROS_DEBUG_STREAM("The NetFT device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    netft_->stop();
  }
}

void NetFTNode::fatalErrorLoopHook()
{
  /* Stop the NetFTNode's current action(s) */
  if (netft_ == 0)
  {
    ROS_DEBUG_STREAM("The NetFT device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    netft_->stop();
  }
}

void NetFTNode::lookupParamWithDefaultDebugMsg(const std::string& param)
{
  ROS_DEBUG_STREAM_COND(!node_handle_.hasParam(param), "Parameter " << param
                                                                    << " not found from param server. Using default.");
}
