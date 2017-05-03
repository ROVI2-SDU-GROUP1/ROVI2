#include <caros/ft_sensor_si_proxy.h>
#include <caros/common_robwork.h>

#include <string>

using namespace caros;

FTSensorSIProxy::FTSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                 const bool use_persistent_connections)
    : nodehandle_(nodehandle)
{
  ft_state_sub_ = nodehandle_.subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::~FTSensorSIProxy()
{
}

void FTSensorSIProxy::handleFTState(const geometry_msgs::WrenchStamped& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  wrench_ = caros::toRw(state.wrench);
  ft_state_ = state;
}

rw::math::Wrench6D<> FTSensorSIProxy::getWrench()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return wrench_;
}

ros::Time FTSensorSIProxy::getTimeStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ft_state_.header.stamp;
}
