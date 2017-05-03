#include <caros/pose_sensor_si_proxy.h>
#include <caros/common_robwork.h>

#include <string>
#include <vector>

using namespace rw::math;
using namespace std;
using namespace caros;

PoseSensorSIProxy::PoseSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                     const bool use_persistent_connections)
    : nodehandle_(nodehandle)
{
  pose_sensor_state_sub_ =
      nodehandle_.subscribe(nodehandle_.getNamespace() + "/poses", 1, &PoseSensorSIProxy::handlePoseSensorState, this);
}

PoseSensorSIProxy::~PoseSensorSIProxy()
{
}

void PoseSensorSIProxy::handlePoseSensorState(const caros_sensor_msgs::PoseSensorState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  poses_.resize(state.poses.size());
  stamp_ = state.header.stamp;
  for (size_t i = 0; i < state.poses.size(); i++)
  {
    PoseData& pdata = poses_[i];
    pdata.pose = caros::toRw(state.poses[i]);
    pdata.id = state.ids[i];
    pdata.quality = state.qualities[i];
    pdata.stamp = state.header.stamp;
    pdata.frame = state.header.frame_id;
  }
}

std::vector<PoseSensorSIProxy::PoseData> PoseSensorSIProxy::getPoses()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_;
}

ros::Time PoseSensorSIProxy::getTimeStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stamp_;
}
