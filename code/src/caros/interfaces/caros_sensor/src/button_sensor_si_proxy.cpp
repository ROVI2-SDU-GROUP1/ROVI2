#include <caros/button_sensor_si_proxy.h>

#include <string>
#include <vector>

using namespace std;
using namespace caros;

ButtonSensorSIProxy::ButtonSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                         const bool use_persistent_connections)
    : nodehandle_(nodehandle)
{
  button_sensor_state_sub_ = nodehandle_.subscribe(nodehandle_.getNamespace() + "/buttons", 1,
                                                   &ButtonSensorSIProxy::handleButtonSensorState, this);
}

ButtonSensorSIProxy::~ButtonSensorSIProxy()
{
}

void ButtonSensorSIProxy::handleButtonSensorState(const caros_sensor_msgs::ButtonSensorState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stamp_ = state.header.stamp;
  buttons_.resize(state.digital.size() + state.analog.size());

  for (size_t i = 0; i < state.digital.size(); i++)
  {
    ButtonData& data = buttons_[i];
    data.button = state.digital[i];
    data.id = state.digital_ids[i];
    data.is_analog = false;
    data.stamp = stamp_;
  }
  for (size_t j = 0; j < state.analog.size(); j++)
  {
    ButtonData& data = buttons_[state.digital.size() + j];
    data.button = state.analog[j];
    data.id = state.analog_ids[j];
    data.is_analog = true;
    data.stamp = stamp_;
  }
}

std::vector<ButtonSensorSIProxy::ButtonData> ButtonSensorSIProxy::getButtons()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buttons_;
}

ros::Time ButtonSensorSIProxy::getTimeStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stamp_;
}
