#include <caros/button_sensor_service_interface.h>
#include <caros_sensor_msgs/ButtonSensorState.h>

#include <utility>
#include <string>
#include <vector>

using namespace caros;

ButtonSensorServiceInterface::ButtonSensorServiceInterface(const ros::NodeHandle& nodehandle)
    : nodehandle_(nodehandle, BUTTON_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Nothing specific to do */
}

ButtonSensorServiceInterface::~ButtonSensorServiceInterface()
{
}

bool ButtonSensorServiceInterface::configureInterface()
{
  if (button_publisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more ButtonSensorServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  button_publisher_ = nodehandle_.advertise<caros_sensor_msgs::ButtonSensorState>(
      "buttons", BUTTON_SENSOR_BUTTONS_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!button_publisher_, "The ButtonSensor buttons publisher is empty!");

  /* Verify that the various ROS services have actually been created properly */
  if (button_publisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM(
        "All ButtonSensorServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The ButtonSensor service could not be properly initialised - one or more ros services or publishers failed to "
        "be properly initialised.");
    return false;
  }

  return true;
}

void ButtonSensorServiceInterface::publishButtons(const std::vector<std::pair<std::string, bool>>& digital_buttons,
                                                  const std::vector<std::pair<std::string, bool>>& analog_buttons)
{
  caros_sensor_msgs::ButtonSensorState button_state;
  button_state.digital_ids.resize(digital_buttons.size());
  button_state.digital.resize(digital_buttons.size());
  button_state.analog_ids.resize(analog_buttons.size());
  button_state.analog.resize(analog_buttons.size());

  for (size_t i = 0; i < digital_buttons.size(); i++)
  {
    button_state.digital_ids[i] = digital_buttons[i].first;
    button_state.digital[i] = digital_buttons[i].second;
  }

  for (size_t i = 0; i < analog_buttons.size(); i++)
  {
    button_state.analog_ids[i] = analog_buttons[i].first;
    button_state.analog[i] = analog_buttons[i].second;
  }

  button_publisher_.publish(button_state);
}
