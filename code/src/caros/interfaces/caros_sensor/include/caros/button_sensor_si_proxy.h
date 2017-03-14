#ifndef CAROS_BUTTON_SENSOR_SI_PROXY_H
#define CAROS_BUTTON_SENSOR_SI_PROXY_H

#include <caros_sensor_msgs/ButtonSensorState.h>

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace caros
{
/**
 * @brief This class implements a C++ proxy to control and read data from
 * a ButtonSensorServiceInterface.
 *
 */
class ButtonSensorSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<ButtonSensorSIProxy> Ptr;

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the node
   * @param[in] usePersistentConnections Define usage of persistent connections
   */
  ButtonSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                      const bool use_persistent_connections = true);

  //! destructor
  virtual ~ButtonSensorSIProxy();

  //! button data structure
  struct ButtonData
  {
    float button;
    std::string id;
    bool is_analog;
    ros::Time stamp;
  };

  //! get current button state
  std::vector<ButtonData> getButtons();

  //! get time stamp of current button state
  ros::Time getTimeStamp();

 protected:
  void handleButtonSensorState(const caros_sensor_msgs::ButtonSensorState& state);

  ros::NodeHandle nodehandle_;

  // states
  ros::Subscriber button_sensor_state_sub_;

  std::mutex mutex_;

  // state variables
  /* Notes:
   * std::vector could potentially, depending on the number of elements, be an ineffective container for looking up
   * specific button IDs, since the list has to be traversed until the ID is found.
   */
  std::vector<ButtonData> buttons_;
  ros::Time stamp_;
};
}  // namespace caros

#endif  // CAROS_BUTTON_SENSOR_SI_PROXY_H
