#ifndef CAROS_FT_SENSOR_SI_PROXY_H
#define CAROS_FT_SENSOR_SI_PROXY_H

#include <geometry_msgs/WrenchStamped.h>

#include <rw/math.hpp>

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>

namespace caros
{
/**
 * @brief This class implements a C++ proxy to control and read data from
 * a FTSensorServiceInterface.
 *
 */
class FTSensorSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<FTSensorSIProxy> Ptr;

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the node
   * @param[in] use_persistent_connections Define usage of persistent connections
   */
  FTSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname, const bool use_persistent_connections = true);

  //! destructor
  virtual ~FTSensorSIProxy();

  //! get current state
  rw::math::Wrench6D<> getWrench();

  //! get time stamp of current reading
  ros::Time getTimeStamp();

 protected:
  void handleFTState(const geometry_msgs::WrenchStamped& state);

  ros::NodeHandle nodehandle_;
  // states
  ros::Subscriber ft_state_sub_;

  std::mutex mutex_;
  // state variables
  rw::math::Wrench6D<> wrench_;
  geometry_msgs::WrenchStamped ft_state_;
};
}  // namespace caros

#endif  // CAROS_FT_SENSOR_SI_PROXY_H
