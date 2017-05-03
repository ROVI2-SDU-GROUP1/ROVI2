#ifndef CAROS_POSE_SENSOR_SI_PROXY_H
#define CAROS_POSE_SENSOR_SI_PROXY_H

#include <caros_sensor_msgs/ButtonSensorState.h>
#include <caros_sensor_msgs/PoseSensorState.h>

#include <rw/math.hpp>

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <vector>
#include <string>

namespace caros
{
/**
 * @brief This class implements a C++ proxy to control and read data from
 * a PoseSensorServiceInterface.
 */
class PoseSensorSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<PoseSensorSIProxy> Ptr;

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the node
   * @param[in] use_persistent_connections Define usage of persistent connections
   */
  PoseSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                    const bool use_persistent_connections = true);

  //! destructor
  virtual ~PoseSensorSIProxy();

  //! pose data structure
  struct PoseData
  {
    rw::math::Transform3D<> pose;
    int id;
    float quality;
    ros::Time stamp;
    std::string frame;
  };

  //! get current pose state
  std::vector<PoseData> getPoses();

  //! get time stamp of current pose state
  ros::Time getTimeStamp();

 protected:
  void handlePoseSensorState(const caros_sensor_msgs::PoseSensorState& state);

  ros::NodeHandle nodehandle_;
  // states
  ros::Subscriber pose_sensor_state_sub_;

  std::mutex mutex_;
  // state variables
  std::vector<PoseData> poses_;
  ros::Time stamp_;
};
}  // namespace caros

#endif  // CAROS_POSE_SENSOR_SI_PROXY_H
