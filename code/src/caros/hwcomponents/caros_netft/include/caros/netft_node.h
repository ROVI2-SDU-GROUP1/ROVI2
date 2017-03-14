#ifndef CAROS_NETFT_NODE_H
#define CAROS_NETFT_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/ft_sensor_service_interface.h>

#include <rwhw/netft/NetFTLogging.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include <ros/ros.h>
#include <string>

namespace caros
{
/**
 * @brief Ros node for accessing NetFT sensor.
 */
class NetFTNode : public caros::CarosNodeServiceInterface, public caros::FTSensorServiceInterface
{
 public:
  // constructor
  explicit NetFTNode(const ros::NodeHandle& node_handle);

  // destructor
  virtual ~NetFTNode();

  enum NETFTNODE_ERRORCODE
  {
    NETFT_UNABLE_TO_START_COMMUNICATION,
    NETFT_INTERNAL_ERROR,
    NETFT_CAROS_FT_SENSOR_SERVICE_CONFIGURE_FAIL
  };

 protected:
  // hooks implemented from CarosNodeServiceInterface base class
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  // convenience functions
  bool isInWorkingCondition();
  void lookupParamWithDefaultDebugMsg(const std::string& param);

 private:
  rwhw::NetFTLogging::Ptr netft_;
  ros::NodeHandle node_handle_;
  std::string ip_;
  int port_;
  int publish_rate_;
};

}  // namespace caros

#endif  // CAROS_NETFT_NODE_H
