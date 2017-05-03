#include <caros/camera_interface.h>

#include <ros/ros.h>
#include <string>

namespace caros
{
CameraInterface::CameraInterface()
{
}

CameraInterface::CameraInterface(unsigned int serial)
{
  fw_serial_ = serial;
}

CameraInterface::CameraInterface(const std::string &serial)
{
  serial_ = serial;
}

CameraInterface::~CameraInterface()
{
}

}  // namespace caros
