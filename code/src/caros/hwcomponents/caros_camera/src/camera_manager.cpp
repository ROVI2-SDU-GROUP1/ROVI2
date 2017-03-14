#include <caros/camera_manager.h>

#ifdef BASLER_SUPPORTED
#include <caros/basler_camera.h>
#endif
#ifdef BUMBLEBEE_SUPPORTED
#include <caros/pointgrey_camera.h>
#endif
#ifdef ENSENSO_SUPPORTED
#include <caros/ensenso_camera.h>
#endif

#include <ros/ros.h>
#include <vector>

namespace caros
{
CameraManager::CameraManager()
{
}

CameraManager::~CameraManager()
{
}

int CameraManager::getCameraType()
{
  return camera_type_;
}

std::shared_ptr<CameraInterface> CameraManager::getCamera(unsigned int serial, int camera_type)
{
  camera_type_ = camera_type;

  switch (camera_type)
  {
#ifdef BASLER_SUPPORTED
    case BASLER:
      return std::make_shared<BaslerCamera>(std::to_string(serial));
#endif
#ifdef BUMBLEBEE_SUPPORTED
    case BUMBLEBEE:
      return std::make_shared<PointGreyCamera>(serial);
#endif
#ifdef ENSENSO_SUPPORTED
    case ENSENSO:
      return std::make_shared<pcl::EnsensoCamera>(std::to_string(serial));
#endif
    default:
      return nullptr;
  }
}

}  // namespace caros
