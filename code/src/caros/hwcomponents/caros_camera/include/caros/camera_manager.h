#ifndef CAROS_CAMERA_MANAGER_H
#define CAROS_CAMERA_MANAGER_H

#include <caros/camera_interface.h>
#include <vector>

namespace caros
{
class CameraManager
{
 public:
  CameraManager();
  ~CameraManager();

  enum CAMERA_TYPE
  {
    BASLER = 0,
    BUMBLEBEE,
    ENSENSO
  };

  int getCameraType();

  std::shared_ptr<CameraInterface> getCamera(unsigned int serial, int camera_type);

 protected:
  int camera_type_;
};
}  // namespace caros

#endif  // CAROS_CAMERA_MANAGER_H
