#ifndef CAROS_TRAKSTAR_H
#define CAROS_TRAKSTAR_H

#include <ros/ros.h>
#include "ATC3DG.h"
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <boost/thread/thread.hpp>

#include <cstdint>
#include <vector>
#include <string>

namespace caros
{
/**
 * @brief this class serves as a cpp wrapper for the trakstar driver interface
 */
// Get all information possible
typedef DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
#define TRAKSTAR_RECORDS_ENUM_TYPE DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON

class Trakstar
{
 public:
  //! init status defines
  typedef enum
  {
    TRAKSTAR_STATUS_STOPPED = -1  //! error occurred and initialization stopped
    ,
    TRAKSTAR_STATUS_INITIALIZING = 0  //! still initializing
    ,
    TRAKSTAR_STATUS_STARTED = 1  //!
  }
  TrakstarStatus;
  //! forward declaration
  struct PoseData;

 public:
  //! constructor
  Trakstar();
  //! destructor
  virtual ~Trakstar();

  /**
   * @brief initialize the trakstar sensor.
   *
   * This may take several 10's of seconds. Hence
   * an option for performing the call non blocking is added. Use isInitialized() to
   * check if initialization is done. And getInitStatus() in order to know if
   * something blocked the initialization.
   */
  void initialize(bool block = true);

  //! check if driver is initialized
  bool isInitialized();

  //! get status of initialization
  TrakstarStatus getInitStatus();

  //! get data of all pose sensors
  std::vector<PoseData> getData(void);

  /**
   * @brief start the transmitter and start polling the poses of all
   * active sensors.
   * @return true if polling started successfully, else false.
   */
  bool startPolling();
  void stopPolling();
  bool isPolling()
  {
    return !flag_stop_poll_;
  }

  int getNumberSensorsAttached();
  int getNumberSensorsSupported();

  std::string getSensorStatusString(int errorcode);

 public:
  struct PoseData
  {
    rw::math::Vector3D<> pos;
    rw::math::Quaternion<> rot;
    double time;
    double quality;  // quality from 0 to 1
    bool valid;
    uint32_t status;
    bool calib_status;
    bool analog_button_on;
  };

 private:
  struct tagSYSTEM_CONFIGURATION *ATC3DG_;  // a pointer to a single instance of the system class
  // a pointer to an array of sensor objects
  std::vector<rw::common::Ptr<struct tagSENSOR_CONFIGURATION>> sensors_;
  // a pointer to an array of transmitter objects
  std::vector<rw::common::Ptr<struct tagTRANSMITTER_CONFIGURATION>> transmitters_;
  std::vector<TRAKSTAR_RECORDS_TYPE> raw_values_;

  std::vector<PoseData> records_, records_tmp_;

  int error_code_;  // used to hold error code returned from procedure call

  int sensors_attached_;  // Is updated with the actual number of connected sensors at startPolling()

  boost::thread init_thread_;

  bool flag_stop_poll_;

  bool analog_button_on_;

  TrakstarStatus init_status_;

  int initializeSystem();
  void pollData();
};
}  // namespace caros
#endif  // CAROS_TRAKSTAR_H
