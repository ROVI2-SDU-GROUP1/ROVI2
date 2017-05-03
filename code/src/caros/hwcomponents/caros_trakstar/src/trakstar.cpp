#include <caros/trakstar.h>
#include <caros/stdafx.h>

#include <rw/rw.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <sys/time.h>

namespace
{
/************************************************************************
*
*  MACROS for simplifying the procedure calls
*
* This macro will set a system parameter and call the error handler if there is
* an error reported. Note These macros do not print to the standard output the
* set value
************************************************************************/

#define SET_SYSTEM_PARAMETER(type, value, l)                         \
  {                                                                  \
    type##_TYPE buffer = value;                                      \
    error_code_ = SetSystemParameter(type, &buffer, sizeof(buffer)); \
    if (error_code_ != BIRD_ERROR_SUCCESS)                           \
      errorHandler(error_code_, l);                                  \
  }

#define SET_SENSOR_PARAMETER(sensor, type, value, l)                          \
  {                                                                           \
    type##_TYPE buffer = value;                                               \
    type##_TYPE *p_buffer = &buffer;                                          \
    error_code_ = SetSensorParameter(sensor, type, p_buffer, sizeof(buffer)); \
    if (error_code_ != BIRD_ERROR_SUCCESS)                                    \
      errorHandler(error_code_, l);                                           \
  }

#define SET_TRANSMITTER_PARAMETER(xmtr, type, value, l)                   \
  {                                                                       \
    type##_TYPE buf = value;                                              \
    type##_TYPE *pBuf = &buf;                                             \
    error_code_ = SetTransmitterParameter(xmtr, type, pBuf, sizeof(buf)); \
    if (error_code_ != BIRD_ERROR_SUCCESS)                                \
      errorHandler(error_code_, l);                                       \
  }

// In order for the above macros to compile without error it is necessary
// to provide typedefs for all the XXX_TYPEs that are generated by "type##_TYPE"
typedef int16_t SELECT_TRANSMITTER_TYPE;
typedef double POWER_LINE_FREQUENCY_TYPE;
// AGC_MODE_TYPE already defined as an enumerated type
typedef double MEASUREMENT_RATE_TYPE;
typedef int16_t REPORT_RATE_TYPE;
typedef double MAXIMUM_RANGE_TYPE;
typedef BOOL METRIC_TYPE;
// DATA_FORMAT_TYPE already defined as an enumerated type
typedef DOUBLE_ANGLES_RECORD ANGLE_ALIGN_TYPE;
typedef DOUBLE_ANGLES_RECORD REFERENCE_FRAME_TYPE;
typedef BOOL XYZ_REFERENCE_FRAME_TYPE;
// HEMISPHERE_TYPE already defined as an enumerated type
typedef BOOL FILTER_AC_WIDE_NOTCH_TYPE;
typedef BOOL FILTER_AC_NARROW_NOTCH_TYPE;
typedef double FILTER_DC_ADAPTIVE_TYPE;
typedef ADAPTIVE_PARAMETERS FILTER_ALPHA_PARAMETERS_TYPE;
typedef BOOL FILTER_LARGE_CHANGE_TYPE;
typedef QUALITY_PARAMETERS QUALITY_TYPE;

/************************************************************************
* This is a simplified error handler.
* This error handler takes the error code and passes it to the GetErrorText()
* procedure along with a buffer to place an error message string.
* This error message string can then be output to a user display device
* like the console
* Specific error codes should be parsed depending on the application.
************************************************************************/
void errorHandler(int error, int lineNum)
{
  char buffer[1024];
  int currentError = error;
  int nextError;

  do
  {
    nextError = GetErrorText(currentError, buffer, sizeof(buffer), SIMPLE_MESSAGE);
    ROS_ERROR_STREAM(buffer);
    currentError = nextError;
  }
  while (currentError != BIRD_ERROR_SUCCESS);
}

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}
}  // namespace

caros::Trakstar::Trakstar() : ATC3DG_(NULL), init_status_(Trakstar::TRAKSTAR_STATUS_INITIALIZING)
{
  ROS_DEBUG("Trakstar constructor ...");
  sensors_attached_ = 0;
  analog_button_on_ = false;
  error_code_ = 0;
  flag_stop_poll_ = false;
}

caros::Trakstar::~Trakstar()
{
  // First stop polling
  stopPolling();
  init_thread_.join();
  CloseBIRDSystem();
}

void caros::Trakstar::initialize(bool block)
{
  // Initialize system by calling _InitializeBird in a thread
  init_thread_ = boost::thread(boost::bind(&Trakstar::initializeSystem, this));
  if (block)
    init_thread_.join();
}

/**
 * Initialize System by calling library function InitializeBIRDSystem.
 * Should run on plugin startup and thus start in a thread so it doesn't increase startup time
 */
int caros::Trakstar::initializeSystem()
{
  ROS_DEBUG("Setting status!");
  TrakstarStatus init_status_local = TRAKSTAR_STATUS_STOPPED;

  ROS_DEBUG("Creating ATC3DG handle!");
  // first initialize variables
  ATC3DG_ = new tagSYSTEM_CONFIGURATION();

  // Set init_status_local to "initializing"

  init_status_local = TRAKSTAR_STATUS_INITIALIZING;
  ROS_DEBUG_STREAM("Init status is: " << init_status_local);
  // Update init_status_ immediately
  init_status_ = init_status_local;

  // Initialize Bird system
  ROS_DEBUG_STREAM("Initializing Trakstar System... This takes some seconds.");
  error_code_ = InitializeBIRDSystem();

  if (error_code_ != BIRD_ERROR_SUCCESS)
  {
    errorHandler(error_code_, __LINE__);
    init_status_local = TRAKSTAR_STATUS_STOPPED;  // Failed. Not initialized

    ROS_ERROR_STREAM("ERROR WHEN INITIALIZING TRAKSTAR");
    ROS_ERROR_STREAM("  error code: " << error_code_);
    ROS_ERROR_STREAM("  Maybe you forgot to start ATCdaemon64?");
    ROS_FATAL("Trakstar cannot function if not ATCdaemon64 is running! closing!");
  }
  else
  {
    // Log::log().info() << "Initialization successfull." << endl;
    ROS_DEBUG_STREAM("Initialization successfull.");

    // Set system parameters

    // Measurement rate, 80 is standard
    // SET_SYSTEM_PARAMETER( MEASUREMENT_RATE, 80, __LINE__ );

    // Metric (use millimeters)
    SET_SYSTEM_PARAMETER(METRIC, true, __LINE__);

    // Range
    // SET_SYSTEM_PARAMETER(MAXIMUM_RANGE, 72.0, __LINE__);

    // Report Rate (how fast does the box prepare new data). reportRate / ( 3*measure_rate ).
    // the RR is actually a report rate devisor. higher => slower report rate.
    // eg. reportrate = 120: 120 / (3*80) = 0.500seconds per update when using GetSynchronousRecord.
    // if using Asynchronous updates, identical values will be read if GetAsynch.. is called faster than data is
    // prepared.
    // Similarly 1 / (3*80) = 4ms per update or approx. 240hz
    // SET_SYSTEM_PARAMETER( REPORT_RATE, 1, __LINE__ ); // 1 is default:

    // SET_SYSTEM_PARAMETER( POWER_LINE_FREQUENCY, 50, __LINE__ );

    error_code_ = GetBIRDSystemConfiguration(ATC3DG_);
    if (error_code_ != BIRD_ERROR_SUCCESS)
    {
      ROS_ERROR_STREAM("ERROR WHEN INITIALIZING: calling GetBIRDSystemConfiguration");
      errorHandler(error_code_, __LINE__);
      init_status_local = TRAKSTAR_STATUS_STOPPED;  // Failed. Not initialized
    }
    else
    {
      ROS_DEBUG_STREAM("Trakstar system configuration information read.");
      ROS_DEBUG_STREAM("Number Boards          = " << ATC3DG_->numberBoards);
      ROS_DEBUG_STREAM("Number Sensors         = " << ATC3DG_->numberSensors);
      ROS_DEBUG_STREAM("Number Transmitters    = " << ATC3DG_->numberTransmitters);
      ROS_DEBUG_STREAM("System AGC mode        = " << ATC3DG_->agcMode);
      ROS_DEBUG_STREAM("Maximum Range          = " << ATC3DG_->maximumRange);
      ROS_DEBUG_STREAM("Measurement Rate       = " << ATC3DG_->measurementRate);
      ROS_DEBUG_STREAM("Metric Mode            = " << ATC3DG_->metric);
      ROS_DEBUG_STREAM("Line Frequency         = " << ATC3DG_->powerLineFrequency);
      ROS_DEBUG_STREAM("Transmitter ID Running = " << ATC3DG_->transmitterIDRunning);

      // Transmitter set:

      DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 180};

      SET_TRANSMITTER_PARAMETER(0, REFERENCE_FRAME, anglesRecord, __LINE__);
      SET_TRANSMITTER_PARAMETER(0, XYZ_REFERENCE_FRAME, true, __LINE__);

      // We now know the number of sensors attached. Setup _record etc.
      raw_values_.resize(ATC3DG_->numberSensors * sizeof(TRAKSTAR_RECORDS_TYPE));
      records_.resize(ATC3DG_->numberSensors);
      records_tmp_.resize(ATC3DG_->numberSensors);

      //
      // GET TRANSMITTER CONFIGURATION
      //
      // The call to GetTransmitterConfiguration() performs a similar task to the
      // GetSensorConfiguration() call. It also returns a status in the filled
      // structure which indicates whether a transmitter is attached to this
      // port or not. In a single transmitter system it is only necessary to
      // find where that transmitter is in order to turn it on and use it.
      //
      transmitters_.resize(ATC3DG_->numberTransmitters);
      for (int i = 0; i < ATC3DG_->numberTransmitters; i++)
      {
        transmitters_[i] = rw::common::ownedPtr(new tagTRANSMITTER_CONFIGURATION());
        error_code_ = GetTransmitterConfiguration(i, transmitters_[i].get());
        if (error_code_ != BIRD_ERROR_SUCCESS)
        {
          errorHandler(error_code_, __LINE__);
          init_status_local = TRAKSTAR_STATUS_STOPPED;  // Failed. Not initialized
        }
        else
        {
          // We have successfully initialized the system.
          init_status_local = TRAKSTAR_STATUS_STARTED;
        }
      }

      // Setup sensors
      sensors_.resize(ATC3DG_->numberSensors);
      for (int i = 0; i < ATC3DG_->numberSensors; i++)
      {
        sensors_[i] = rw::common::ownedPtr(new tagSENSOR_CONFIGURATION());
        error_code_ = GetSensorConfiguration(i, sensors_[i].get());
        if (error_code_ != BIRD_ERROR_SUCCESS)
          errorHandler(error_code_, __LINE__);

        // Hemisphere, top..
        SET_SENSOR_PARAMETER(i, HEMISPHERE, TOP, __LINE__);

        // Data format
        SET_SENSOR_PARAMETER(i, DATA_FORMAT, TRAKSTAR_RECORDS_ENUM_TYPE, __LINE__);

        // filtering
      }
      // Read sensor's configuration
      if (0)
      {
        ROS_DEBUG_STREAM("Sensor's configuration:");
        for (int i = 0; i < ATC3DG_->numberSensors; i++)
        {
          ROS_DEBUG_STREAM("Sensor " << (i + 1) << ":");
          //
          // DATA_FORMAT
          //
          {
            DATA_FORMAT_TYPE buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, DATA_FORMAT, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("DATA_FORMAT: " << buffer);
          }
          //
          // ANGLE_ALIGN
          //
          {
            DOUBLE_ANGLES_RECORD buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, ANGLE_ALIGN, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("ANGLE_ALIGN: " << buffer.a << "," << buffer.e << "," << buffer.r);
          }
          //
          // HEMISPHERE
          //
          {
            HEMISPHERE_TYPE buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, HEMISPHERE, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("HEMISPHERE: " << buffer);
          }
          //
          // FILTER_AC_WIDE_NOTCH
          //
          {
            BOOL buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, FILTER_AC_WIDE_NOTCH, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("FILTER_AC_WIDE_NOTCH: " << buffer);
          }
          //
          // FILTER_AC_NARROW_NOTCH
          //
          {
            BOOL buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, FILTER_AC_NARROW_NOTCH, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("FILTER_AC_NARROW_NOTCH: " << buffer);
          }
          //
          // FILTER_DC_ADAPTIVE
          //
          {
            double buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, FILTER_DC_ADAPTIVE, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("FILTER_DC_ADAPTIVE: " << buffer);
          }
          //
          // FILTER_ALPHA_PARAMETERS
          //
          {
            ADAPTIVE_PARAMETERS buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, FILTER_ALPHA_PARAMETERS, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("FILTER_ALPHA_PARAMETERS:");
            ROS_DEBUG_STREAM("    Alpha max " << buffer.alphaMax[0] << "," << buffer.alphaMax[1] << ","
                                              << buffer.alphaMax[2] << "," << buffer.alphaMax[3] << ","
                                              << buffer.alphaMax[4] << "," << buffer.alphaMax[5] << ","
                                              << buffer.alphaMax[6]);
            ROS_DEBUG_STREAM("    Alpha Min " << buffer.alphaMin[0] << "," << buffer.alphaMin[1] << ","
                                              << buffer.alphaMin[2] << "," << buffer.alphaMin[3] << ","
                                              << buffer.alphaMin[4] << "," << buffer.alphaMin[5] << ","
                                              << buffer.alphaMin[6]);
            ROS_DEBUG_STREAM("    Vm "
                             << "," << buffer.vm[0] << "," << buffer.vm[1] << "," << buffer.vm[2] << "," << buffer.vm[3]
                             << "," << buffer.vm[4] << "," << buffer.vm[5] << "," << buffer.vm[6]);
            ROS_DEBUG_STREAM("    On/Off " << buffer.alphaOn);
          }
          //
          // FILTER_LARGE_CHANGE
          //
          {
            BOOL buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, FILTER_LARGE_CHANGE, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("FILTER_LARGE_CHANGE: " << buffer);
          }
          //
          // QUALITY
          //
          {
            QUALITY_PARAMETERS buffer, *p_buffer = &buffer;
            error_code_ = GetSensorParameter(i, QUALITY, p_buffer, sizeof(buffer));
            if (error_code_ != BIRD_ERROR_SUCCESS)
              errorHandler(error_code_, __LINE__);
            ROS_DEBUG_STREAM("QUALITY: "
                             << "," << buffer.error_offset << "," << buffer.error_sensitivity << ","
                             << buffer.error_slope << "," << buffer.filter_alpha);
          }
        }
      }
    }
  }

  init_status_ = init_status_local;
  return error_code_;
}

caros::Trakstar::TrakstarStatus caros::Trakstar::getInitStatus()
{
  return init_status_;
}

bool caros::Trakstar::isInitialized()
{
  return init_status_ == TRAKSTAR_STATUS_STARTED;
}

int caros::Trakstar::getNumberSensorsAttached()
{
  if (init_status_)
    return sensors_attached_;
  else
    return -1;
}

bool caros::Trakstar::startPolling()
{
  // If we were running. Stop first, and then start.
  if (flag_stop_poll_)
  {
    ROS_DEBUG_STREAM("flag_stop_poll_ flag is " << flag_stop_poll_);
    stopPolling();
  }

  bool ret = false;
  // Search for transmitters. Turn the first one on. (there is only one)
  for (int16_t id = 0; id < ATC3DG_->numberTransmitters; id++)
  {
    if (transmitters_[id]->attached)
    {
      // Transmitter selection is a system function.
      // Using the SELECT_TRANSMITTER parameter we send the id of the
      // transmitter that we want to run with the SetSystemParameter() call
      error_code_ = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
      if (error_code_ != BIRD_ERROR_SUCCESS)
      {
        ROS_DEBUG_STREAM("SetSystemParameter:SELECT_TRANSMITTER" << id << "failed!");
        errorHandler(error_code_, __LINE__);
        return ret;
      }
      break;
    }
  }

  // Set the data format type for each attached sensor.
  for (int16_t i = 0; i < ATC3DG_->numberSensors; i++)
  {
    DATA_FORMAT_TYPE type = TRAKSTAR_RECORDS_ENUM_TYPE;
    error_code_ = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
    if (error_code_ != BIRD_ERROR_SUCCESS)
    {
      ROS_DEBUG_STREAM("SetSystemParameter: DATA_FORMAT for sensor " << i << " failed!");
      errorHandler(error_code_, __LINE__);
    }
  }

  // Count how many sensors are attached by fetching a single record from all and reading status

  error_code_ = GetSynchronousRecord(ALL_SENSORS, &raw_values_[0], raw_values_.size());
  if (error_code_ != BIRD_ERROR_SUCCESS)
  {
    errorHandler(error_code_, __LINE__);
  }

  // Read sensorStatus
  sensors_attached_ = 0;
  std::stringstream attached_sensors_ss;
  attached_sensors_ss << "Attached sensors: ";
  bool firstRun = true;
  for (int i = 0; i < ATC3DG_->numberSensors; i++)
  {
    unsigned int status = GetSensorStatus(i);
    if ((status & NOT_ATTACHED) != 0)
      continue;
    sensors_attached_++;
    if (!firstRun)
      attached_sensors_ss << ", ";
    firstRun = false;
    attached_sensors_ss << (i + 1);
  }
  ROS_DEBUG_STREAM(attached_sensors_ss.str());

  flag_stop_poll_ = false;
  // Start polling thread
  ret = true;

  return ret;
}

void caros::Trakstar::pollData()
{
  // scan the sensors (all) non blocking call

  error_code_ = GetSynchronousRecord(ALL_SENSORS, &raw_values_[0], raw_values_.size());
  if (error_code_ != BIRD_ERROR_SUCCESS)
    errorHandler(error_code_, __LINE__);

  const TRAKSTAR_RECORDS_TYPE *raw_records = reinterpret_cast<TRAKSTAR_RECORDS_TYPE *>(&raw_values_[0]);
  // Get status of sensors (only updates after a Get***Record call)
  for (int16_t id = 0; id < ATC3DG_->numberSensors; id++)
  {
    // get the status of the last data record
    // only report the data if everything is okay
    ULONG status = GetSensorStatus(id);

    // Set default state
    records_tmp_[id].status = status;
    records_tmp_[id].valid = false;
    if (status == VALID_STATUS)
    {
      analog_button_on_ = static_cast<bool>(raw_records[id].button);
      records_tmp_[id].analog_button_on = analog_button_on_;

      // Copy raw data into records
      records_tmp_[id].pos[0] = raw_records[id].x;
      records_tmp_[id].pos[1] = raw_records[id].y;
      records_tmp_[id].pos[2] = raw_records[id].z;
      records_tmp_[id].rot(3) = raw_records[id].q[0];  // Scalar component
      records_tmp_[id].rot(0) = raw_records[id].q[1];  // qx
      records_tmp_[id].rot(1) = raw_records[id].q[2];  // qy
      records_tmp_[id].rot(2) = raw_records[id].q[3];  // qz
      records_tmp_[id].time = raw_records[id].time;
      records_tmp_[id].quality = raw_records[id].quality * 1.0 / 65536.0;
      records_tmp_[id].valid = true;
    }
    else if (status == (SATURATED | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Sensor[" << id << "] is saturated.");
    }
    else if (status == (OUT_OF_MOTIONBOX | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Sensor[" << id << "] is out of range.");
    }
    else if (status == (NOT_ATTACHED | GLOBAL_ERROR))
    {
      // Don't tell us that a sensor is not attached. We probably(hopefully!) know
      // This would be a place to debug for sensor-data not read if that error present.
      // ROS_ERROR_STREAM("No sensor [" << id << "] attached! ");
    }
    else if (status == (NO_TRANSMITTER_RUNNING | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Transmitter not ready!");
    }
    else
    {
      ROS_ERROR_STREAM("Sensor[" << id << "]: status not valid: " << status - 1);
    }
  }

  records_ = records_tmp_;
}

void caros::Trakstar::stopPolling()
{
  // Signal to _poll to stop polling
  flag_stop_poll_ = true;

  // Get active transmitter
  int16_t buffer, *p_buffer = &buffer;
  error_code_ = GetSystemParameter(SELECT_TRANSMITTER, p_buffer, sizeof(buffer));
  if (error_code_ != BIRD_ERROR_SUCCESS)
  {
    errorHandler(error_code_, __LINE__);
    return;
  }

  // Stop transmitter if one was turned on
  if (buffer != -1)
  {
    int16_t id = -1;
    error_code_ = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (error_code_ != BIRD_ERROR_SUCCESS)
      errorHandler(error_code_, __LINE__);
  }
}

std::vector<caros::Trakstar::PoseData> caros::Trakstar::getData(void)
{
  pollData();
  return records_;
}

std::string caros::Trakstar::getSensorStatusString(int error_code)
{
  if (error_code == VALID_STATUS)
  {
    return "OK";
  }
  else if (error_code & NOT_ATTACHED)
  {
    return "NOT CONNECTED";
  }
  else if (error_code & OUT_OF_MOTIONBOX)
  {
    return "RANGE";
  }
  else if (error_code & SATURATED)
  {
    return "SATURATED";
  }

  // else
  std::stringstream ss;
  ss << "e" << error_code;
  return ss.str();
}

int caros::Trakstar::getNumberSensorsSupported()
{
  return ATC3DG_->numberSensors;
}
