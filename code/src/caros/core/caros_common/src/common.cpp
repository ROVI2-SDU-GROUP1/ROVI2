#include <caros/common.h>

#include <ros/ros.h>

#include <cstdint>
#include <cmath>
#include <sstream>

namespace caros
{
/************************************************************************
 * Double
 ************************************************************************/
double toRos(const double value)
{
  return value;
}

/************************************************************************
 * Double to float
 * Float to double
 ************************************************************************/
float toRosFloat(const double value)
{
  return static_cast<float>(value);
}

double toRosDouble(const float value)
{
  return static_cast<double>(value);
}

/************************************************************************
 * Float
 ************************************************************************/
float toRos(const float value)
{
  return value;
}

/************************************************************************
 * Boolean
 ************************************************************************/
bool toRos(const bool value)
{
  return value;
}

/************************************************************************
 * Utility
 ************************************************************************/
void verifyValueIsWithin(const float& value, const float& min, const float& max)
{
  ROS_DEBUG_STREAM("Verifying that the value '" << value << "' is within [" << min << ";" << max << "]");
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}

}  // namespace caros
