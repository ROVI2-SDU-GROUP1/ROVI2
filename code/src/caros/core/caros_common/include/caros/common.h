#ifndef CAROS_COMMON_H
#define CAROS_COMMON_H

/**
 * \brief CAROS specific functionality
 */
namespace caros
{
/**
 * \addtogroup TypeConversion CAROS Type Conversion
 * Overloaded utility functions for converting between the different system types (e.g. from ROS to RobWork)
 * If a type can be converted to more types, then the most obvious/direct type conversion is having the toRos or toRw
 * signature, while the alternative types will be toRosType and toRwType.
 * @{
 */

//! convert double to double
double toRos(const double value);

//! convert double to ros float
float toRosFloat(const double value);

//! convert float to ros double
double toRosDouble(const float value);

//! convert float to float
float toRos(const float value);

//! convert bool to bool
bool toRos(const bool value);

/**
 * @}
 */

/**
 * \addtogroup Utility CAROS Utility Functions
 * Utility functions for various CAROS related tasks, such as obtaining a workcell
 * @{
 */
/**
 * @brief Verify that the value is within the range [min;max]. If the value is not within the range, then appropriate
exceptions will be thrown - behaving mostly as a runtime assertion.
 * @param[in] value the value to be compared
 * @param[in] min the lower boundary of the range
 * @param[in] max the upper boundary of the range
 *
 * @throws std::invalid_argument if any of the supplied values are NaN's
 * @throws std::range_error if the value is not within the specified range [min;max]
 */
/* Not doing this as a do {...} while(0) macro, as there then wouldn't be any type enforcement */
void verifyValueIsWithin(const float& value, const float& min, const float& max);

/**
 * @}
 */
}  // namespace caros

#endif  // CAROS_COMMON_H
