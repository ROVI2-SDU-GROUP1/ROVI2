#ifndef CAROS_EXCEPTIONS_H
#define CAROS_EXCEPTIONS_H

#include <stdexcept>
#include <string>
#include <sstream>

/**
 * \addtogroup Exceptions CAROS Exceptions
 * Use the THROW_CAROS_<...> macros to throw the corresponding exceptions.
 *
 * THROW_CAROS_UNAVAILABLE_SERVICE(...) and THROW_CAROS_BAD_SERVICE_CALL(...) are primarily for use within service
 *interface proxies (SIP), and the corresponding exceptions are to be caught in the client code making use of the SIP.
 * @{
 */

/*
 * @brief Throw an UnavailableService exception with the message \b ostream_expression.
 *
 * \b ostream_expression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_UNAVAILABLE_SERVICE("The service " << service_name << " is unavailable.");
 * \endcode
 */
#define THROW_CAROS_UNAVAILABLE_SERVICE(ostream_expression) \
  do                                                        \
  {                                                         \
    std::ostringstream CAROS__message;                      \
    CAROS__message << ostream_expression;                   \
    throw caros::UnavailableService(CAROS__message.str());  \
  } while (0)

/*
 * @brief Throw a BadServiceCall exception with the message \b ostream_expression.
 *
 * \b ostream_expression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_BAD_SERVICE_CALL("An unexpected error happened while calling the service " << service_name);
 * \endcode
 */
#define THROW_CAROS_BAD_SERVICE_CALL(ostream_expression) \
  do                                                     \
  {                                                      \
    std::ostringstream CAROS__message;                   \
    CAROS__message << ostream_expression;                \
    throw caros::BadServiceCall(CAROS__message.str());   \
  } while (0)

/**
 * @}
 */

namespace caros
{
/**
 * \addtogroup Exceptions
 * @{
 */

/**
 * @brief unavailable service exception.
 *
 * Used when the requested service is unavailable.
 */
class UnavailableService : public std::runtime_error
{
 public:
  explicit UnavailableService(const std::string& what) : runtime_error(what)
  {
    /* Empty */
  }

  virtual ~UnavailableService() throw()
  {
    /* Empty */
  }
};

/**
 * @brief bad service call exception.
 *
 * Used when an (unknown) error occured while requesting a service.
 */
class BadServiceCall : public std::runtime_error
{
 public:
  explicit BadServiceCall(const std::string& what) : runtime_error(what)
  {
    /* Empty */
  }
  virtual ~BadServiceCall() throw()
  {
    /* Empty */
  }
};

/**
 * @}
 */
}  // namespace caros

#endif  // CAROS_EXCEPTIONS_H
