#ifndef CAROS_SERVICE_INTERFACE_PROXY_TEST_SETUP_H
#define CAROS_SERVICE_INTERFACE_PROXY_TEST_SETUP_H

#include <caros/exceptions.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <functional>
#include <tuple>
#include <memory>

namespace caros
{
namespace test
{
/**
 * \addtogroup Test Service Interface Proxy Testing Infrastructure
 * Overview:
 * - __testServices__ is the function that is supposed to be called from the test suite(s)
 * - __TestType__ that lists the supported types of tests (e.g. testor functions)
 * - __testorFunctions__ that are used to perform the actual tests
 * - __testWrapper__ that takes care of iterating through the services that should be tested and call the appropriate
 *testor function
 * - __createTestConfiguration__ properly sets up the test configuration with the different parameters required for
 *performing the chosen test type
 *
 * @{
 **/
/************************************************************************
 * Testor
 * Functions that perform the actual tests
 ************************************************************************/
/**
 * @brief configuration for the testor functions
 *
 * Parameters that should be used for creating the services backend that is to be used for testing the given service
 *interface proxy.
 * And the function to invoke for requesting the service
 */
struct TestorConfiguration
{
  std::function<bool()> service_func;
  std::string expected_service_called;
  bool expected_return_value;
  std::function<const std::string&()> get_service_called_func;
  std::function<void()> close_persistent_connection_func;
};
//! Typedef for the signature of a testor function
typedef std::function<void(const TestorConfiguration&)> Testor;

/**
 * @brief Test the return value from invoking the service
 */
void testorReturnValue(const TestorConfiguration& conf)
{
  bool actual_return_value = false;
  EXPECT_NO_THROW(actual_return_value = conf.service_func());

  EXPECT_EQ(conf.expected_return_value, actual_return_value);

  EXPECT_EQ(conf.expected_service_called, conf.get_service_called_func());
}

/**
 * @brief Test that an unavailable service causes the SIP to throw an caros::UnavailableService exception
 */
/* EXPECT_THROW doesn't work well with templated exception types, as it outputs the literal name of the exception
 * identifier that is specified */
void testorUnavailableService(const TestorConfiguration& conf)
{
  EXPECT_THROW(conf.service_func(), caros::UnavailableService);
}

/**
 * @brief Test that an (unknown) error caused when requesting a service, causes the SIP to throw an
 * caros::BadServiceCall exception
 */
void testorBadServiceCall(const TestorConfiguration& conf)
{
  EXPECT_THROW(conf.service_func(), caros::BadServiceCall);
}

/**
 * @brief Test that a persistent connection is automatically reestablished after it has been shut down
 *
 * Requires the use of a function to manually shut down the established persistent connections.
 */
void testorReestablishPersistentConnection(const TestorConfiguration& conf)
{
  bool actual_return_value = false;
  EXPECT_NO_THROW(actual_return_value = conf.service_func());
  EXPECT_EQ(conf.expected_return_value, actual_return_value);
  EXPECT_EQ(conf.expected_service_called, conf.get_service_called_func());

  conf.close_persistent_connection_func();

  actual_return_value = false;
  EXPECT_NO_THROW(actual_return_value = conf.service_func());
  EXPECT_EQ(conf.expected_return_value, actual_return_value);
  EXPECT_EQ(conf.expected_service_called, conf.get_service_called_func());
}

/************************************************************************
 * Test wrapper
 * (also being the services iterator)
 ************************************************************************/
/**
 * @brief configuration for the testWrapper function
 *
 * Parameters to use when constructing the specified service interface backend (also called a service interface dummy)
 */
struct TestWrapperConfiguration
{
  bool return_value_to_test;
  bool cause_error;
  bool use_service_interface_dummy;
  Testor testor_func;
};

/**
 * @brief iterates through the services to be requested, and prepares and invokes the assigned testorFunction
 * @tparam D The service interface dummy (i.e. backend) type to use
 * @tparam P The service interface proxy to use
 * @tparam C The container holding the services to be requested
 * @param[in] services The services to be requested
 * @param[in] conf The test wrapper configuration to use
 */
template <typename D, typename P, typename C>
void testWrapper(const C& services, const TestWrapperConfiguration& conf)
{
  ros::NodeHandle nodehandle_service("si_dummy");
  std::shared_ptr<D> si_dummy(nullptr);
  if (conf.use_service_interface_dummy)
  {
    // Create the service interface dummy with the required parameters/configuration
    si_dummy = std::make_shared<D>(nodehandle_service, conf.return_value_to_test, conf.cause_error);
  }

  // Spawn 1 spinner thread
  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  /* Make sure to test the usage of both persistent and non-persistent connections for every test case */
  const bool boolean_values[] = {true, false};
  for (const auto use_persistent_connections : boolean_values)
  {
    ros::NodeHandle nodehandle_client("sip");
    P sip(nodehandle_client, "si_dummy", use_persistent_connections);

    for (const auto& service : services)
    {
      TestorConfiguration testor_conf;
      testor_conf.expected_return_value = conf.return_value_to_test;
      testor_conf.expected_service_called = std::get<1>(service);

      // Bind the SIProxy member function to be called on the sip object (std::bind will implicitly make a copy of the
      // sip object, so using std::ref to make a copy of a reference to the object - invoking the function on the
      // original sip object)
      auto service_func = std::get<0>(service);
      testor_conf.service_func = std::bind(service_func, std::ref(sip));

      // Bind functionality to close/reset persistent connections, allowing for testing if a persistent connection can
      // be properly reestablished
      testor_conf.close_persistent_connection_func = std::bind(&P::closePersistentConnections, std::ref(sip));

      // See comment for previous std::bind and std::ref (however not necessary if using a shared_ptr, as that can
      // easily be copied and still point to the same object)
      testor_conf.get_service_called_func = std::bind(&D::getMostRecentFunctionCalled, si_dummy);

      conf.testor_func(testor_conf);
    }

    nodehandle_client.shutdown();
  }
  /* End */
  nodehandle_service.shutdown();

  spinner.stop();
}

/************************************************************************
 * Convenience functions
 ************************************************************************/
/**
 * @brief The types of available tests
 */
enum class TestType
{
  ReturnTrue,
  ReturnFalse,
  BadServiceCall,
  UnavailableService,
  ReestablishPersistentConnection
};

/**
 * @brief create the test wrapper configuration accordingly to the chosen TestType
 * @param[in] test_type The test that should be performed
 */
TestWrapperConfiguration createTestConfiguration(const TestType test_type)
{
  TestWrapperConfiguration conf;
  /* Default values */
  conf.return_value_to_test = true;
  conf.cause_error = false;
  conf.use_service_interface_dummy = true;

  switch (test_type)
  {
    case TestType::ReturnTrue:
      conf.return_value_to_test = true;
      conf.testor_func = std::bind(testorReturnValue, std::placeholders::_1);
      break;
    case TestType::ReturnFalse:
      conf.return_value_to_test = false;
      conf.testor_func = std::bind(testorReturnValue, std::placeholders::_1);
      break;
    case TestType::BadServiceCall:
      conf.cause_error = true;
      conf.testor_func = std::bind(testorBadServiceCall, std::placeholders::_1);
      break;
    case TestType::UnavailableService:
      conf.use_service_interface_dummy = false;
      conf.testor_func = std::bind(testorUnavailableService, std::placeholders::_1);
      break;
    case TestType::ReestablishPersistentConnection:
      conf.testor_func = std::bind(testorReestablishPersistentConnection, std::placeholders::_1);
      break;
    default:
      throw std::runtime_error("Unsupported TestType enum!");
  }

  return conf;
}

/**
 * @brief The function to call when wanting to test the provided services according to the specified TestType
 * @tparam D The service interface dummy (i.e. backend) type to use
 * @tparam P The service interface proxy to use
 * @tparam C The container holding the services to be requested. The container could be something like a vector or list
 * and should contain a std::tuple holding (at least) 2 elements; the service to request (with the form:
 * std::function<bool(caros::SIP &)>) and a string that should match the service that is invoked in the backend/dummy.
 * See the actual tests for more concrete and better examples.
 * @param[in] services container holding the services to be requested
 * @param[in] test_type the type of test to perform on the services
 */
template <typename D, typename P, typename C>
void testServices(const C& services, const TestType test_type)
{
  auto conf = createTestConfiguration(test_type);

  testWrapper<D, P, C>(services, conf);
}
/**
 * @}
 */
}  // namespace test
}  // namespace caros

#endif  // CAROS_SERVICE_INTERFACE_PROXY_TEST_SETUP_H
