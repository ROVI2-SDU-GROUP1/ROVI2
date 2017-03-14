#ifndef CAROS_CAROS_SERVICE_CLIENT_H
#define CAROS_CAROS_SERVICE_CLIENT_H

#include <caros/exceptions.h>

#include <ros/ros.h>

#include <string>

/* Fully defined in the header file as the major functions are templated */

namespace caros
{
/**
 * \addtogroup SIP CAROS SIP Functionality
 */
/**
 * \ingroup SIP
 * @brief this class implements a wrapper around the ros::ServiceClient class/object and provides builtin reconnection
 *functionality for persistent connections.
 *
 * Connections are not created before they are actually being used (e.g. one of the call functions)
 */
class CarosServiceClient
{
 public:
  /**
   * @brief Constructor
   * @param[in] nodehandle used to obtain/create ros::ServiceClient object (could be converted into a version that
   * doesn't require a nodehandle)
   * @param[in] connection_identifier specifying the service name to connect to (excluding the namespace)
   * @param[in] service_namespace specifying the namespace of where the service is to be found
   * @param[in] use_persistent_connection specify the persistence of the connections (this can be changed at run time)
   */
  CarosServiceClient(ros::NodeHandle nodehandle, const std::string& connection_identifier,
                     const std::string& service_namespace, const bool use_persistent_connection)
      : nodehandle_(nodehandle),
        connection_identifier_(connection_identifier),
        use_persistent_connection_(use_persistent_connection)
  {
    service_name_ = service_namespace + "/" + connection_identifier_;
  }

  /**
   * @brief Destructor
   */
  virtual ~CarosServiceClient()
  { /* Nothing for now */
  }

  /**
   * @brief call the service
   * @param[in] srv the service communication type
   * @param[in] persistence use persistent (i.e. true) or non-persistent (i.e. false) connection
   */
  template <typename T>
  bool call(T& srv, const bool persistence)
  {
    use_persistent_connection(persistence);
    return call<T>(srv);
  }

  /**
   * @brief call the service
   * @param[in] srv the service communication type
   */
  template <typename T>
  bool call(T& srv)
  {
    prepareConnection<T>();

    bool srv_call_success = false;

    if (!service_client_.exists())
    {
      THROW_CAROS_UNAVAILABLE_SERVICE("The service " << service_client_.getService() << " does not exist.");
    }

    srv_call_success = service_client_.call(srv);
    if (!srv_call_success)
    {
      THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << service_client_.getService());
    }

    return srv_call_success;
  }

  /**
   * @brief make sure the connection is shutdown if necessary (i.e. a persistent connection that is still valid)
   */
  void shutdown()
  {
    if (service_client_.isPersistent() && service_client_.isValid())
    {
      ROS_DEBUG_STREAM("Shutting down persistent connection (id: " << connection_identifier_ << ")");
      service_client_.shutdown();
    }
  }

  /**
   * @brief change persistence of the connection(s) to be established
   * @param[in] persistence use persistent (i.e. true) or non-persistent (i.e. false) connection
   */
  void use_persistent_connection(const bool persistence)
  {
    use_persistent_connection_ = persistence;
  }

  /**
   * @brief is it currently set to use persistent or non-persistent connection(s)
   * @returns a boolean indicating the persistence (i.e. true for persistent connections and false for non-persistent
   * connections)
   */
  bool isUsingPersistentConnection()
  {
    return use_persistent_connection_;
  }

 protected:
  /**
   * @brief Make sure to setup an appropriate connection or change it based on changed persistence preferences
   */
  template <typename T>
  void prepareConnection()
  {
    bool create_new_connection = false;

    /* The ordering is important to make sure that the switch between persistent and non-persistent connection is
     * happening */
    /* Switch from persistent to non-persistent connection */
    if (service_client_.isPersistent() && !use_persistent_connection_)
    {
      ROS_DEBUG_STREAM("Switching from persistent to non-persistent connection (id: " << connection_identifier_ << ")");
      if (service_client_.isValid())
      {
        ROS_DEBUG_STREAM("Shutting down the old persistent connection (id: " << connection_identifier_ << ")");
        service_client_.shutdown();
      }
      create_new_connection = true;
    }
    /* Switch from non-persistent to persistent connection */
    /* the .isValid() test is to make sure that this case is not wrongly chosen when a new persistent connection should
     * be created */
    else if (!service_client_.isPersistent() && service_client_.isValid() && use_persistent_connection_)
    {
      ROS_DEBUG_STREAM("Switching from non-persistent to persistent connection (id: " << connection_identifier_ << ")");
      create_new_connection = true;
    }
    /* Reestablish the persistent connection */
    else if (service_client_.isPersistent() && !service_client_.isValid())
    {
      ROS_DEBUG_STREAM("Reconnecting the persistent connection with (id: " << connection_identifier_ << ")");
      create_new_connection = true;
    }
    /* No previous connection setup */
    else if (!service_client_.isValid())
    {
      ROS_DEBUG_STREAM("Setting up new connection (id: " << connection_identifier_ << ")");
      create_new_connection = true;
    }

    if (create_new_connection)
    {
      /* Can use the ros::service::createClient call and not have to rely on having a nodehandle */
      // service_client_ = ros::service::createClient<T>(service_name_, use_persistent_connection_);
      service_client_ = nodehandle_.serviceClient<T>(service_name_, use_persistent_connection_);
    }
  }

 protected:
  ros::NodeHandle nodehandle_;
  ros::ServiceClient service_client_;
  std::string connection_identifier_;
  std::string service_name_;
  bool use_persistent_connection_;
};

}  // namespace caros

#endif  // CAROS_CAROS_SERVICE_CLIENT_H
