#ifndef CAROS_TEST_UNIVERSAL_ROBOT_TEST_H
#define CAROS_TEST_UNIVERSAL_ROBOT_TEST_H

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <ros/ros.h>

#include <string>
#include <stdexcept>

/* WARNING: USE AT YOUR OWN RISK!
 * None of this is code is guaranteed to not make the robot crash into anything or do unexpected behaviour!
 * It is highly recommended that you understand what the code does before you run it!
 *
 * To minimise risk, please use (very) small q_change values!
 */

/* This class is supposed to make it a bit easier to reuse the test code.
 * To use another planner or collision detector, simply add a member function that the user can invoke and override the
 * defaults (before invoking the test functions)
 */
class UrTest
{
 public:
  UrTest() : nodehandle_("~"), sdsip_(nodehandle_, "ur_simple_demo_node")
  {
    initWorkCell();
    initDevice();
    initPathPlannerWithCollisionDetector();
  }

  virtual ~UrTest()
  {
    /* Empty */
  }

  bool testMovePtp(const double q_change)
  {
    if (!doTestMovePtp(q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    if (!doTestMovePtp(-q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    return true;
  }

  bool testMoveServoQ(const double q_change)
  {
    if (!doTestMoveServoQ(q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    if (!doTestMoveServoQ(-q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    return true;
  }

 protected:
  void initWorkCell()
  {
    workcell_ = caros::getWorkCell();
    if (workcell_ == NULL)
    {
      ROS_ERROR("No workcell was loaded - exiting...");
      throw std::runtime_error("Not able to obtain a workcell.");
    }
  }

  void initDevice()
  {
    std::string device_name;
    if (!nodehandle_.getParam("device_name", device_name))
    {
      ROS_FATAL_STREAM("The parameter '" << nodehandle_.getNamespace()
                                         << "/device_name' was not present on the parameter "
                                            "server! This parameter has to be specified "
                                            "for this test-node to work properly.");
      throw std::runtime_error("Not able to obtain device name.");
    }

    ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
    device_ = workcell_->findDevice(device_name);
    if (device_ == NULL)
    {
      ROS_FATAL_STREAM("Unable to find device " << device_name << " in the loaded workcell");
      throw std::runtime_error("Not able to find the device within the workcell.");
    }
  }

  void initPathPlannerWithCollisionDetector()
  {
    rw::kinematics::State state = workcell_->getDefaultState();
    /* Collision detector */
    auto detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(workcell_, rwlibs::proximitystrategies::ProximityStrategyYaobi::make()));
    /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
     * collision free and that the edge(s) between those is/are also collision free. */
    const rw::pathplanning::PlannerConstraint planner_constraint =
        rw::pathplanning::PlannerConstraint::make(detector, device_, state);

    /* Just using a really simple path planner (straight line in the configuration space) */
    planner_ = rw::pathplanning::QToQPlanner::make(planner_constraint);
  }

  rw::math::Q getCurrentJointConfiguration()
  {
    /* Make sure to get and operate on fresh data from the serial device
     * It's assumed that the serial device is not moving
     * ^- That could be asserted / verified using sdsip.isMoving()
     * However other sources could invoke services on the UR that causes it to move...
     */
    ros::Time current_timestamp = ros::Time::now();
    ros::Time obtained_timestamp = sdsip_.getTimeStamp();
    while (current_timestamp > obtained_timestamp)
    {
      ros::Duration(0.1).sleep();  // In seconds
      ros::spinOnce();
      obtained_timestamp = sdsip_.getTimeStamp();
    }

    return sdsip_.getQ();
  }

  rw::trajectory::QPath getQPath(const double q_change)
  {
    rw::math::Q start_configuration = getCurrentJointConfiguration();
    rw::math::Q end_configuration = start_configuration + rw::math::Q(start_configuration.size(), q_change);

    rw::trajectory::QPath path;
    bool valid_path = false;
    ROS_ASSERT(planner_);
    valid_path = planner_->query(start_configuration, end_configuration, path);

    if (!valid_path)
    {
      ROS_ERROR_STREAM("Could not find a path from '" << start_configuration << "' to '" << end_configuration << "'.");
      throw std::runtime_error("No valid path found.");
    }

    return path;
  }

  rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end,
                                               const double total_duration = 10.0, const double duration_step = 1.0)
  {
    ROS_ASSERT(duration_step > 0);
    ROS_ASSERT(duration_step < total_duration);

    rw::trajectory::QLinearInterpolator interpolator(start, end, total_duration);

    rw::trajectory::QPath path;

    path.push_back(start);
    for (double t = duration_step; t <= (total_duration - duration_step); t += duration_step)
    {
      path.push_back(interpolator.x(t));
    }
    path.push_back(end);

    return path;
  }

  bool doTestMovePtp(const double q_change)
  {
    bool return_status = true;
    rw::trajectory::QPath path = getQPath(q_change);
    for (const rw::math::Q& p : path)
    {
      ROS_INFO_STREAM("Ask to movePtp to '" << p << "'.");
      bool ret = false;
      ret = sdsip_.movePtp(p);
      if (!ret)
      {
        return_status = false;
        ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
      }
    }

    return return_status;
  }

  bool doTestMoveServoQ(const double q_change)
  {
    bool return_status = true;
    rw::trajectory::QPath path = getQPath(q_change);

    ROS_ASSERT(path.size() == 2);
    rw::math::Q start_configuration = path.at(0);
    rw::math::Q end_configuration = path.at(1);

    // replace the path with an interpolated path
    path = linearInterpolatedPath(start_configuration, end_configuration);

    for (const rw::math::Q& p : path)
    {
      ROS_INFO_STREAM("Ask to moveServoQ to '" << p << "'.");
      bool ret = false;
      ret = sdsip_.moveServoQ(p);
      if (!ret)
      {
        return_status = false;
        ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
      }
    }

    return return_status;
  }

 protected:
  ros::NodeHandle nodehandle_;
  caros::SerialDeviceSIProxy sdsip_;

  rw::models::WorkCell::Ptr workcell_;
  rw::models::Device::Ptr device_;
  rw::pathplanning::QToQPlanner::Ptr planner_;
};

#endif  // CAROS_TEST_UNIVERSAL_ROBOT_TEST_H
