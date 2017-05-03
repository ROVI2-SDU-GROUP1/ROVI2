#ifndef CAROS_TEST_GRIPPER_SERVICE_INTERFACE_PROXY_H
#define CAROS_TEST_GRIPPER_SERVICE_INTERFACE_PROXY_H

#include <caros/service_interface_proxy_test_setup.h>

#include <caros/gripper_si_proxy.h>
#include "gripper_service_interface_dummy.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <list>
#include <functional>
#include <tuple>
#include <string>

namespace
{
// Container type to hold the services that should be tested
typedef std::list<std::tuple<std::function<bool(caros::GripperSIProxy &)>, const std::string>> Services_t;

const Services_t services_to_test = {
    {std::make_tuple(std::bind(&caros::GripperSIProxy::moveQ, std::placeholders::_1, rw::math::Q()),
                     "virtual bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q&)")},
    {std::make_tuple(std::bind(&caros::GripperSIProxy::gripQ, std::placeholders::_1, rw::math::Q()),
                     "virtual bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q&)")},
    {std::make_tuple(std::bind(&caros::GripperSIProxy::setForceQ, std::placeholders::_1, rw::math::Q()),
                     "virtual bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q&)")},
    {std::make_tuple(std::bind(&caros::GripperSIProxy::setVelocityQ, std::placeholders::_1, rw::math::Q()),
                     "virtual bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q&)")},
    {std::make_tuple(std::bind(&caros::GripperSIProxy::stopMovement, std::placeholders::_1),
                     "virtual bool GripperServiceInterfaceDummy::stopMovement()")}};

typedef GripperServiceInterfaceDummy D_t;
typedef caros::GripperSIProxy P_t;
}  // end namespace

#endif  // CAROS_TEST_GRIPPER_SERVICE_INTERFACE_PROXY_H
