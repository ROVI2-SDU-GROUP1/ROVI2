#ifndef CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_PROXY_H
#define CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_PROXY_H

#include <caros/service_interface_proxy_test_setup.h>

#include <caros/serial_device_si_proxy.h>
#include "serial_device_service_interface_dummy.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <vector>
#include <functional>
#include <tuple>
#include <string>

namespace
{
typedef std::vector<std::tuple<std::function<bool(caros::SerialDeviceSIProxy &)>, const std::string>> Services_t;

const Services_t services_to_test = {
    {std::make_tuple(
        std::bind(&caros::SerialDeviceSIProxy::moveLin, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
        "virtual bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t&)")},
    {std::make_tuple(std::bind(&caros::SerialDeviceSIProxy::movePtp, std::placeholders::_1, rw::math::Q(), 0, 0),
                     "virtual bool SerialDeviceServiceInterfaceDummy::movePtp(const QAndSpeedContainer_t&)")},
    {std::make_tuple(
        std::bind(&caros::SerialDeviceSIProxy::movePtpT, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
        "virtual bool SerialDeviceServiceInterfaceDummy::movePtpT(const TransformAndSpeedContainer_t&)")},
    {std::make_tuple(std::bind(&caros::SerialDeviceSIProxy::moveServoQ, std::placeholders::_1, rw::math::Q(), 0),
                     "virtual bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t&)")},
    {std::make_tuple(
        std::bind(&caros::SerialDeviceSIProxy::moveServoT, std::placeholders::_1, rw::math::Transform3D<>(), 0),
        "virtual bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t&)")},
    {std::make_tuple(std::bind(&caros::SerialDeviceSIProxy::moveVelQ, std::placeholders::_1, rw::math::Q()),
                     "virtual bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q&)")},
    {std::make_tuple(
        std::bind(&caros::SerialDeviceSIProxy::moveVelT, std::placeholders::_1, rw::math::VelocityScrew6D<>()),
        "virtual bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<double>&)")},
    {std::make_tuple(std::bind(&caros::SerialDeviceSIProxy::stop, std::placeholders::_1),
                     "virtual bool SerialDeviceServiceInterfaceDummy::moveStop()")}};

typedef SerialDeviceServiceInterfaceDummy D_t;
typedef caros::SerialDeviceSIProxy P_t;
}  // end namespace

#endif  // CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_PROXY_H
