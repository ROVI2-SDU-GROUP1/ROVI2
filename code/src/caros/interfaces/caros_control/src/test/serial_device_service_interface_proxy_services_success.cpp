#include <caros/test/serial_device_service_interface_proxy.h>

TEST(SerialDeviceSIProxy, servicesSuccess)
{
  caros::test::testServices<D_t, P_t, Services_t>(services_to_test, caros::test::TestType::ReturnTrue);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_device_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
