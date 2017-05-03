#include <caros/test/gripper_service_interface_proxy.h>

TEST(GripperSIProxy, badServiceCall)
{
  caros::test::testServices<D_t, P_t, Services_t>(services_to_test, caros::test::TestType::BadServiceCall);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
