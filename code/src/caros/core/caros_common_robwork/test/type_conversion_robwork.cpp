#include <caros/common_robwork.h>
#include <gtest/gtest.h>

#include <rw/loaders/WorkCellFactory.hpp>

namespace
{
void testQAbsoluteDifference(const rw::math::Q& value1, const rw::math::Q& value2, const rw::math::Q& difference)
{
  // Make sure all the Q's have the same size
  ASSERT_EQ(value1.size(), value2.size());
  ASSERT_EQ(value1.size(), difference.size());

  for (std::size_t index = 0; index < value1.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(std::fabs(value1[index] - value2[index]), difference[index]);
  }
}

void testQEqual(const rw::math::Q& value1, const rw::math::Q& value2)
{
  // Make sure all the Q's have the same size
  ASSERT_EQ(value1.size(), value2.size());

  for (std::size_t index = 0; index < value1.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(value1[index], value2[index]);
  }
}
}  // namespace

/************************************************************************
 * State
 ************************************************************************/
TEST(TypeConversion, rw_state)
{
  rw::models::WorkCell::Ptr wc;
  ASSERT_TRUE(wc == 0);
  EXPECT_NO_THROW(
      {
        wc = rw::loaders::WorkCellFactory::load("data/workcell.xml");
      });
  ASSERT_TRUE(wc != 0);
  const auto device = wc->findDevice("UR1");
  ASSERT_TRUE(device != 0);

  const auto default_state = wc->getDefaultState();
  auto new_state = default_state;

  rw::math::Q q_change(device->getQ(new_state).size(), 1.3);
  for (std::size_t index = 0; index < q_change.size(); ++index)
  {
    q_change[index] += index;
  }
  device->setQ(device->getQ(new_state) + q_change, new_state);

  // Verify that the states give different Q's
  testQAbsoluteDifference(device->getQ(new_state), device->getQ(default_state), q_change);

  const auto to_ros = caros::toRos(new_state);
  // Test the function taking a workcell ptr
  const auto state_from_ros = caros::toRw(to_ros, wc);
  testQEqual(device->getQ(new_state), device->getQ(state_from_ros));
  testQAbsoluteDifference(device->getQ(state_from_ros), device->getQ(default_state), q_change);

  // Test the function taking a RobWork state reference
  auto modified_state = wc->getDefaultState();
  // Verify that the modified state is similar to default_state
  testQEqual(device->getQ(default_state), device->getQ(modified_state));
  caros::toRw(to_ros, modified_state);
  testQEqual(device->getQ(new_state), device->getQ(modified_state));
  testQAbsoluteDifference(device->getQ(default_state), device->getQ(modified_state), q_change);
}

/************************************************************************
 * Q
 ************************************************************************/
TEST(TypeConversion, toRosFromRwQ)
{
  const rw::math::Q rw_q(6, 3.0);
  const auto to_ros_q = caros::toRos(rw_q);
  ASSERT_EQ(to_ros_q.data.size(), rw_q.size());
  for (std::size_t index = 0; index < rw_q.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(to_ros_q.data.at(index), rw_q[index]);
  }

  /* Forth and back */
  const auto to_ros_and_back = caros::toRw(caros::toRos(rw_q));
  // Note: Currently the comparison operator implementation is not doing proper floating point comparisons
  EXPECT_TRUE(rw_q == to_ros_and_back);
  testQEqual(rw_q, to_ros_and_back);
}

TEST(TypeConversion, toRwFromRosQ)
{
  // Create ros Q and fill it with data
  caros_common_msgs::Q ros_q;
  ros_q.data.resize(4);
  for (std::size_t index = 0; index < ros_q.data.size(); ++index)
  {
    ros_q.data.at(index) = static_cast<double>(1.0 + index);
  }

  // Convert to robwork Q
  const auto to_rw_q = caros::toRw(ros_q);
  ASSERT_EQ(to_rw_q.size(), ros_q.data.size());
  for (std::size_t index = 0; index < ros_q.data.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(to_rw_q[index], ros_q.data.at(index));
  }

  /* Forth and back */
  const auto to_rw_and_back = caros::toRos(caros::toRw(ros_q));
  ASSERT_EQ(ros_q.data.size(), to_rw_and_back.data.size());
  for (std::size_t index = 0; index < ros_q.data.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(ros_q.data.at(index), to_rw_and_back.data.at(index));
  }
}

/************************************************************************
 * Transform
 ************************************************************************/
TEST(TypeConversion, to_rosFromRwTransform3D)
{
  // The comparison_precision value is found by trial and error.
  const double comparison_precision = 1.0e-05;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> transforms[] = {
      {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791),
       rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702,
                              -0.748207)},
      {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179),
       rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419,
                              -0.626255)},
      {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656),
       rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328,
                              -0.0248621)},
      {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235),
       rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239,
                              -0.44795)},
      {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964),
       rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181,
                              0.421408)}};

  for (const auto& transform : transforms)
  {
    const auto to_ros = caros::toRos(transform);
    const rw::math::Vector3D<> translation(to_ros.translation.x, to_ros.translation.y, to_ros.translation.z);
    const rw::math::Quaternion<> quaternion(to_ros.rotation.x, to_ros.rotation.y, to_ros.rotation.z, to_ros.rotation.w);
    const rw::math::Transform3D<> reconstructed_transform(translation, quaternion.toRotation3D());

    EXPECT_TRUE(transform.equal(reconstructed_transform, comparison_precision));

    /* Forth and back */
    const auto to_ros_and_back = caros::toRw(caros::toRos(transform));
    EXPECT_TRUE(transform.equal(to_ros_and_back, comparison_precision));
  }
}

TEST(TypeConversion, toRwFromRosTransform)
{
  // The comparison_precision value is found by trial and error.
  const double comparison_precision = 1.0e-04;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> rw_transforms[] = {
      {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791),
       rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702,
                              -0.748207)},
      {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179),
       rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419,
                              -0.626255)},
      {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656),
       rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328,
                              -0.0248621)},
      {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235),
       rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239,
                              -0.44795)},
      {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964),
       rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181,
                              0.421408)}};

  for (const auto& rw_transform : rw_transforms)
  {
    geometry_msgs::Transform transform;
    transform.translation.x = rw_transform.P()[0];
    transform.translation.y = rw_transform.P()[1];
    transform.translation.z = rw_transform.P()[2];
    const rw::math::Quaternion<> rw_quaternion(rw_transform.R());
    transform.rotation.x = rw_quaternion.getQx();
    transform.rotation.y = rw_quaternion.getQy();
    transform.rotation.z = rw_quaternion.getQz();
    transform.rotation.w = rw_quaternion.getQw();

    const auto to_rw = caros::toRw(transform);
    EXPECT_TRUE(rw_transform.equal(to_rw, comparison_precision));

    /* Forth and back */
    const auto to_rw_and_back = caros::toRos(caros::toRw(transform));
    EXPECT_NEAR(transform.translation.x, to_rw_and_back.translation.x, comparison_precision);
    EXPECT_NEAR(transform.translation.y, to_rw_and_back.translation.y, comparison_precision);
    EXPECT_NEAR(transform.translation.z, to_rw_and_back.translation.z, comparison_precision);
    EXPECT_NEAR(transform.rotation.x, to_rw_and_back.rotation.x, comparison_precision);
    EXPECT_NEAR(transform.rotation.y, to_rw_and_back.rotation.y, comparison_precision);
    EXPECT_NEAR(transform.rotation.z, to_rw_and_back.rotation.z, comparison_precision);
    EXPECT_NEAR(transform.rotation.w, to_rw_and_back.rotation.w, comparison_precision);
  }
}

/************************************************************************
 * Pose
 ************************************************************************/
TEST(TypeConversion, toRosPoseFromRwTransform3D)
{
  /********************************
   * Notes:
   * Basically copy/paste from toRos_from_rw_transform3D with the following replacements
   * caros::toRos -> caros::toRosPose
   * to_ros.translation -> to_ros.position
   * to_ros.rotation -> to_ros.orientation
   ********************************/
  // The comparison_precision value is found by trial and error.
  const double comparison_precision = 1.0e-05;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> transforms[] = {
      {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791),
       rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702,
                              -0.748207)},
      {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179),
       rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419,
                              -0.626255)},
      {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656),
       rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328,
                              -0.0248621)},
      {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235),
       rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239,
                              -0.44795)},
      {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964),
       rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181,
                              0.421408)}};

  for (const auto& transform : transforms)
  {
    const auto to_ros = caros::toRosPose(transform);
    const rw::math::Vector3D<> position(to_ros.position.x, to_ros.position.y, to_ros.position.z);
    const rw::math::Quaternion<> quaternion(to_ros.orientation.x, to_ros.orientation.y, to_ros.orientation.z,
                                            to_ros.orientation.w);
    const rw::math::Transform3D<> reconstructed_transform(position, quaternion.toRotation3D());

    EXPECT_TRUE(transform.equal(reconstructed_transform, comparison_precision));

    /* Forth and back */
    const auto to_ros_pose_and_back = caros::toRw(caros::toRosPose(transform));
    EXPECT_TRUE(transform.equal(to_ros_pose_and_back, comparison_precision));
  }
}

TEST(TypeConversion, toRwFromRosPose)
{
  /********************************
   * Notes:
   * Basically copy/paste from toRw_from_ros_transform with a bit more extensive changes.
   ********************************/
  // The comparison_precision value is found by trial and error.
  const double comparison_precision = 1.0e-04;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> rw_transforms[] = {
      {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791),
       rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702,
                              -0.748207)},
      {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179),
       rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419,
                              -0.626255)},
      {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656),
       rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328,
                              -0.0248621)},
      {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235),
       rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239,
                              -0.44795)},
      {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964),
       rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181,
                              0.421408)}};

  for (const auto& rw_transform : rw_transforms)
  {
    geometry_msgs::Pose pose;
    pose.position.x = rw_transform.P()[0];
    pose.position.y = rw_transform.P()[1];
    pose.position.z = rw_transform.P()[2];
    const rw::math::Quaternion<> rw_quaternion(rw_transform.R());
    pose.orientation.x = rw_quaternion.getQx();
    pose.orientation.y = rw_quaternion.getQy();
    pose.orientation.z = rw_quaternion.getQz();
    pose.orientation.w = rw_quaternion.getQw();

    const auto to_rw = caros::toRw(pose);
    EXPECT_TRUE(rw_transform.equal(to_rw, comparison_precision));

    /* Forth and back */
    const auto to_rw_and_back = caros::toRosPose(caros::toRw(pose));
    EXPECT_NEAR(pose.position.x, to_rw_and_back.position.x, comparison_precision);
    EXPECT_NEAR(pose.position.y, to_rw_and_back.position.y, comparison_precision);
    EXPECT_NEAR(pose.position.z, to_rw_and_back.position.z, comparison_precision);
    EXPECT_NEAR(pose.orientation.x, to_rw_and_back.orientation.x, comparison_precision);
    EXPECT_NEAR(pose.orientation.y, to_rw_and_back.orientation.y, comparison_precision);
    EXPECT_NEAR(pose.orientation.z, to_rw_and_back.orientation.z, comparison_precision);
    EXPECT_NEAR(pose.orientation.w, to_rw_and_back.orientation.w, comparison_precision);
  }
}

/************************************************************************
 * Wrench
 ************************************************************************/
TEST(TypeConversion, toRosFromRwWrench6D)
{
  const rw::math::Wrench6D<> wrenches[] = {
      {0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095, 0.913375855656},
      {0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345, 0.278498218395},
      {0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381},
      {0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551, 0.957166949753},
      {0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372, 0.141886345111}};

  for (const auto& wrench : wrenches)
  {
    const auto to_ros = caros::toRos(wrench);
    EXPECT_DOUBLE_EQ(wrench(0), to_ros.force.x);
    EXPECT_DOUBLE_EQ(wrench(1), to_ros.force.y);
    EXPECT_DOUBLE_EQ(wrench(2), to_ros.force.z);
    EXPECT_DOUBLE_EQ(wrench(3), to_ros.torque.x);
    EXPECT_DOUBLE_EQ(wrench(4), to_ros.torque.y);
    EXPECT_DOUBLE_EQ(wrench(5), to_ros.torque.z);

    /* Forth and back */
    const auto to_ros_and_back = caros::toRw(caros::toRos(wrench));
    EXPECT_DOUBLE_EQ(wrench(0), to_ros_and_back(0));
    EXPECT_DOUBLE_EQ(wrench(1), to_ros_and_back(1));
    EXPECT_DOUBLE_EQ(wrench(2), to_ros_and_back(2));
    EXPECT_DOUBLE_EQ(wrench(3), to_ros_and_back(3));
    EXPECT_DOUBLE_EQ(wrench(4), to_ros_and_back(4));
    EXPECT_DOUBLE_EQ(wrench(5), to_ros_and_back(5));
  }
}

TEST(TypeConversion, toRwFromRosWrench)
{
  const double wrench_values[][6] = {
      {0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095, 0.913375855656},
      {0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345, 0.278498218395},
      {0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381},
      {0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551, 0.957166949753},
      {0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372, 0.141886345111}};

  for (const auto& wrench_value : wrench_values)
  {
    geometry_msgs::Wrench wrench;
    wrench.force.x = wrench_value[0];
    wrench.force.y = wrench_value[1];
    wrench.force.z = wrench_value[2];
    wrench.torque.x = wrench_value[3];
    wrench.torque.y = wrench_value[4];
    wrench.torque.z = wrench_value[5];

    const auto to_rw = caros::toRw(wrench);
    EXPECT_DOUBLE_EQ(wrench.force.x, to_rw(0));
    EXPECT_DOUBLE_EQ(wrench.force.y, to_rw(1));
    EXPECT_DOUBLE_EQ(wrench.force.z, to_rw(2));
    EXPECT_DOUBLE_EQ(wrench.torque.x, to_rw(3));
    EXPECT_DOUBLE_EQ(wrench.torque.y, to_rw(4));
    EXPECT_DOUBLE_EQ(wrench.torque.z, to_rw(5));

    /* Forth and back */
    const auto to_rw_and_back = caros::toRos(caros::toRw(wrench));
    EXPECT_DOUBLE_EQ(wrench.force.x, to_rw_and_back.force.x);
    EXPECT_DOUBLE_EQ(wrench.force.y, to_rw_and_back.force.y);
    EXPECT_DOUBLE_EQ(wrench.force.z, to_rw_and_back.force.z);
    EXPECT_DOUBLE_EQ(wrench.torque.x, to_rw_and_back.torque.x);
    EXPECT_DOUBLE_EQ(wrench.torque.y, to_rw_and_back.torque.y);
    EXPECT_DOUBLE_EQ(wrench.torque.z, to_rw_and_back.torque.z);
  }
}

/************************************************************************
 * Twist / VelocityScrew
 ************************************************************************/
TEST(TypeConversion, toRosFromRwVelocityScrew6D)
{
  const rw::math::VelocityScrew6D<> velocity_screws[] = {
      {0.814723691903, 0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095},
      {0.913375855656, 0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345},
      {0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325},
      {0.96488853381, 0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551},
      {0.957166949753, 0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372}};

  for (const auto& velocity_screw : velocity_screws)
  {
    const auto to_ros = caros::toRos(velocity_screw);
    EXPECT_DOUBLE_EQ(velocity_screw(0), to_ros.linear.x);
    EXPECT_DOUBLE_EQ(velocity_screw(1), to_ros.linear.y);
    EXPECT_DOUBLE_EQ(velocity_screw(2), to_ros.linear.z);
    EXPECT_DOUBLE_EQ(velocity_screw(3), to_ros.angular.x);
    EXPECT_DOUBLE_EQ(velocity_screw(4), to_ros.angular.y);
    EXPECT_DOUBLE_EQ(velocity_screw(5), to_ros.angular.z);

    /* Forth and back */
    const auto to_ros_and_back = caros::toRw(caros::toRos(velocity_screw));
    EXPECT_DOUBLE_EQ(velocity_screw(0), to_ros_and_back(0));
    EXPECT_DOUBLE_EQ(velocity_screw(1), to_ros_and_back(1));
    EXPECT_DOUBLE_EQ(velocity_screw(2), to_ros_and_back(2));
    EXPECT_DOUBLE_EQ(velocity_screw(3), to_ros_and_back(3));
    EXPECT_DOUBLE_EQ(velocity_screw(4), to_ros_and_back(4));
    EXPECT_DOUBLE_EQ(velocity_screw(5), to_ros_and_back(5));
  }
}

TEST(TypeConversion, toRwFromRosTwist)
{
  const double velocity_screw_values[][6] = {
      {0.814723691903, 0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095},
      {0.913375855656, 0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345},
      {0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325},
      {0.96488853381, 0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551},
      {0.957166949753, 0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372}};

  for (const auto& velocity_screw_value : velocity_screw_values)
  {
    geometry_msgs::Twist twist;
    twist.linear.x = velocity_screw_value[0];
    twist.linear.y = velocity_screw_value[1];
    twist.linear.z = velocity_screw_value[2];
    twist.angular.x = velocity_screw_value[3];
    twist.angular.y = velocity_screw_value[4];
    twist.angular.z = velocity_screw_value[5];

    const auto to_rw = caros::toRw(twist);
    EXPECT_DOUBLE_EQ(twist.linear.x, to_rw(0));
    EXPECT_DOUBLE_EQ(twist.linear.y, to_rw(1));
    EXPECT_DOUBLE_EQ(twist.linear.z, to_rw(2));
    EXPECT_DOUBLE_EQ(twist.angular.x, to_rw(3));
    EXPECT_DOUBLE_EQ(twist.angular.y, to_rw(4));
    EXPECT_DOUBLE_EQ(twist.angular.z, to_rw(5));

    /* Forth and back */
    const auto to_rw_and_back = caros::toRos(caros::toRw(twist));
    EXPECT_DOUBLE_EQ(twist.linear.x, to_rw_and_back.linear.x);
    EXPECT_DOUBLE_EQ(twist.linear.y, to_rw_and_back.linear.y);
    EXPECT_DOUBLE_EQ(twist.linear.z, to_rw_and_back.linear.z);
    EXPECT_DOUBLE_EQ(twist.angular.x, to_rw_and_back.angular.x);
    EXPECT_DOUBLE_EQ(twist.angular.y, to_rw_and_back.angular.y);
    EXPECT_DOUBLE_EQ(twist.angular.z, to_rw_and_back.angular.z);
  }
}

/************************************************************************
 * Float
 ************************************************************************/
TEST(TypeConversion, toRwFromFloat)
{
  const float floats[] = {0.8147, 0.1354, 0.9057, 0.8350, 0.1269, 0.9688, 0.9133, 0.2210, 0.6323, 0.3081};

  for (const auto f : floats)
  {
    const auto to_rw = caros::toRw(f);
    EXPECT_DOUBLE_EQ(f, to_rw);
  }
}

/************************************************************************
 * Double
 ************************************************************************/
TEST(TypeConversion, toRwFromDouble)
{
  const double doubles[] = {0.0975404016208, 0.547220596345, 0.278498218395, 0.188381975982, 0.546881519025,
                            0.99288130179,   0.95750682964,  0.996461325325, 0.96488853381,  0.967694936786};

  for (const auto d : doubles)
  {
    const auto to_rw = caros::toRw(d);
    EXPECT_DOUBLE_EQ(d, to_rw);
  }
}

/************************************************************************
 * Bool
 ************************************************************************/
TEST(TypeConversion, toRwFromBool)
{
  const bool bools[] = {true, false};

  for (const auto d : bools)
  {
    const auto to_rw = caros::toRw(d);
    EXPECT_EQ(d, to_rw);
  }
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
