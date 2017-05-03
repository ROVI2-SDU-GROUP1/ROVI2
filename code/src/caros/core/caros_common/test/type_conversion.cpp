#include <caros/common.h>
#include <gtest/gtest.h>

/************************************************************************
 * Float
 ************************************************************************/
TEST(TypeConversion, toRosFromFloat)
{
  const float floats[] = {0.8147, 0.1354, 0.9057, 0.8350, 0.1269, 0.9688, 0.9133, 0.2210, 0.6323, 0.3081};

  for (const auto f : floats)
  {
    const auto to_ros = caros::toRos(f);
    EXPECT_DOUBLE_EQ(f, to_ros);
  }
}

/************************************************************************
 * Double
 ************************************************************************/
TEST(TypeConversion, toRosFromDouble)
{
  const double doubles[] = {0.0975404016208, 0.547220596345, 0.278498218395, 0.188381975982, 0.546881519025,
                            0.99288130179,   0.95750682964,  0.996461325325, 0.96488853381,  0.967694936786};

  for (const auto d : doubles)
  {
    const auto to_ros = caros::toRos(d);
    EXPECT_DOUBLE_EQ(d, to_ros);
  }
}

/************************************************************************
 * Bool
 ************************************************************************/
TEST(TypeConversion, toRosFromBool)
{
  const bool bools[] = {true, false};

  for (const auto d : bools)
  {
    const auto to_ros = caros::toRos(d);
    EXPECT_EQ(d, to_ros);
  }
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
