#include "ForwardKinematics.hpp"

int main(int argc, char *argv[]) {
  ForwardKinematics FK(argc, argv);
  FK.spin();
  return 0;
}
