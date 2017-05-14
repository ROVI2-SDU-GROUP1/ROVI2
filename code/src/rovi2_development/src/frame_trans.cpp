#include "rwlibs/algorithms/PointPairsRegistration.hpp"
#include <fstream>

int main(){
  std::vector< rwlibs::algorithms::PointPairsRegistration::PointPair > pointPairs;


    std::ifstream  data("/home/mneerup/hte_back.csv");
    if(!data.is_open()){
        std::cout << "File not found!\n";
        return 1;
      }
    std::string line;
    while(std::getline(data,line)){
        std::stringstream  lineStream(line);
        double cam_x, cam_y, cam_z, robot_x, robot_y, robot_z;
        std::string cell;

        std::getline(lineStream,cell,',');
        cam_x = std::stod(cell)/1000;

        std::getline(lineStream,cell,',');
        cam_y = std::stod(cell)/1000;

        std::getline(lineStream,cell,',');
        cam_z = std::stod(cell)/1000;

        std::getline(lineStream,cell,',');
        robot_x = std::stod(cell);

        std::getline(lineStream,cell,',');
        robot_y = std::stod(cell);

        std::getline(lineStream,cell,',');
        robot_z = std::stod(cell);

        rw::math::Vector3D<> cam_vec(cam_x, cam_y, cam_z);
        rw::math::Vector3D<> robot_vec(robot_x, robot_y, robot_z);

        std::cout << robot_vec << std::endl;

        pointPairs.push_back(std::make_pair(robot_vec,cam_vec));
    }

  std::cout << "Num of pointPairs: " << pointPairs.size() << std::endl;
  rw::math::Transform3D<> Trans_camera_in_base =  rwlibs::algorithms::PointPairsRegistration::pointPairRegistrationSVD(pointPairs);


  //rw::math::Vector3D<> P1(-0.317222072079,-0.487509135996,-1.85257493885);
  rw::math::Vector3D<> P1(0.181131055917,0.37173324503,2.15828502449);
  std::cout << Trans_camera_in_base.P().norm2() << std::endl;

  auto P1_in_base = rw::math::inverse(Trans_camera_in_base)*P1;
  std::cout << "P1 in base: " << P1_in_base << std::endl;
  std::cout << "Distance from base to P1: " << P1_in_base.norm2() << std::endl;
  return 0;
}
