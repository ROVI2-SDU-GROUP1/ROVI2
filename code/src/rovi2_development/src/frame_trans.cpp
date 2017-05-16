#include "rwlibs/algorithms/PointPairsRegistration.hpp"
#include "rw/math/RPY.hpp"
#include "rw/math/Constants.hpp"

#include "YAMLCalibration/YAMLCalibration.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

rw::math::Transform3D<double> get_cam_to_base(std::string filename){
    YAMLCalibration hte_calibration( std::string(CALIBRATION_DIR) + filename);
    cv::Mat translation_vector = hte_calibration.get_translation_vector();
    cv::Mat rotation_matrix = hte_calibration.get_rotation_matrix();
    rw::math::Vector3D<double> trans(   translation_vector.at<double>(0,0),
                                        translation_vector.at<double>(1,0),
                                        translation_vector.at<double>(2,0));

    rw::math::Rotation3D<double> rot( rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
                                      rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
                                      rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));

    rw::math::Transform3D<double> cam_to_base(trans, rot);
    return cam_to_base;
}

int main(int argc, char **argv){

    std::cout <<  "loading from " << argv[1] << std::endl;
    auto trans =  get_cam_to_base(argv[1]);


    std::cout << "ROT: " << rw::math::RPY<>(trans.R())[0]*rw::math::Rad2Deg << " " << rw::math::RPY<>(trans.R())[1]*rw::math::Rad2Deg << " " << rw::math::RPY<>(trans.R())[2]*rw::math::Rad2Deg << std::endl;
    std::cout << trans.P() << std::endl;
    std::cout << "norm2: " << trans.P().norm2() << std::endl;

    std::cout << "in:  " << trans.R() << std::endl;
    auto rpy = rw::math::RPY<>(trans.R());

    std::cout << "out: " << rpy.toRotation3D() << std::endl;

    return  0;
    std::vector< rwlibs::algorithms::PointPairsRegistration::PointPair > pointPairs;
    std::cout <<  "loading from " << argv[1] << std::endl;
    std::ifstream  data(argv[1]);
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
        cam_x = std::stod(cell);

        std::getline(lineStream,cell,',');
        cam_y = std::stod(cell);

        std::getline(lineStream,cell,',');
        cam_z = std::stod(cell);

        std::getline(lineStream,cell,',');
        robot_x = std::stod(cell);

        std::getline(lineStream,cell,',');
        robot_y = std::stod(cell);

        std::getline(lineStream,cell,',');
        robot_z = std::stod(cell);

        rw::math::Vector3D<> cam_vec(cam_x, cam_y, cam_z);
        rw::math::Vector3D<> robot_vec(robot_x, robot_y, robot_z);

        std::cout << cam_vec << std::endl;
        std::cout << robot_vec << std::endl << std::endl;

        pointPairs.push_back(std::make_pair(cam_vec,robot_vec));
    }

  std::cout << "Num of pointPairs: " << pointPairs.size() << std::endl;
  rw::math::Transform3D<> Trans_camera_in_base = rwlibs::algorithms::PointPairsRegistration::pointPairRegistrationSVD(pointPairs);

  std::cout << Trans_camera_in_base << std::endl;
  // Write transformation to file
  YAMLCalibration::WriteToYaml(std::string(CALIBRATION_DIR) + "hte.yaml", Trans_camera_in_base);

  // Read transformation from file
  YAMLCalibration hte_calibration(std::string(CALIBRATION_DIR) + "hte.yaml");
  cv::Mat translation_vector = hte_calibration.get_translation_vector();
  cv::Mat rotation_matrix = hte_calibration.get_rotation_matrix();





  // output
  // std::cout << translation_vector << std::endl;
  // std::cout << rotation_matrix << std::endl;


  rw::math::Vector3D<> P1(0.181131055917,0.37173324503,2.15828502449); // <- right side of base-plate
  rw::math::Vector3D<> P2(-0.111626661713,0.366865480386,2.147308153); // <- left side of base-plate
  std::cout<< "Distance from camera to base: " << Trans_camera_in_base.P().norm2() << std::endl;

  auto P1_in_base = Trans_camera_in_base*P1;
  auto P2_in_base = Trans_camera_in_base*P2;

  std::cout << "P1 in camera: " <<  P1 << std::endl;
  std::cout << "Distance from camera to P1: " << P1.norm2() << std::endl;
  std::cout << "P1 in base: " << P1_in_base << std::endl;
  std::cout << "Distance from base to P1: " << P1_in_base.norm2() << std::endl;

  std::cout << "Distance between P1 and P2 in camera frame:" << (P1-P2).norm2() << std::endl;
  std::cout << "Distance between P1 and P2 in base frame:" << (P1_in_base-P2_in_base).norm2() << std::endl;

  return 0;
}
