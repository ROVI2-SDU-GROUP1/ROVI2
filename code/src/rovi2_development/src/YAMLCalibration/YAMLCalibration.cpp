#include "YAMLCalibration.hpp"
YAMLCalibration::YAMLCalibration(std::string abs_filename) {
  if(!if_file_exist(abs_filename)){
    ROS_WARN("File: %s does not exist\n", abs_filename.c_str());
  } else {
    ROS_INFO("Loading YAML %s\n", abs_filename.c_str());
  }

  yaml_file = YAML::LoadFile(abs_filename);
}

bool YAMLCalibration::if_file_exist(std::string filename){
    std::ifstream infile(filename.c_str());
    return infile.good();
}

cv::Mat YAMLCalibration::get_camera_matrix(){
  YAML::Node camera_matrix_node = yaml_file["camera_matrix"];

  cv::Mat cameraMatrix = cv::Mat(3,3, CV_64FC1);
  for(unsigned int i = 0; i < 3; i++){
    for(unsigned int j = 0; j < 3; j++){
      cameraMatrix.at<double>(i,j) = camera_matrix_node["data"][i*3+j].as<double>();
    }
  }
  return cameraMatrix;
}

cv::Mat YAMLCalibration::get_distortion_coeffs(){
  YAML::Node distortion_coefficients_node = yaml_file["distortion_coefficients"];

  cv::Mat distCoeffs = cv::Mat(1,distortion_coefficients_node["data"].size(), CV_64FC1);
  for(unsigned int i = 0; i < distortion_coefficients_node["data"].size();i++){
    distCoeffs.at<double>(i) = distortion_coefficients_node["data"][i].as<double>();
  }
  return distCoeffs;
}

cv::Mat YAMLCalibration::get_rect_matrix(){
  YAML::Node rectification_matrix_node = yaml_file["rectification_matrix"];

  cv::Mat rectMatrix = cv::Mat(3,3, CV_64FC1);
  for(unsigned int i = 0; i < 3; i++){
    for(unsigned int j = 0; j < 3; j++){
      rectMatrix.at<double>(i,j) = rectification_matrix_node["data"][i*3+j].as<double>();
    }
  }
  return rectMatrix;
}

cv::Mat YAMLCalibration::get_translation_vector(){
  YAML::Node translation_vector_node = yaml_file["translation_vector"];

  cv::Mat translationVector = cv::Mat(3,1, CV_64FC1);
  for(unsigned int i = 0; i < translation_vector_node.size(); i++){
    translationVector.at<double>(i) = translation_vector_node["data"][i].as<double>();
  }
  return translationVector;
}

cv::Mat YAMLCalibration::get_rotation_matrix(){
  YAML::Node rotation_matrix_node = yaml_file["rotation_matrix"];

  cv::Mat rotationMatrix = cv::Mat(3,3, CV_64FC1);
  for(unsigned int y = 0; y < 3; y++){
    for(unsigned int x = 0; x < 3; x++){
      rotationMatrix.at<double>(y,x) = rotation_matrix_node["data"][y*3+x].as<double>();
    }
  }
  return rotationMatrix;
}

int YAMLCalibration::get_height(){
  return yaml_file["image_height"].as<int>();
}

int YAMLCalibration::get_width(){
  return yaml_file["image_width"].as<int>();;
}
