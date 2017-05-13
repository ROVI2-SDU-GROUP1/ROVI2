#include "yaml-cpp/yaml.h"
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>


class YAMLCalibration {
public:
    YAMLCalibration(std::string filename);
    cv::Mat get_camera_matrix();
    cv::Mat get_distortion_coeffs();
    cv::Mat get_rect_matrix();
    cv::Mat get_translation_vector();
    cv::Mat get_rotation_matrix();
    int get_height();
    int get_width();
  private:
    bool if_file_exist(std::string filname);
    YAML::Node yaml_file;
};
