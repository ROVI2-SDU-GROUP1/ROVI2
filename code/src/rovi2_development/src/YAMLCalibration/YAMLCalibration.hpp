#include "yaml-cpp/yaml.h"
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include "rw/math/Transform3D.hpp"

static YAML::Emitter& operator << (YAML::Emitter& out, rw::math::Vector3D<double> & v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.e()(0) << v.e()(1)<< v.e()(2) << YAML::EndSeq;
    return out;
}

static YAML::Emitter& operator << (YAML::Emitter& out, rw::math::Rotation3D<double> & v) {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    std::vector<double> tmp;
    for(int col = 0; col < 3; col++){
      for(int row = 0; row < 3; row++){
          out << v.e()(row,col);
      }
    }
    out << YAML::EndSeq;
    return out;
}


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

    static void WriteToYaml(std::string filename, rw::math::Transform3D<double> & v){
        YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "rotation_matrix";
      out << YAML::BeginMap;
        out << YAML::Key << "rows";
        out << YAML::Value << "3";
        out << YAML::Key << "cols";
        out << YAML::Value << "3";
        out << YAML::Key << "data";
        out << YAML::Value << v.R();
      out << YAML::EndMap;

        out << YAML::Key << "translation_vector";
        out << YAML::BeginMap;
          out << YAML::Key << "rows";
          out << YAML::Value << "3";
          out << YAML::Key << "cols";
          out << YAML::Value << "1";
          out << YAML::Key << "data";
          out << YAML::Value << v.P();
        out << YAML::EndMap;

        out << YAML::EndMap;

        std::ofstream myfile;
        myfile.open (filename.c_str());
        myfile << out.c_str();
        myfile.close();
        return ;
      }
  private:
    bool if_file_exist(std::string filname);
    YAML::Node yaml_file;
};
