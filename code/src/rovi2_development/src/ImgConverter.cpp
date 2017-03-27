#include "../include/ImgConverter.hpp"

ImageConverter::ImageConverter(std::string subcription, uint _type) : it_(nh_){
  this->type = _type;
  image_sub_ = it_.subscribe(subcription, 1, &ImageConverter::imageCb, this);
}

ImageConverter::~ImageConverter(){

}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    switch(this->type) {
    case 0:
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      break;
    case 1:
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      break;
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  image = cv_ptr->image.clone();
}

cv::Mat ImageConverter::get_image(){
  return image.clone();
}
