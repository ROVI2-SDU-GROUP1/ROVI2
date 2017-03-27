#ifndef IMG_CONVERTER_HPP
#define IMG_CONVERTER_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class ImageConverter{
    public:
        ImageConverter(std::string subcription, uint _type);
        ~ImageConverter();

        cv::Mat get_image();
        void imageCb(const sensor_msgs::ImageConstPtr& msg);

    private:
        cv::Mat image;
        uint type;

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
};

#endif
