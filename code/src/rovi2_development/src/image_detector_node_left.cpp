#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cassert>

int mx = 0;
int my = 0;

ros::Subscriber image_sub;
ros::Publisher point_pub;

void closing(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_dilate = 10;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(src, dst, elementDilate);

  int erosion_size_erode = 7;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(dst, src, elementErode);
}

void opening(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_erode = 7;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(src, dst, elementErode);

  int erosion_size_dilate = 10;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(dst, src, elementDilate);
}

int constrain(int x, int min, int max){
  if (x < min) return min;
  if (max < x) return max;
  return x;
}

int abs(int x){
  if (x < 0) return -x;
  return x;
}

// Adaptive filter
void adaptive(cv::Mat &source, cv::Mat &dist, cv::Scalar color, int threshold){
  dist.create(source.rows, source.cols, CV_8UC1);
  for (int i = 0; i < source.rows; i++){
    for (int j = 0; j < source.cols; j++){
      for (int k = 0; k < 3; k++){
        int temp = source.at<cv::Vec3b>(i, j)[k] - color[k];
        dist.at<char>(i, j) = constrain(abs(temp), 0, 255);
      }
    }
  }
  /*int halfMask = (mask.cols - 1) / 2;
  for (int y = halfMask; y < source.rows - halfMask; y++){
    for (int x = halfMask; x < source.cols - halfMask; x++){
      int temp = 0;
      for (int i = 0; i <= halfMask * 2; i++){
        for (int j = 0; j <= halfMask * 2; j++){
          for (int c = 0; c < 3; c++){
            temp += mask.at<cv::Vec3b>(i, j)[c] * (color[c] - source.at<cv::Vec3b>(y + i - halfMask, x + j - halfMask)[c]);
          }
        }
      }
      if (temp / 256 < threshold) dist.at<char>(y, x) = 255;
      else dist.at<char>(y, x) = 0;
    }
  }*/
}

// state subscriber
void find2DPose(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat imageBGR = cv_ptr->image.clone();
  cv::Mat imageHSV;
  cv::cvtColor(imageBGR, imageHSV, CV_BGR2HSV);


/*
  cv::Mat threshold;
  adaptive(imageHSV, threshold, cv::Scalar(0, 255, 255), 10);


  std::cout << imageHSV.at<cv::Vec3b>(my, mx) << std::endl;

  */
  cv::Mat thresholdLow;
  cv::Mat thresholdHigh;
  cv::inRange(imageHSV, cv::Scalar(0, 220, 50), cv::Scalar(10, 255, 255), thresholdLow);
  cv::inRange(imageHSV, cv::Scalar(215, 220, 50), cv::Scalar(255, 255, 255), thresholdHigh);

  cv::Mat threshold = thresholdLow + thresholdHigh;

  opening(threshold);
  closing(threshold);


	std::vector<std::vector<cv::Point> > cont;
	std::vector<cv::Vec4i> hier;
	cv::findContours(threshold, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  geometry_msgs::PointStamped point;

  if (!cont.size()){
    //cv::imshow("t2", imageBGR);
    //cv::waitKey(1);
    point_pub.publish(point);
    return;
  }

	cv::Moments moments = cv::moments(cont[0], false);
	cv::Point2f imgCoord = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

  point.point.x = imgCoord.x;
  point.point.y = imgCoord.y;
  point.header.stamp = msg->header.stamp;

  cv::circle(imageBGR, imgCoord, 10, cv::Scalar(0, 255, 0), 2);

/*
  cv::imshow("t2", imageBGR);
  cv::waitKey(1);
  cv::imshow("t1", threshold);
  cv::waitKey(1);
*/

  //cv::setMouseCallback("t2", mouseCallback, NULL);


  //
  point_pub.publish(point);
}

int main(int argc, char **argv){

  std::string image_sub_name;
  std::string point_pub_name;

  ros::init(argc, argv, "image_detection_left");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);

  nh.param<std::string>("image_sub", image_sub_name, "/camera/left/image_raw");
  nh.param<std::string>("point_pub", point_pub_name, "/pose/2d_left");

  image_sub = nh.subscribe<sensor_msgs::Image>(image_sub_name, 1, find2DPose);
  point_pub = nh.advertise<geometry_msgs::PointStamped>(point_pub_name, 1);

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
