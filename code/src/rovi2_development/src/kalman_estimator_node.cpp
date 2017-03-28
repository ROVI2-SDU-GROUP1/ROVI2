#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Subscriber sub;
ros::Publisher pub;

std::vector<float> secondPoint;
std::vector<float> firstPoint;


void parameterEstimation(const geometry_msgs::PointStamped::ConstPtr &msg

  //  Do the parameter estimation // TODO

  pub.publish(parameters)

  secondPoint = firstPoint;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "kalman_estimator");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  sub = nh.subscribe<geometry_msgs::PointStamped>("/pose/3d",1,parameterEstimation);
  pub = nh.advertise<geometry_msgs::PointStamped>("/pose/parameter",1);

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
