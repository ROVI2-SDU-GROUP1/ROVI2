#include <mutex>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

ros::Subscriber sub2DLeft;
ros::Subscriber sub2DRight;
ros::Publisher pub3D;

std::mutex mutex;
char poses = 0;

geometry_msgs::PointStamped leftPose;
geometry_msgs::PointStamped rightPose;

void calc3DPose(){
  if (poses < 2) return;

  geometry_msgs::PointStamped pose3D;

  // TODO: do some vision

  pub3D.publish(pose3D);
}

void receiveLeftImage(const geometry_msgs::PointStamped::ConstPtr &msg){
  leftPose = *msg;
  mutex.lock();
  poses++;
  mutex.unlock();
  calc3DPose();
}

void receiveRightImage(const geometry_msgs::PointStamped::ConstPtr &msg){
  rightPose = *msg;
  mutex.lock();
  poses++;
  mutex.unlock();
  calc3DPose();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stereo");
  	ros::NodeHandle nh;
  	ros::Rate rate(20);

    sub2DLeft = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_left", 1, receiveLeftImage);
    sub2DRight = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_right", 1, receiveRightImage);
    pub3D = nh.advertise<geometry_msgs::PointStamped>("/pose/3d", 1);

    ros::Time last = ros::Time::now();

    while (ros::ok()){
      last = ros::Time::now();
      ros::spinOnce();
  		rate.sleep();
    }

    return 0;
}
