//Very simply node. Simply generates points in a balls trajectory
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <random>
ros::Time start_time;

double x_acc = 3.; double y_acc = -3.; double z_acc = -9.82;
double x_v0 = -5; double y_v0 = 5.; double z_v0 = 90;
double x_0 = 5; double y_0 = 0; double z_0 = 0;


double calc_pos(double acc, double v0, double p0, double ti)
{ //Return the position at time ti ussing the parameters given.
    return 0.5 * acc * ti * ti + v0 * ti + p0;
}

int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "ball_trajectory_generator");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/pose/3d",1);
    ros::Publisher pub_raw = nh.advertise<geometry_msgs::PointStamped>("/pose/3d_true",1);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0,0.4);
    start_time = ros::Time::now();
    ros::Rate r(5); // 5 hz
    while (ros::ok())
    {
      ros::Time t_now = ros::Time::now();
      geometry_msgs::PointStamped pos;
      geometry_msgs::PointStamped pos_noisy;

      pos.point.x = calc_pos(x_acc, x_v0, x_0, (t_now - start_time).toSec());
      pos.point.y = calc_pos(y_acc, y_v0, y_0,  (t_now - start_time).toSec());
      pos.point.z = calc_pos(z_acc, z_v0, z_0,  (t_now - start_time).toSec());
      pos_noisy.point.x = pos.point.x + distribution(generator);
      pos_noisy.point.y = pos.point.y + distribution(generator);
      pos_noisy.point.z = pos.point.z + distribution(generator);

      pos.header.stamp = t_now;
      pos_noisy.header.stamp = t_now;

      pub.publish(pos_noisy);
      pub_raw.publish(pos);

      r.sleep();
    }
    return 0;
}
