//Very simply node. Simply generates points in a balls trajectory
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <cstdlib>
ros::Time start_time;

double x_acc = 3.; double y_acc = -3.; double z_acc = -9.82;
double x_v0 = -5; double y_v0 = 5.; double z_v0 = 35;
double x_0 = 0; double y_0 = 0; double z_0 = 0;


double calc_pos(double acc, double v0, double p0, double ti)
{ //Return the position at time ti ussing the parameters given.
    return 0.5 * acc * ti * ti + v0 * ti + p0;
}

int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "ball_trajectory_generator");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/pose/3d",1);

    start_time = ros::Time::now();
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
      ros::Time t_now = ros::Time::now();
      geometry_msgs::PointStamped pos;
      pos.point.x = calc_pos(x_acc, x_v0, x_0, (t_now - start_time).toSec()) + random() % 1000 / 10000.;
      pos.point.y = calc_pos(y_acc, y_v0, y_0,  (t_now - start_time).toSec()) + random() % 1000 / 10000.;;
      pos.point.z = calc_pos(z_acc, z_v0, z_0,  (t_now - start_time).toSec()) + random() % 1000 / 10000.;;
      pos.header.stamp = t_now;
      pub.publish(pos);
      r.sleep();
    }
    return 0;
}
