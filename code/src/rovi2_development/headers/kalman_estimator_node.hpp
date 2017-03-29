#pragma once
#include <eigen3/Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Kalman_Estimator
{
    public:
        Kalman_Estimator();
        ~Kalman_Estimator();
    private:
        void pose_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        tf2::Stamped<Eigen::Vector3d> prev_points[3]; //The three previosly recieved points
};
