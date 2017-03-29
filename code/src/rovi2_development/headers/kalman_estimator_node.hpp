#pragma once
#include <eigen3/Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//Some inspiration can be gained from http://www.image.esys.tsukuba.ac.jp/~kitahara/data/pubs/200712_PCM2007-ishii.pdf
class Kalman_Estimator
{
    public:
        Kalman_Estimator();
        ~Kalman_Estimator();
    private:
        void pose_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub_filtered;
        ros::Publisher pub_raw;

        tf2::Stamped<Eigen::Vector3d> prev_points[3]; //The three previosly recieved points
        Eigen::VectorXd cur_state;
        Eigen::Matrix<double, 9, 9> cur_covariance;
        Eigen::Matrix<double, 9, 9> transition_matrix;
        Eigen::Matrix<double, 9, 9> measurement_matrix;

        ros::Time t0;
        uint64_t position_count = 0;
        void do_predict_step();
        void do_update_step(Eigen::VectorXd measured_state);
        void update_transition_matrix(double time_step);

};
