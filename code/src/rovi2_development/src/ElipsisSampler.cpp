#include <ElipsisSampler.hpp>
#include <UnitBallSampler.hpp>

rw::math::Q ElipsisSampler::doSample()
{
    auto x_center = (this->x_start + this->x_goal) / 2;
    //Create a 6D roration matrix.
    //https://arxiv.org/pdf/1404.2334.pdf
    //(6)
    auto dir_vec = this->x_goal - this->x_start;
    double c_min = dir_vec.norm2();
    rw::math::Q unit_dir_vec = dir_vec / c_min;
    Eigen::Matrix<double, 6, 6> I_6 = Eigen::MatrixXd::Identity(6, 6);

    Eigen::MatrixXd M = unit_dir_vec.e() * I_6.col(0).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto diag_matrix = I_6;
    diag_matrix(diag_matrix.cols() - 1, diag_matrix.cols() - 1) = svd.matrixU().determinant() * svd.matrixV().determinant();
    //std::cout << diag_matrix << std::endl << std::endl;
    Eigen::MatrixXd C = svd.matrixU() * diag_matrix * svd.matrixV().transpose();
    //std::cout << C << std::endl;
    // C is now our rotation matrix for rotating a point into the direction x_start->x_goal

    //We create a matrix, L, which describes the size of each axis of the elipsis.
    auto L = I_6;
    L(0,0) = this->c_max / 2;
    for(uint8_t i = 1; i < 6; i++) L(i,i) = sqrt(this->c_max * this->c_max - c_min * c_min) / 2.;
    //Now, we draw a sample uniformly distributed in a 6D unitball.
    auto q_rand_ball = UnitBallSampler::get_instance()->doSample();
    //Transform into an elipsis, rotate it to the correct orientation, and center it between start and goal.
    auto x_rand = C * L * q_rand_ball.e() + x_center.e();
    return rw::math::Q(x_rand);
}
