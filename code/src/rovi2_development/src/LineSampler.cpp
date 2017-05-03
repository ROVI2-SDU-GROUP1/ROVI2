#include <LineSampler.hpp>
LineSampler *LineSampler::instance = nullptr;

LineSampler::LineSampler(rw::math::Q _q1, rw::math::Q _q2)
: q1(_q1), q2(_q2), q_direction(q1 - q2), distribution(0,1)
{
}

rw::math::Q LineSampler::doSample()
{
    double rand_point = this->distribution(generator);
    //std::cout << "rand: " << rand_point;
    return this->q2 + rand_point * q_direction;
}
