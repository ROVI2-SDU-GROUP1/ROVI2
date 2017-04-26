#include <LineSampler.hpp>
LineSampler::LineSampler(rw::math::Q _q1, rw::math::Q _q2)
: q1(_q1), q2(_q2), q_direction(q1 - q2), len(q_direction.norm2()), distribution(0,len)
{
    //Normalize the direction vector to a unit vector
    this->q_direction /= this->len;
}

rw::math::Q LineSampler::doSample()
{
    double rand_point = this->distribution(generator);
    return this->q2 + rand_point * q_direction;
}
