#pragma once


#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Q.hpp>
#include <random>


class LineSampler;

class LineSampler: public rw::pathplanning::QSampler {
public:
    static LineSampler *get_instance(rw::math::Q _q1, rw::math::Q _q2)
    {

        if(LineSampler::instance)
        {
            LineSampler::instance->q1 = _q1;
            LineSampler::instance->q2 = _q2;
            LineSampler::instance->q_direction = _q1 - _q2;
        }
        else LineSampler::instance = new LineSampler(_q1, _q2);
        return LineSampler::instance;
    }
    virtual rw::math::Q doSample();
    rw::math::Q q1;
    rw::math::Q q2;

private:
    static LineSampler *instance;
    LineSampler(rw::math::Q _q1, rw::math::Q _q2);
    virtual ~LineSampler() {};
    rw::math::Q q_direction;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};
