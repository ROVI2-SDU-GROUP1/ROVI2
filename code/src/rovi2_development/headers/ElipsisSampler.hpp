#pragma once


#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Q.hpp>
#include <random>
#include <UnitBallSampler.hpp>

class ElipsisSampler;

class ElipsisSampler: public rw::pathplanning::QSampler {
    public:
        virtual rw::math::Q doSample();
        ElipsisSampler(const rw::math::Q &_x_start, const rw::math::Q &_x_goal, const double &_c_max)
        : x_start(_x_start), x_goal(_x_goal), c_max(_c_max) {}
        virtual ~ElipsisSampler() {};
    private:
        rw::math::Q x_start;
        rw::math::Q x_goal;
        double c_max;
        UnitBallSampler u_sampler;

};
