#pragma once


#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Q.hpp>
#include <random>
#include <LineSampler.hpp>


class UnitBallSampler;

class UnitBallSampler: public rw::pathplanning::QSampler {
public:
    virtual rw::math::Q doSample();
    UnitBallSampler();
    virtual ~UnitBallSampler() {};

private:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};
