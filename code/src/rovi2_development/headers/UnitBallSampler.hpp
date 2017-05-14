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
    virtual ~UnitBallSampler() {};
    static UnitBallSampler *get_instance()
    {
        if(UnitBallSampler::instance == nullptr)
            UnitBallSampler::instance = new UnitBallSampler();
        return instance;
    }
private:
    static UnitBallSampler *instance;
    UnitBallSampler();
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};
