#pragma once


#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Q.hpp>
#include <random>

class LineSampler: public rw::pathplanning::QSampler {
public:
	LineSampler(rw::math::Q _q1, rw::math::Q _q2);
    virtual rw::math::Q doSample();
	virtual ~LineSampler() {};
    rw::math::Q q1;
    rw::math::Q q2;

private:
    rw::math::Q q_direction;
    std::default_random_engine generator;
    double len;
    std::uniform_real_distribution<double> distribution;
};
