#include "AnytimeRRT.hpp"
#include <cstdio>

AnytimeRRT::AnytimeRRT(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
    rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric, double extend)
    : _rrt(constraint, sampler, metric, extend)
{
    this->q_start = _q_start;
    this->q_goal = _q_goal;
}


int main(__attribute__((unused)) int argc, __attribute__((unused)) char const *argv[]) {
    printf("Wow, it works!\n");
    auto metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
        rw::proximity::ProximityStrategyFactory::makeDefaultCollisionStrategy detector =
        new rw::proximity::CollisionDetector(wc, rw::proximity::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    return 0;
}
