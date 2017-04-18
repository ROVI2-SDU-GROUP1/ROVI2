#pragma once
//Implementation of the AnytimeRRT algorithm to create itteratively better
//Planning solutions. It should also make it possible to update the goal position
//During computations.

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>
#include <chrono>


typedef rwlibs::pathplanners::RRTNode<rw::math::Q> Node;
typedef rwlibs::pathplanners::RRTTree<rw::math::Q> Tree;
typedef rw::trajectory::QPath Path;

const rw::math::Q& getQ(Node* node) { return node->getValue(); }

enum ExtendResult { Trapped, Reached, Advanced };

class RRTStruct
{
public:
    RRTStruct(const rw::pathplanning::PlannerConstraint& _constraint,
        rw::pathplanning::QSampler::Ptr _sampler,
        rw::math::QMetric::Ptr _metric,
        double _extend)
        :
        constraint(_constraint),
        sampler(_sampler),
        metric(_metric),
        extend(_extend)
    {
        RW_ASSERT(_sampler);
        RW_ASSERT(_metric);
    }

    rw::pathplanning::PlannerConstraint constraint;
    rw::pathplanning::QSampler::Ptr sampler;
    rw::math::QMetric::Ptr metric;
    double extend;
};


class AnytimeRRT
{
    public:
        AnytimeRRT(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
        	rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric, double extend);

        //Change the goal position to a new one
        //Don't really know if we need to connect the goal the tree in here or if we can do it later on.
        void set_new_goal(rw::math::Q q_newgoal);
        void grow_rrt(std::chrono::duration<int64_t> rrt_time); //Use "rrt_time" to improve the path from q_start to q_goal

        std::vector<rw::math::Q> get_current_path();

    private:
        rw::math::Q q_goal;
        rw::math::Q q_start;
        RRTStruct _rrt;

};
