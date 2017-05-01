#pragma once
//Implementation of the Real-time RRT Star algorithm
//https://mediatech.aalto.fi/~phamalainen/FutureGameAnimation/p113-naderi.pdf


#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>
#include <chrono>
#include <queue>
#include <random>
#include "RT_RRTNode.hpp"
#include "RT_RRT_Tree.hpp"
#include "LineSampler.hpp"


typedef RT_RRTNode<rw::math::Q> RT_Node;
typedef rwlibs::pathplanners::RT_RRT_Tree<rw::math::Q> Tree;
typedef rw::trajectory::QPath Path;

enum ExtendResult { Trapped, Reached, Advanced };


class RRTStruct
{
public:
    RRTStruct(const rw::pathplanning::PlannerConstraint& _constraint,
        rw::pathplanning::QSampler::Ptr _sampler,
        rw::math::QMetric::Ptr _metric)
        :
        constraint(_constraint),
        sampler(_sampler),
        metric(_metric)
    {
        RW_ASSERT(_sampler);
        RW_ASSERT(_metric);
    }

    rw::pathplanning::PlannerConstraint constraint;
    rw::pathplanning::QSampler::Ptr sampler;
    rw::math::QMetric::Ptr metric;
};


class RT_RRT_Star
{
    public:
        RT_RRT_Star(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
        	rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric);

        //Change the goal position to a new one
        void set_new_goal(rw::math::Q q_newgoal);
        void move_agent(RT_Node *_agent_node);
        void add_node_to_tree(RT_Node &x_new, RT_Node &x_closest, std::vector<RT_Node> &x_near);
        void expand_and_rewire();
        rw::math::Q create_random_node();
        void rewire_random_nodes(double epsilon);
        void rewire_from_tree_root(double epsilon);
        void add_nodes_to_tree(rw::math::Q &x_new, RT_Node * x_closest, std::vector<RT_Node *> &X_near);
        std::vector<RT_Node *> find_next_path(std::chrono::milliseconds rrt_time);
        std::vector<RT_Node *> plan_path();
        RT_Node *find_nearest_node(RT_Node &node) { return this->find_nearest_node(node.getValue()); };
        RT_Node *find_nearest_node(const rw::math::Q &x);
        size_t get_size();
        bool found_solution();

    private:
        double alpha = 0.1;
        double beta = 0.5;
        double k_max = 90;
        double r_s = 90;
        std::queue<RT_Node *> Q_r;
        std::queue<RT_Node *> Q_s;
        RRTStruct _rrt;
        RT_Node *goal;
        RT_Node *agent;
        Tree RT_Tree;
        std::chrono::steady_clock::time_point rewire_expand_deadline;
        std::chrono::steady_clock::time_point rewire_from_root_deadline;

        std::default_random_engine generator;
        std::uniform_real_distribution<double> unit_distribution;
        RT_Node *closest = nullptr;
};
