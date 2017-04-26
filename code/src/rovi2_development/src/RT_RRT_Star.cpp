#include "RT_RRT_Star.hpp"
#include <iostream>
#include <rw/rw.hpp>

#include "rw/kinematics/Kinematics.hpp"
#include "rw/math/MetricFactory.hpp"
#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include <limits>

RT_RRT_Star::RT_RRT_Star(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
    rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric, double extend)
    : _rrt(constraint, sampler, metric, extend), goal(_q_start, nullptr), agent(_q_goal, nullptr), RT_Tree(this->agent.getValue()),
    unit_distribution(0, 1)
{
    this->agent.set_cost(0.);
    this->goal.set_cost(std::numeric_limits<double>::max());
}

std::vector<RT_Node> RT_RRT_Star::find_next_path(std::chrono::duration<int64_t> rrt_time)
{   //Algorithm 1
    start_time = std::chrono::steady_clock::now();

    while(std::chrono::steady_clock::now() - start_time < rrt_time)
        this->expand_and_rewire(rrt_time);
    std::vector<RT_Node> path = this->plan_path();
    //Perform line 9 in algorithm 1
    return path;
}

void RT_RRT_Star::expand_and_rewire(std::chrono::duration<int64_t> _time)
{   //Algrorithm 2
    rw::math::Q x_rand = this->create_random_node();
    RT_Node *closest_node = this->find_nearest_node(x_rand);
    if(!this->_rrt.constraint.getQEdgeConstraint().inCollision(closest_node->getValue(), this->goal.getValue() ))
    {

    }
    return;
}

rw::math::Q RT_RRT_Star::create_random_node()
{
    double P_r = this->unit_distribution(this->generator);
    if(P_r <= (1 - this->alpha) / this->beta || this->goal_found)
    {
        return this->_rrt.sampler->sample();
    }
    else if(P_r > 1 - this->alpha)
    {
        LineSampler l_sampler(this->closest->getValue(), this->goal.getValue());
        return l_sampler.doSample();
    }
    else
    {   //Do elipsis sampling
        return this->_rrt.sampler->sample();
    }
}

RT_Node *RT_RRT_Star::find_nearest_node(const rw::math::Q &x)
{
    RT_Node *closest_node = nullptr;
    double shortest_dist = std::numeric_limits<double>::max();
    for (RT_Node *node : this->RT_Tree._nodes)
    {
        if(closest_node == nullptr)
            closest_node = node;
        else if( (x - node->getValue()).norm2() < shortest_dist)
        {
            closest_node = node;
        }
    }
    return closest_node;
}


int main(__attribute__((unused)) int argc, __attribute__((unused)) char const *argv[]) {
    printf("Wow, it works!\n");
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	rw::models::Device::Ptr device = wc->findDevice("UR1");

    rw::math::Metric<rw::math::Q>::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::proximity::CollisionDetector *detector = new rw::proximity::CollisionDetector(wc,
        rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    rw::math::Q q_1(6, 0,0,0,0,0,0);
    rw::math::Q q_2(6, 1,1,1,1,1,1);

    LineSampler *l_sampler = new LineSampler(q_1, q_2);

    for(uint64_t i = 0; i < 10000; i++)
    {
        std::cout << l_sampler->doSample() << std::endl;
    }
    return 0;
}
