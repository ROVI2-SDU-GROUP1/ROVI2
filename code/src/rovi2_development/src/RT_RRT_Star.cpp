#include "RT_RRT_Star.hpp"
#include <iostream>
#include <rw/rw.hpp>

#include "rw/kinematics/Kinematics.hpp"
#include "rw/math/MetricFactory.hpp"
#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include <limits>
#include <cstdlib>
#include <cmath>

RT_RRT_Star::RT_RRT_Star(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
    rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric)
    : _rrt(constraint, sampler, metric), RT_Tree(_q_start),
    unit_distribution(0, 1)
{
    assert(!this->_rrt.constraint.getQConstraint().inCollision(_q_start));
    assert(!this->_rrt.constraint.getQConstraint().inCollision(_q_goal));

    this->agent = this->RT_Tree.getLastPtr();
    this->agent->set_cost(0.);
    this->RT_Tree.add(_q_goal, nullptr);
    this->goal = this->RT_Tree.getLastPtr();
    this->goal->set_cost(std::numeric_limits<double>::max());
}

std::vector<RT_Node *> RT_RRT_Star::find_next_path(std::chrono::milliseconds rrt_time)
{   //Algorithm 1
    auto clock_now = std::chrono::steady_clock::now();
    this->rewire_expand_deadline = clock_now + rrt_time;
    this->rewire_from_root_deadline = clock_now + rrt_time;
    std::chrono::steady_clock::time_point &global_deadline = this->rewire_from_root_deadline;

    while(std::chrono::steady_clock::now() < global_deadline)
        this->expand_and_rewire();
    std::vector<RT_Node *> path = this->plan_path();
    //Perform line 9 in algorithm 1
    return path;
}

void RT_RRT_Star::expand_and_rewire()
{   //Algrorithm 2
    //printf("Expanding tree!!\n");
    double epsilon = pow( 2 * M_PI, 6) * this->k_max / (M_PI * this->RT_Tree.size());
    rw::math::Q x_rand = this->create_random_node();
    if(!this->_rrt.constraint.getQConstraint().inCollision(x_rand))
    {
        RT_Node *closest_node = this->RT_Tree.get_nearest(x_rand);
        //printf("closest_node %p\n", closest_node);
        if(!this->_rrt.constraint.getQEdgeConstraint().inCollision(closest_node->getValue(), x_rand ))
        {
            epsilon = pow( 2 * M_PI, 6) * this->k_max / (M_PI * this->RT_Tree.size());
            if(epsilon < this->r_s) epsilon = this->r_s;
            std::vector<RT_Node *> nodes_near = this->RT_Tree.get_nodes_within(x_rand, epsilon);
            if(nodes_near.size() < this->k_max || (closest_node->getValue() - x_rand).norm2() > this->r_s )
            {
                this->add_nodes_to_tree(x_rand, closest_node, nodes_near); //add x_rand to the tree
                this->Q_r.push(this->RT_Tree.getLastPtr()); //add x_rand, which is the last node added to the tree.
            }
            else
            {
                this->Q_r.push(closest_node);
            }
            this->rewire_random_nodes(epsilon);
        }
    }
    this->rewire_from_tree_root(epsilon);
    return;
}

void RT_RRT_Star::set_new_goal(rw::math::Q q_newgoal)
{
    //Add the goal to the tree
    #warning "SET NEW GOAL NOT IMPLEMENTED!"

    //Yeah, we cheat. Just loop through the nodes and find the closes one which can connect to the goal
    RT_Node *closest_node = this->RT_Tree.get_nearest(q_newgoal);
    closest_node = nullptr;
    double c_min = std::numeric_limits<double>::max();
    for(auto node : this->RT_Tree._nodes)
    { //The nodes are sorted to be closest to q_newgoal now.
        if(!this->_rrt.constraint.getQEdgeConstraint().inCollision(node->getValue(), q_newgoal))
        {
            closest_node = node;
            c_min = (node->getValue() - q_newgoal).norm2();
            break;
        }
    }

    if(closest_node)
    {
        this->RT_Tree.add(q_newgoal, closest_node);
        this->RT_Tree.getLastPtr()->set_cost( c_min + closest_node->get_cost());
    }
    else
    {
        this->RT_Tree.add(q_newgoal, nullptr);
        this->RT_Tree.getLastPtr()->set_cost( std::numeric_limits<double>::max() );
    }
    return;
}

void RT_RRT_Star::move_agent(RT_Node *_agent_node)
{
    //We can only move the agent to a node in the tree, which follows the computed path.
    //If this is not done, the planner will return suboptimal solutions
    #warning "MOVE AGENT NOT IMPLEMENTED!"
    //We Set the new node to have 0 parents, and a cost of 0 (Do we need to update child node cost?, also, we could set the former parent as a child, with some smart propegation?)
    _agent_node->set_cost(0);
    _agent_node->setParent(nullptr);
    this->agent = _agent_node;
    for(auto node : this->RT_Tree._nodes)
    { //Set rewired to false for all nodes
        node->set_rewired(false);
    }
    //Clear Q_s
    std::queue<RT_Node *>().swap(this->Q_s);
}

void RT_RRT_Star::add_nodes_to_tree(rw::math::Q &x_new, RT_Node * x_closest, std::vector<RT_Node *> &X_near)
{
    RT_Node *x_min = x_closest;
    //printf("x_closest: %p\n", x_closest);
    double c_min = x_closest->get_cost() + (x_closest->getValue() - x_new).norm2();
    for (auto x_near : X_near)
    {
        double c_new = x_near->get_cost() + (x_near->getValue() - x_new).norm2();
        if(c_new < c_min and !this->_rrt.constraint.getQEdgeConstraint().inCollision(x_near->getValue(), x_new))
        {
            c_min = c_new; x_min = x_closest;
        }
    }
    this->RT_Tree.add(x_new, x_min);
    this->RT_Tree.getLastPtr()->set_cost(c_min);
}

void RT_RRT_Star::rewire_random_nodes(double epsilon)
{
    while(std::chrono::steady_clock::now() < this->rewire_expand_deadline and this->Q_r.size())
    {
        RT_Node *x_r = this->Q_r.front();
        this->Q_r.pop();
        std::vector<RT_Node *> X_near = this->RT_Tree.get_nodes_within(x_r->getValue(), epsilon);
        for (auto x_near : X_near)
        {
            double c_old = x_near->get_cost();
            double c_new = x_r->get_cost() + (x_r->getValue() - x_near->getValue()).norm2();
            if(c_new < c_old and !this->_rrt.constraint.getQEdgeConstraint().inCollision(x_r->getValue(), x_near->getValue()))
            {
                if ( x_near == this->goal) std::cout << "Goal got new cost!" << std::endl;
                x_near->set_cost(c_new);
                x_near->setParent(x_r);
                this->Q_r.push(x_r);
            }
        }
    }
}

void RT_RRT_Star::rewire_from_tree_root(double epsilon)
{
    if(!this->Q_s.size())
    {
        this->Q_s.push(this->agent);
    }
    while(std::chrono::steady_clock::now() < this->rewire_from_root_deadline and this->Q_s.size())
    {
        RT_Node *x_s = this->Q_s.front();
        //printf("x_s: %p\n", x_s);
        this->Q_s.pop();
        std::vector<RT_Node *> X_near = this->RT_Tree.get_nodes_within(x_s->getValue(), epsilon);
        for (auto x_near : X_near)
        {
            //printf("x_near: %p\n", x_s);
            double c_old = x_near->get_cost();
            double c_new = x_s->get_cost() + (x_s->getValue() - x_near->getValue()).norm2();
            if(c_new < c_old and !this->_rrt.constraint.getQEdgeConstraint().inCollision(x_s->getValue(), x_near->getValue()))
            {
                if ( x_near == this->goal) std::cout << "Goal got new cost: " << c_new <<  std::endl;
                x_near->set_cost(c_new);
                x_near->setParent(x_s);
                this->Q_s.push(x_s);
            }
            if(x_near->get_rewired() == false)
            {
                x_near->set_rewired(true);
                this->Q_s.push(x_near);
            }
        }
    }
}

std::vector<RT_Node *> RT_RRT_Star::plan_path()
{
    std::vector<RT_Node *> path;
    if(this->goal->get_cost() < std::numeric_limits<double>::max())
    {   //goal was found
        std::cout << "We found the goal!" << std::endl;
        RT_Node * cur_node = this->goal;
        do {
            path.push_back(cur_node);
            cur_node = cur_node->getParent();
        } while(cur_node != nullptr);
        std::reverse(path.begin(),path.end());
    }
    else
    {   //We find the most likely good nodes, all the way from x0
        RT_Node * cur_node = this->agent;
        while(true)
        {
            path.push_back(cur_node);
            if(cur_node->get_childs().size() <= 0) break;
            RT_Node * next_node = nullptr;
            double cheapest = std::numeric_limits<double>::max();
            for(RT_Node * child : cur_node->get_childs())
            {
                if(child->get_cost() + child->get_heuristic(this->goal) < cheapest)
                {
                    cheapest = child->get_cost();
                    next_node = child;
                }
            }
            cur_node = next_node;
        }
    }
    return path;



}

rw::math::Q RT_RRT_Star::create_random_node()
{
    //printf("Created Random node\n");
    double P_r = this->unit_distribution(this->generator);
    if(P_r <= (1 - this->alpha) / this->beta || this->goal->get_cost() < std::numeric_limits<double>::max())
    {
        return this->_rrt.sampler->sample();
    }
    else if(P_r > 1 - this->alpha)
    {
        LineSampler l_sampler(this->closest->getValue(), this->goal->getValue());
        return l_sampler.doSample();
    }
    else
    {   //Do elipsis sampling
        return this->_rrt.sampler->sample();
    }
}

size_t RT_RRT_Star::get_size()
{
    return this->RT_Tree._nodes.size();
}

bool RT_RRT_Star::found_solution()
{
    return this->goal->get_cost() < std::numeric_limits<double>::max();
}




int main(__attribute__((unused)) int argc, __attribute__((unused)) char const *argv[]) {
    printf("Wow, it works!\n");
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	rw::models::Device::Ptr device = wc->findDevice("UR1");

    rw::math::Metric<rw::math::Q>::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    __attribute__((unused)) rw::proximity::CollisionDetector *detector = new rw::proximity::CollisionDetector(wc,
        rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    rw::kinematics::State state = wc->getDefaultState();;

    rw::math::Q q_1(6, 0.0, -1.573, 0.0, -1.573, 0., 0.);
    rw::math::Q q_2(6, 1.729, 0.107, 0.069, -2.857, -4.981, -0.527);
    device->setQ(q_1, state);
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(detector,device,state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::QSampler::makeUniform(device),constraint.getQConstraintPtr());

    RT_RRT_Star rt_rrt_star(q_1, q_2, constraint, sampler, metric);
    std::chrono::milliseconds time_to_solve{1000};
    for(uint64_t itterations = 0; rt_rrt_star.found_solution() == false; itterations++)
    {
        for(auto node : rt_rrt_star.find_next_path(time_to_solve))
        {
            std::cout << node->getValue() << std::endl;
        }
        std::cout << "End of itteration " << itterations << " Tree size: " << rt_rrt_star.get_size() << std::endl;
    }
    std::cout << "Tree size at exit: " << rt_rrt_star.get_size() << std::endl;
    return 0;




    __attribute__((unused)) LineSampler *l_sampler = new LineSampler(q_1, q_2);

    /*for(uint64_t i = 0; i < 10000; i++)
    {
        std::cout << l_sampler->doSample() << std::endl;
    }*/
    //Test the flann implementation
    rwlibs::pathplanners::RT_RRT_Tree<rw::math::Q> RT_Tree(q_1);
    //auto flann_index = RT_Tree.getFlannIndex();
    for(uint32_t i = 0; i < 100000; i++)
    {
        rw::math::Q q_rand(6,   rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.);

        RT_Tree.add(q_rand, nullptr);
    }
    for(auto  node : RT_Tree.get_n_nearest(q_1, 10))
    {
        std::cout << node->getValue() << " " << (node->getValue() - q_1).norm2() << std::endl;
    }
    for(auto  node : RT_Tree.get_nodes_within(q_1, 5))
    {
        std::cout << node->getValue() << " " << (node->getValue() - q_1).norm2() << std::endl;
    }

    return 0;
}
