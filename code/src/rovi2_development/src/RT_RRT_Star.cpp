#include "RT_RRT_Star.hpp"
#include <iostream>
#include <rw/rw.hpp>

#include "rw/kinematics/Kinematics.hpp"
#include "rw/math/MetricFactory.hpp"
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>

#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include <limits>
#include <cstdlib>
#include <cmath>
//TODO: This pathplanning strategy is very slow apperently...
//Try to combine this aproach of rewiring and heurisitcs with RRT_Connect

double get_dist(rw::math::Q &q1, rw::math::Q &q2)
{
    auto diff = q1 - q2;
    double dist = 0;
    for(uint8_t i = 0; i < 6; i++)
    {
        dist += pow(diff[i], 2);
    }
    return sqrt(dist);
}

double get_path_length(rw::trajectory::QPath &path)
{
    double length = 0;
    for(size_t i = 1; i < path.size(); i++)
    {   double tmp = get_dist(path[i - 1], path[i]);
        length += tmp;
    }
    return length;
}



double get_dist(const rw::math::Q &q1, const rw::math::Q &q2)
{
    auto diff = q1 - q2;
    double dist = 0;
    for(uint8_t i = 0; i < 6; i++)
    {
        dist += pow(diff[i], 2);
    }
    return sqrt(dist);
}

uint64_t fac(uint64_t n)
{
    if(n == 1)
        return 1;
    return n * fac(n - 1);
}


double get_path_length(std::vector<RT_Node *> &path)
{
    double length = 0;
    for(size_t i = 1; i < path.size(); i++)
    {   double tmp = get_dist( path[i - 1]->getValue() , path[i]->getValue() );
        length += tmp;
    }
    return length;
}




RT_RRT_Star::RT_RRT_Star(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
    rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric, double _closeness)
    : _rrt(constraint, sampler, metric), startTree(_q_start), goalTree(_q_goal), TreeA(&startTree), TreeB(&goalTree), closeness(_closeness),
    unit_distribution(0, 1),  u_X(pow( M_PI, 6 / 2 ) / fac( 6 / 2 + 1) * pow( (2 * M_PI), 6))
{
    assert(!this->_rrt.constraint.getQConstraint().inCollision(_q_start));
    assert(!this->_rrt.constraint.getQConstraint().inCollision(_q_goal));



    this->agent = this->startTree.getLastPtr();
    this->agent->set_cost(0.);
    //this->startTree.add(_q_goal, nullptr);
    this->goal = goalTree.getLastPtr();
    this->goal->set_cost(std::numeric_limits<double>::max());
    this->closest = this->agent;

    if (!_rrt.constraint.getQEdgeConstraint().inCollision(_q_start, _q_goal)) {
        this->startTree.add(this->goal->getValue(), this->agent);
        this->closest = this->startTree.getLastPtr();
    }

}

std::vector<RT_Node *> RT_RRT_Star::find_next_path(std::chrono::milliseconds rrt_time)
{   //Algorithm 1
    auto clock_now = std::chrono::steady_clock::now();
    this->rewire_expand_deadline = clock_now + rrt_time;
    this->rewire_from_root_deadline = clock_now + rrt_time;
    std::chrono::steady_clock::time_point &global_deadline = this->rewire_from_root_deadline;

    while(std::chrono::steady_clock::now() < global_deadline)
    {
        if(this->found_solution())
            this->expand_and_rewire();
        else
            this->connect_trees();
    }
    std::vector<RT_Node *> path = this->plan_path();
    //Perform line 9 in algorithm 1
    return path;
}

ExtendResult RT_RRT_Star::growTree(
    Tree *tree,
    const rw::math::Q& q)
{
    RT_Node* qNearNode = tree->get_nearest(q);
    return this->extend(tree, q, qNearNode);
}

ExtendResult RT_RRT_Star::extend(Tree *tree,
                    const rw::math::Q& q,
                    RT_Node* qNearNode)
{
    const rw::math::Q& qNear = qNearNode->getValue();
    const rw::math::Q delta = q - qNear;
    const double dist = delta.norm2();

    if (dist <= this->rrt_connect_epsilon) {
        if (!_rrt.constraint.getQEdgeConstraint().inCollision(qNearNode->getValue(), q)) {
            tree->add(q, qNearNode);
            return Reached;
        } else {
            return Trapped;
        }
    } else {
        const rw::math::Q qNew = qNear + (this->rrt_connect_epsilon / dist) * delta;
        if (!_rrt.constraint.getQEdgeConstraint().inCollision(qNearNode->getValue(), qNew)) {
            tree->add(qNew, qNearNode);
            return Advanced;
        } else {
            return Trapped;
        }
    }
}

ExtendResult RT_RRT_Star::connect(
    Tree *tree,
    const rw::math::Q& q)
{
    RT_Node* qNearNode = tree->get_nearest(q);

    ExtendResult s = Advanced;
    bool hasAdvanced = false;
    while (s == Advanced) {
        s = extend(tree, q, qNearNode);
        if (s == Advanced) {
            qNearNode = tree->getLastPtr();
            hasAdvanced = true;
        }
    }

    if (s == Trapped && hasAdvanced)
        return Advanced;
    else
        return s;
}




void RT_RRT_Star::connect_trees()
{ //Perform a sligtly modified RRT_Connect untill we have connected the trees
    auto uniform_rand = this->_rrt.sampler->sample();

    if (growTree(this->TreeA, uniform_rand) != Trapped &&
        connect(this->TreeB, this->TreeA->getLast().getValue()) == Reached)
    {
        //The trees are now connected, do something
        //We need to merge the goal and start tree
        this->mergeTrees();
        printf("Trees are connected now!\n");

    }
    std::swap(this->TreeA, this->TreeB);
}

void RT_RRT_Star::mergeTrees()
{   //We merge the two trees, startTree and goalTree.
    //The newest added node to each tree __needs__ to be at the same coordinate

    std::cout << this->startTree.getLast().getValue() << "\t" << this->goalTree.getLast().getValue() << std::endl;
    std::cout << this->startTree.getLastPtr() << "\t" << this->goalTree.getLastPtr() << std::endl;
    //Find the first node to add to the tree
    RT_Node *next_to_fix = this->goalTree.getLastPtr();
    printf("next_to_fix: %p\n", next_to_fix);
    RT_Node *next_parent = this->startTree.getLastPtr();
    printf("next_parent: %p\n", next_parent);
    RT_Node *next_n = next_to_fix;
    //Move all goalTree nodes to startTree
    for(auto node : this->goalTree._nodes)
    {
        //printf("Adding %p\n", node);
        this->startTree.add(node);
    }
    //Re-establish the path from first_node_to_add to goal

    while(next_to_fix != nullptr)
    {
        //printf("fixing %p\n", next_to_fix);
        RT_Node *tmp = next_to_fix->getParent();
        next_to_fix->setParent(next_parent);
        next_parent = next_to_fix;
        next_to_fix = tmp;
    }
    //printf("goal is %p\n", this->goal);
    this->closest = this->goal;
    this->goalTree.clear_unsafe();
}

double RT_RRT_Star::getEpsilon()
{
    double epsilon = sqrt( this->u_X * this->k_max / (M_PI * this->startTree.size()) );
    return std::max(epsilon, this->r_s);
}

void RT_RRT_Star::expand_and_rewire()
{   //Algrorithm 2
    //printf("Expanding tree!!\n");

    double epsilon = this->getEpsilon();
    rw::math::Q x_rand = this->create_random_node();
    //std::cout << "Distance from rand to goal: " << (x_rand - this->goal->getValue()).norm2() << " rand:" << x_rand << " closest: " << this->closest->getValue() << std::endl;
    //for(uint8_t i = 2; i < x_rand.size(); i++ ) x_rand[i] = 0.;
    if(!this->_rrt.constraint.getQConstraint().inCollision(x_rand))
    {
        bool added = false;
        //std::cout << x_rand << std::endl;
        epsilon = this->getEpsilon();
        if(epsilon < this->r_s) epsilon = this->r_s;
        std::vector<RT_Node *> nodes_near = this->startTree.get_nodes_within(x_rand, epsilon);
        //std::cout << epsilon << ", " << this->u_X <<  ", " << nodes_near.size() << std::endl;
        if(nodes_near.size() < this->k_max || (nodes_near[0]->getValue() - x_rand).norm2() > this->r_s )
        {
            if(this->add_nodes_to_tree(x_rand, nodes_near))//add x_rand to the tree
            {
                this->Q_r.push(this->startTree.getLastPtr()); //add x_rand, which is the last node added to the tree.
                added = true;
            }
        }
        else
        {
            this->Q_r.push(nodes_near[0]);
        }
        if(added)
            this->rewire_random_nodes(epsilon);
        this->rewire_from_tree_root(epsilon);
    }
    return;
}

void RT_RRT_Star::set_new_goal(rw::math::Q q_newgoal)
{
    assert(!this->_rrt.constraint.getQConstraint().inCollision(q_newgoal));
    this->goalTree.clear_unsafe();
    this->goalTree.add(q_newgoal, nullptr);
    this->goal = this->goalTree.getLastPtr();
    this->TreeA = &this->goalTree;
    this->TreeB = &this->startTree;
    this->closest = this->startTree.get_nearest(q_newgoal);
    return;
}

//TODO: cost recomputation of the path to x0
//TODO: Don't add goal to the tree, instead just keep it in this class, and have a closeness check, when adding a node, to check if we are close enough.

void RT_RRT_Star::move_agent(RT_Node *_agent_node)
{
    //We can only move the agent to a node in the tree, which follows the computed path.
    //If this is not done, the planner will return suboptimal solutions
    #warning "MOVE AGENT NOT IMPLEMENTED!"
    //We Set the new node to have 0 parents, and a cost of 0 (Do we need to update child node cost?, also, we could set the former parent as a child, with some smart propegation?)
    _agent_node->set_cost(0);
    _agent_node->setParent(nullptr);
    this->agent = _agent_node;
    for(auto node : this->startTree._nodes)
    { //Set rewired to false for all nodes
        node->set_rewired(false);
    }
    //Clear Q_s
    std::queue<RT_Node *>().swap(this->Q_s);
}

bool RT_RRT_Star::add_nodes_to_tree(rw::math::Q &x_new, std::vector<RT_Node *> &X_near)
{
    RT_Node *x_min = nullptr;
    //printf("x_closest: %p\n", x_closest);
    double c_min = std::numeric_limits<double>::max();
    for (auto x_near : X_near)
    {
        double c_new = x_near->get_cost() + (x_near->getValue() - x_new).norm2();
        if(c_new < c_min and !this->_rrt.constraint.getQEdgeConstraint().inCollision(x_near->getValue(), x_new) )
        {
            c_min = c_new; x_min = x_near;
        }
    }
    if(c_min == std::numeric_limits<double>::max()) return false;
    this->startTree.add(x_new, x_min);
    this->startTree.getLastPtr()->set_cost(c_min);
    if( (this->startTree.getLastPtr()->getValue() - this->goal->getValue()).norm2() < (this->closest->getValue() - this->goal->getValue()).norm2() )
    {
        this->closest = this->startTree.getLastPtr();
        std::cout << "closest node is now: " << this->closest->getValue() << " with cost" << this->closest->get_cost()
            << " There was " << X_near.size() << " nodes in X_near. dist to goal: "  << (this->closest->getValue() - this->goal->getValue()).norm2() <<  std::endl;
    }
    return true;
}

void RT_RRT_Star::rewire_random_nodes(double epsilon)
{
    while(std::chrono::steady_clock::now() < this->rewire_expand_deadline and this->Q_r.size())
    {
        RT_Node *x_r = this->Q_r.front();
        this->Q_r.pop();
        std::vector<RT_Node *> X_near = this->startTree.get_nodes_within(x_r->getValue(), epsilon);
        for (auto x_near : X_near)
        {
            double c_old = x_near->get_cost();
            double c_new = x_r->get_cost() + (x_r->getValue() - x_near->getValue()).norm2();
            if(c_new < c_old and (!this->_rrt.constraint.getQEdgeConstraint().inCollision(x_r->getValue(), x_near->getValue())) )
            {
                //if ( x_near == this->goal) std::cout << "Goal got new cost!" << std::endl;
                x_near->set_cost(c_new);
                x_near->setParent(x_r);
                this->Q_r.push(x_near);
            }
        }
    }
}

uint64_t RT_RRT_Star::nodes_without_parents()
{
    for (auto node : this->startTree._nodes)
    {
        if(!node->getParent())
        {
            std::cout << node->getValue() << std::endl;
        }
    }

    return 0;
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
        std::vector<RT_Node *> X_near = this->startTree.get_nodes_within(x_s->getValue(), epsilon);
        for (auto x_near : X_near)
        {
            //printf("x_near: %p\n", x_s);
            double c_old = x_near->get_cost();
            double c_new = x_s->get_cost() + (x_s->getValue() - x_near->getValue()).norm2();
            if(c_new < c_old and !this->_rrt.constraint.getQEdgeConstraint().inCollision(x_s->getValue(), x_near->getValue()) )
            {
                //if ( x_near == this->goal) std::cout << "Goal got new cost: " << c_new <<  std::endl;
                x_near->set_cost(c_new);
                x_near->setParent(x_s);
                //this->Q_s.push(x_s);
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
    if(this->found_solution())
    {   //goal was found
        std::cout << "We found the goal!" << std::endl;
        RT_Node * cur_node = this->closest;
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
    if(P_r > 1 - this->alpha and !this->found_solution())
    {
        LineSampler *l_sampler = LineSampler::get_instance(this->closest->getValue(), this->goal->getValue());
        //std::cout << "Sampling from " << closest->getValue() << " to " << this->goal->getValue() << std::endl;
        return l_sampler->doSample();
    }
    else if(P_r <= (1 - this->alpha) / this->beta || this->found_solution())
    {
        return this->_rrt.sampler->sample();
    }
    else
    {   //Do elipsis sampling
        return this->_rrt.sampler->sample();
    }
}

size_t RT_RRT_Star::get_size()
{
    return this->startTree._nodes.size();
}

bool RT_RRT_Star::found_solution()
{
    return this->goal == this->closest;
}

void RT_RRT_Star::validate_tree_structure()
{
    for (auto node : this->startTree._nodes)
    {
        //Validate parent -> child relation
        if(node->getParent())
        {
            bool node_is_child = false;
            for(auto child : node->getParent()->get_childs())
            {
                if(child ==node)
                {
                    node_is_child = true;
                }
            }
            assert(node_is_child);
        }
        //Validate child ->parent relation
        for(auto child : node->get_childs())
        {
            assert(child->getParent() == node);
        }
    }
}

void validate_path(std::vector<RT_Node *> &path, const rw::pathplanning::PlannerConstraint& constraint)
{
    for(auto node : path)
    {
        for(auto child : node->get_childs())
            assert(!constraint.getQEdgeConstraint().inCollision(child->getValue(), node->getValue()));
    }
}



int main(__attribute__((unused)) int argc, __attribute__((unused)) char const *argv[]) {
    printf("Wow, it works!\n");
    printf("Compile info: GCC %u.%u.%u\t", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    printf("Compile date: %s -- %s\n", __DATE__, __TIME__);

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	rw::models::Device::Ptr device = wc->findDevice("UR1");

    rw::math::Metric<rw::math::Q>::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    __attribute__((unused)) rw::proximity::CollisionDetector *detector = new rw::proximity::CollisionDetector(wc,
        rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    rw::kinematics::State state = wc->getDefaultState();;

    rw::math::Q q_1(6, 0, -1.5, -0.298, -0.341, 0, 0);
    rw::math::Q q_2(6, -1.032, -3.186, -0.298, -0.341, 0, 0);
    device->setQ(q_1, state);
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(detector,device,state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::QSampler::makeUniform(device),constraint.getQConstraintPtr());



    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, 1, rwlibs::pathplanners::RRTPlanner::RRTConnect);
    rw::trajectory::QPath path;
    #define MAXTIME 1000.

    planner->query(q_1,q_2,path,MAXTIME);
    for(auto q_ : path)
    {
        std::cout << q_ << std::endl;
    }
    //return 0;
    RT_RRT_Star rt_rrt_star(q_1, q_2, constraint, sampler, metric);
    std::chrono::milliseconds time_to_solve{1000};
    for(uint64_t itterations = 0; rt_rrt_star.found_solution() == false || itterations < 200; itterations++)
    {
        auto new_path = rt_rrt_star.find_next_path(time_to_solve);
        validate_path(new_path, constraint);
        for(auto node : new_path)
        {
            std::cout << node->getValue() << std::endl;
        }
        std::cout << "End of itteration " << itterations << " Tree size: " << rt_rrt_star.get_size() << std::endl;
        rt_rrt_star.nodes_without_parents();
        rt_rrt_star.validate_tree_structure();
        std::cout << get_path_length(new_path) << std::endl;
    }
    std::cout << "Tree size at exit: " << rt_rrt_star.get_size() << std::endl;

    return 0;




    __attribute__((unused)) LineSampler *l_sampler = LineSampler::get_instance(q_1, q_2);

    /*for(uint64_t i = 0; i < 10000; i++)
    {
        std::cout << l_sampler->doSample() << std::endl;
    }*/
    //Test the flann implementation
    Tree startTree(q_1);
    //auto flann_index = startTree.getFlannIndex();
    for(uint32_t i = 0; i < 100000; i++)
    {
        rw::math::Q q_rand(6,   rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.,
                                rand() % 10000 / 10000.);

        startTree.add(q_rand, nullptr);
    }
    for(auto  node : startTree.get_n_nearest(q_1, 10))
    {
        std::cout << node->getValue() << " " << (node->getValue() - q_1).norm2() << std::endl;
    }
    for(auto  node : startTree.get_nodes_within(q_1, 5))
    {
        std::cout << node->getValue() << " " << (node->getValue() - q_1).norm2() << std::endl;
    }

    return 0;
}
