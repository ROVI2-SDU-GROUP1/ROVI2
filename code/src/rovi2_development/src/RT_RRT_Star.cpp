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
#include <ElipsisSampler.hpp>

static rw::pathplanning::PlannerConstraint *stat_constraint = nullptr;


bool inBounds(rw::models::CompositeDevice::QBox bounds, rw::math::Q q)
{
    for(uint8_t i = 0; i < q.size(); i++)
    {
        if(bounds.first[i] > q[i] or bounds.second[i] < q[i])
            return false;
    }
    return true;
}

void validate_path(std::vector<RT_Node *> &path, const rw::pathplanning::PlannerConstraint& constraint);

void validate_path(std::vector<RT_Node *> &path)
{
    validate_path(path, *stat_constraint);
}



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



bool inCollision(const rw::pathplanning::PlannerConstraint& constraint, const RT_Node *q)
{
    return constraint.getQConstraint().inCollision(q->getValue());
}

bool inCollision(const rw::pathplanning::PlannerConstraint& constraint, const rw::math::Q& q)
{
    return constraint.getQConstraint().inCollision(q);
}


// 'node' is known to be collision free, but 'b' is not.
bool inCollision(const rw::pathplanning::PlannerConstraint& constraint,
                 RT_Node *a,
                 const rw::math::Q& b)
{
    return
    constraint.getQConstraint().inCollision(b) || constraint.getQConstraint().inCollision(a->getValue()) ||
        constraint.getQEdgeConstraint().inCollision(a->getValue(), b);
}

bool inCollision(const rw::pathplanning::PlannerConstraint& constraint,
                 const rw::math::Q& a,
                 RT_Node *b)
{
    return
    constraint.getQConstraint().inCollision(b->getValue()) || constraint.getQConstraint().inCollision(a) ||
        constraint.getQEdgeConstraint().inCollision(a, b->getValue());
}

bool inCollision(const rw::pathplanning::PlannerConstraint& constraint,
                 const rw::math::Q& a,
                 const rw::math::Q& b)
{
    return
    constraint.getQConstraint().inCollision(b) || constraint.getQConstraint().inCollision(a) ||
        constraint.getQEdgeConstraint().inCollision(a, b);
}

bool inCollision(const rw::pathplanning::PlannerConstraint& constraint,
                 RT_Node *a,
                 RT_Node *b)
{
    return
        constraint.getQConstraint().inCollision(b->getValue()) || constraint.getQConstraint().inCollision(a->getValue()) ||
        constraint.getQEdgeConstraint().inCollision(a->getValue(), b->getValue());
}


RT_RRT_Star::RT_RRT_Star(rw::math::Q _q_start, rw::math::Q _q_goal, const rw::pathplanning::PlannerConstraint& constraint,
    rw::pathplanning::QSampler::Ptr sampler, rw::math::QMetric::Ptr metric, rw::models::Device::Ptr _device, double _closeness)
    : _rrt(constraint, sampler, metric), startTree(_q_start), goalTree(_q_goal), TreeA(&startTree), TreeB(&goalTree), closeness(_closeness), device(_device),
    unit_distribution(0, 1),  u_X(pow( M_PI, 6 / 2 ) / fac( 6 / 2 + 1) * pow( (2 * M_PI), 6))
{
    assert(!inCollision(this->_rrt.constraint, _q_start));
    assert(!inCollision(this->_rrt.constraint, _q_goal));



    this->agent = this->startTree.getLastPtr();
    this->agent->set_cost(0.);
    this->goal = goalTree.getLastPtr();
    this->goal->set_cost(std::numeric_limits<double>::max());
    this->closest = this->agent;

    if (!inCollision(this->_rrt.constraint, _q_start, _q_goal)) {
        this->startTree.add(this->goal->getValue(), this->agent);
        this->closest = this->startTree.getLastPtr();
    }

}

std::vector<RT_Node *> RT_RRT_Star::find_next_path(std::chrono::milliseconds rrt_time)
{   //Algorithm 1
    if(this->goal->get_cost() <= (this->goal->getValue() - this->agent->getValue()).norm2() )
    {
        return this->plan_path();
    }

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
        if (!inCollision(this->_rrt.constraint, qNearNode, q)) {
            //assert(!inCollision(this->_rrt.constraint, q));
            tree->add(q, qNearNode);
            return Reached;
        } else {
            return Trapped;
        }
    } else {
        const rw::math::Q qNew = qNear + (this->rrt_connect_epsilon / dist) * delta;
        if (!inCollision(this->_rrt.constraint, qNearNode, qNew)) {
            /*assert(!inCollision(this->_rrt.constraint, qNew));
            assert(!inCollision(this->_rrt.constraint, qNearNode));
            assert(!inCollision(this->_rrt.constraint, qNearNode, qNew));
            assert(!this->_rrt.constraint.getQEdgeConstraint().inCollision(qNearNode->getValue(), qNew));
            assert(!this->_rrt.constraint.getQEdgeConstraint().inCollision(qNew, qNearNode->getValue()));

            assert(!inCollision(this->_rrt.constraint, qNew, qNearNode ));*/

            //std::cout << qNearNode->getValue() << "\t" << qNew << std::endl;
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
        //validate_path(this->startTree._nodes, this->_rrt.constraint);
        //validate_path(this->goalTree._nodes, this->_rrt.constraint);
        this->mergeTrees();
        //printf("Trees are connected now!\n");

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
    //printf("next_to_fix: %p\n", next_to_fix);
    RT_Node *next_parent = this->startTree.getLastPtr();
    //printf("next_parent: %p\n", next_parent);
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
    if(!inCollision(this->_rrt.constraint, x_rand))
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
    assert(!inCollision(this->_rrt.constraint, q_newgoal));
    if (!inCollision(this->_rrt.constraint, this->agent->getValue(), q_newgoal)) {
        this->startTree.add(q_newgoal, this->agent);
        this->closest = this->startTree.getLastPtr();
        this->goal = this->startTree.getLastPtr();
    }
    else
    {
        this->goalTree.clear();
        this->goalTree.add(q_newgoal, nullptr);
        this->goal = this->goalTree.getLastPtr();
        this->goal->set_cost(std::numeric_limits<double>::max());
        this->TreeA = &this->goalTree;
        this->TreeB = &this->startTree;
        this->closest = this->startTree.get_nearest(q_newgoal);
    }
    return;

}

void RT_RRT_Star::move_agent(RT_Node *_agent_node)
{
    //We can only move the agent to a node in the tree, which follows the computed path.
    //If this is not done, the planner may return suboptimal solutions
    //#warning "MOVE AGENT NOT IMPLEMENTED!"
    //We Set the new node to have 0 parents, and a cost of 0. Also, we propegate this as the new tree root
    this->propegate_new_agent(_agent_node->getParent(), _agent_node);
    //printf("Propegation done\n");
    _agent_node->set_cost(0);
    _agent_node->setParent(nullptr);
    this->agent = _agent_node;
    //printf("agent changed\n");
    for(auto node : this->startTree._nodes)
    { //Set rewired to false for all nodes
        node->set_rewired(false);
    }
    //Clear Q_s
    std::queue<RT_Node *>().swap(this->Q_s);
}

void RT_RRT_Star::propegate_new_agent(RT_Node * node, RT_Node *new_parent)
{   //This function propegates the updates of node child->parent relationships when the agent is moved.
    //This should propegate towards the old agent and stop when it is reached.
    //validate_path(this->startTree._nodes, this->_rrt.constraint);
    if(node == nullptr) return;
    //printf("Updating relationships %p->%p to %p->%p\n", node, node->getParent(), node, new_parent);
    auto old_parent = node->getParent();
    node->setParent(new_parent);
    this->propegate_new_agent(old_parent, node);
}

bool RT_RRT_Star::add_nodes_to_tree(rw::math::Q &x_new, std::vector<RT_Node *> &X_near)
{
    RT_Node *x_min = nullptr;
    //printf("x_closest: %p\n", x_closest);
    double c_min = std::numeric_limits<double>::max();
    for (auto x_near : X_near)
    {
        double c_new = x_near->get_cost() + (x_near->getValue() - x_new).norm2();
        if(c_new < c_min and !inCollision(this->_rrt.constraint, x_near->getValue(), x_new) )
        {
            c_min = c_new; x_min = x_near;
        }
    }
    if(c_min == std::numeric_limits<double>::max()) return false;
    //validate_path(this->startTree._nodes, this->_rrt.constraint);
    //printf("Adding %p\n", x_min);
    this->startTree.add(x_new, x_min);
    //validate_path(this->startTree._nodes, this->_rrt.constraint);
    //printf("%p added without issues\n", x_min);
    this->startTree.getLastPtr()->set_cost(c_min);
    if( (this->startTree.getLastPtr()->getValue() - this->goal->getValue()).norm2() < (this->closest->getValue() - this->goal->getValue()).norm2() )
    {
        this->closest = this->startTree.getLastPtr();
        //std::cout << "closest node is now: " << this->closest->getValue() << " with cost" << this->closest->get_cost()
        //    << " There was " << X_near.size() << " nodes in X_near. dist to goal: "  << (this->closest->getValue() - this->goal->getValue()).norm2() <<  std::endl;
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
            if(c_new < c_old and (!inCollision(this->_rrt.constraint, x_r->getValue(), x_near->getValue())) )
            {
                //if ( x_near == this->goal) std::cout << "Goal got new cost!" << std::endl;
                //validate_path(this->startTree._nodes, this->_rrt.constraint);
                //printf("rewiring %p %p\n", x_near, x_r);
                x_near->set_cost(c_new);
                x_near->setParent(x_r);
                //validate_path(this->startTree._nodes, this->_rrt.constraint);
                //printf("%p %p rewired successfully\n", x_near, x_r);

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
            if(c_new < c_old and !inCollision(this->_rrt.constraint, x_s->getValue(), x_near->getValue()) )
            {
                //validate_path(this->startTree._nodes, this->_rrt.constraint);
                //printf("rewiring %p %p\n", x_near, x_s);
                x_near->set_cost(c_new);
                x_near->setParent(x_s);
                //validate_path(this->startTree._nodes, this->_rrt.constraint);
                //printf("%p %p rewired successfully\n", x_near, x_s);
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
    /*double P_r = this->unit_distribution(this->generator);
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
    else*/
    {   //Do elipsis sampling
        ElipsisSampler e_sampler(this->agent->getValue(), this->goal->getValue(), this->goal->get_cost());
        while(true)
        {
            auto q_rand_elipsis = e_sampler.doSample();
            if(inBounds(this->device->getBounds(), q_rand_elipsis)) return q_rand_elipsis;
        }
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
            assert(child != child->getParent());
            assert(node != child);
        }
    }
}

void validate_path(std::vector<RT_Node *> &path, const rw::pathplanning::PlannerConstraint& constraint)
{
    for(auto node : path)
    {
        assert(!inCollision(constraint,node->getValue()));
        for(auto child : node->get_childs())
        {
            std::cout << "Checking collision " << child->getValue() << "->" << node->getValue() << "\t" << child << "->" << node << std::endl;
            assert(!inCollision(constraint, child->getValue()));
            assert(!inCollision(constraint, node->getValue()));
            assert(!inCollision(constraint, child->getValue(), node->getValue()));
        }
    }
}

RT_Node *RT_RRT_Star::get_random_node()
{
    return this->startTree.get_random_node();
}


int main(__attribute__((unused)) int argc, __attribute__((unused)) char const *argv[]) {
    rw::math::Math::seed(time(NULL));
    //printf("Wow, it works!\n");
    //printf("Compile info: GCC %u.%u.%u\t", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    //printf("Compile date: %s -- %s\n", __DATE__, __TIME__);
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	rw::models::Device::Ptr device = wc->findDevice("UR1");

    rw::math::Metric<rw::math::Q>::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    __attribute__((unused)) rw::proximity::CollisionDetector *detector = new rw::proximity::CollisionDetector(wc,
        rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    rw::kinematics::State state = wc->getDefaultState();;

    rw::math::Q q_1(6, 0, -1.5, -0.298, -0.341, 0, 0);
    rw::math::Q q_2(6, 4.52282, -3.22411, 5.80915, 3.47969, -0.552745, 4.30143);
    //std::cout << q_2 << std::endl;
    /*ElipsisSampler e_sampler(q_1, q_2, (q_2 - q_1).norm2() * 1.1 );
    for(uint64_t i = 0; i < 10000; i++)
    {
        auto q_rand = e_sampler.doSample();
        std::cout << q_rand(0) << "," << q_rand(1)  << std::endl;
    }
    return 0;*/


    device->setQ(q_1, state);
    rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(detector, device, state);
    rw::math::QMetric::Ptr norm_metric = rw::pathplanning::PlannerUtil::normalizingInfinityMetric(device->getBounds());

    rw::pathplanning::QEdgeConstraint::Ptr edge_constraint = rw::pathplanning::QEdgeConstraint::make(constraint, norm_metric, 0.02);

    rw::pathplanning::PlannerConstraint p_constraint = rw::pathplanning::PlannerConstraint::make(constraint, edge_constraint);
    stat_constraint = &p_constraint;
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::QSampler::makeUniform(device),p_constraint.getQConstraintPtr());



    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(p_constraint, sampler, metric, 1, rwlibs::pathplanners::RRTPlanner::RRTConnect);
    rw::trajectory::QPath path;
    #define MAXTIME 1000.

    /*planner->query(q_1,q_2,path,MAXTIME);
    for(auto q_ : path)
    {
        std::cout << q_ << std::endl;
    }*/
    //return 0;
    RT_RRT_Star rt_rrt_star(q_1, q_2, p_constraint, sampler, metric, device);
    std::chrono::milliseconds time_to_solve{1000};
    for(uint64_t itterations = 0; rt_rrt_star.found_solution() == false || itterations < 2000; itterations++)
    {
        auto new_path = rt_rrt_star.find_next_path(time_to_solve);
        for(auto node : new_path)
        {
            std::cout << node->getValue() << std::endl;
        }
        //validate_path(new_path, p_constraint);

        std::cout << "End of itteration " << itterations << " Tree size: " << rt_rrt_star.get_size() << std::endl;
        rt_rrt_star.nodes_without_parents();
        rt_rrt_star.validate_tree_structure();
        std::cout << get_path_length(new_path) << std::endl;
        std::cout << rt_rrt_star.found_solution() << std::endl;
        if(rt_rrt_star.found_solution())
        {
            while(true)
            {
                auto tmp_q = sampler->sample();
                if(p_constraint.getQConstraint().inCollision(tmp_q)) continue;
                assert(!inCollision(p_constraint, tmp_q));
                rt_rrt_star.set_new_goal(tmp_q);
                std::cout << "New goal is " << tmp_q << std::endl;
                break;
            }
        }
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
