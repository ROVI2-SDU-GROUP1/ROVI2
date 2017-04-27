#pragma once
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>
#include <chrono>
#include <queue>




/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

 template <class X>
 class RT_RRT_Tree;


template <class T>
class RT_RRTNode;


template <class X>
class RRTNode
{
public:
    typedef RRTNode<X> node_type;
    typedef X value_type;

    node_type* getParent() const { return _parent; }
    const value_type& getValue() const { return _value; }

private:
    friend class RT_RRT_Tree<X>;
    friend class RT_RRTNode<X>;

    RRTNode(const value_type& value, node_type* parent) :
        _value(value),
        _parent(parent)
    {}

    value_type _value;
    node_type* _parent;

};




template <class T>
class RT_RRTNode : public RRTNode<T>
{
    friend class RT_RRT_Tree<T>;
    public:
        RT_RRTNode(const T& value, RT_RRTNode<T>* parent)
            : RRTNode<T>(value, parent) {}
            
        RT_RRTNode()
            : RRTNode<T>(rw::math::Q(6,0,0,0,0,0,0), nullptr) {}

        double get_cost()               { return cost; }
        double get_heuristic(rw::math::Q) { return 0; }
        void set_cost(double &_cost)    { this->cost = _cost; }
        void set_cost(double _cost)    { this->cost = _cost; }
        void recompute_cost();
    private:
        double cost;
};
