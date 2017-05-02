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
#include <iostream>



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






template <class T>
class RT_RRTNode
{
    friend class RT_RRT_Tree<T>;
    public:
        typedef RT_RRTNode<T> node_type;
        typedef T value_type;

        const value_type& getValue() const { return _value; }

        RT_RRTNode<T> *getParent() const { return this->_parent; }
        void setParent(RT_RRTNode<T> *p)
        {
            if(this->_parent)
            {
                this->_parent->remove_child(this);
            }
            this->_parent = p;
            if(this->_parent)
            {
                this->_parent->add_child(this);
            }

        }

        RT_RRTNode(const T& value, RT_RRTNode<T>* parent)
            : _value(value), _parent(parent), rewired(false)
            {
                if(this->_parent)
                {
                    this->_parent->add_child(this);
                }
             }
        RT_RRTNode<T>(int i)
        : _value(T(6,i,i,i,i,i,i)), _parent(nullptr) {}

        RT_RRTNode()
            : _value(rw::math::Q(6,0,0,0,0,0,0)), _parent(nullptr)
            {}

        double get_cost()
        {
            if(this->getParent() == nullptr)
                return 0;
            return this->_parent->get_cost() + (this->_parent->_value - this->_value).norm2();
        }
        double get_heuristic(RT_RRTNode<T> *goal) { return (this->getValue() - goal->getValue()).norm2(); }
        void set_cost(double _cost)    { this->cost = _cost; }
        void set_rewired(bool _rewired) { this->rewired = _rewired; }
        bool get_rewired() { return this->rewired; }
        void add_child(RT_RRTNode<T> *child)
        {
            childs.push_back(child);
        }

        void remove_child(RT_RRTNode<T> *child)
        {
            size_t i = 0;
            bool found = false;
            for(RT_RRTNode<T> *c : this->childs)
            {
                if(c == child)
                {
                    found = true;
                    break;
                }
                i++;
            }
            if(found)
            {
                this->childs.erase(this->childs.begin() + i);
            }
        }
        int get_child_count()
        {
            return this->childs.size();
        }
        bool operator()( const RT_RRTNode<T> *lhs, const RT_RRTNode<T> *rhs  )
        {
            //printf("comparing distance to %p from %p and %p\n", this, lhs, rhs );
            //std::cout << "this value: " << this->_value << " lhs value: " << lhs->_value << " rhs value: " <<  rhs->_value << std::endl;
            auto comp1 = lhs->_value - this->_value;
            auto comp2 = rhs->_value - this->_value;
            return comp1.norm2() < comp2.norm2();
            //printf("Exited compare\n");
        }
        /*explicit operator double() const
        {
            return this->_value.norm2();
        }*/
        std::vector<RT_RRTNode<T> *> &get_childs()
        {
            return this->childs;
        }

        void recompute_cost();
    private:
        double cost;
        std::vector<RT_RRTNode<T> *> childs;
        value_type _value;
        node_type* _parent;
        bool rewired = false;

};

template <class T>
bool operator>(const double &lhs, const RT_RRTNode<T> &rhs)
{
    return lhs > rhs.getValue().norm2();
}

template <class T>
bool operator<(const double &lhs, const RT_RRTNode<T> &rhs)
{
    return lhs < rhs.getValue().norm2();
}
