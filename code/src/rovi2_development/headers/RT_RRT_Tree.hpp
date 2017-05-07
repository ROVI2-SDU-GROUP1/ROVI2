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
#pragma once
/**
   @file RRTTree.hpp
*/

#include "RT_RRTNode.hpp"
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>
#include <cstdlib>
typedef RT_RRTNode<rw::math::Q> RT_Node;

void validate_path(std::vector<RT_Node *> &path, const rw::pathplanning::PlannerConstraint& constraint);


bool operator>(const rw::math::Q &lhs, const rw::math::Q &rhs)
{
    return lhs.norm2() > rhs.norm2();
}

bool operator>=(const rw::math::Q &lhs, const rw::math::Q &rhs)
{
    return lhs.norm2() >= rhs.norm2();
}



#include <flann/flann.hpp>

template <class X>
struct RT_RRTNode_Flann
{
    typedef RT_RRTNode<X> ElementType;
    typedef bool is_kdtree_distance;
    typedef typename flann::Accumulator<double>::Type ResultType;
    template <typename Iterator1, typename Iterator2>
    ResultType operator() (Iterator1 a, Iterator2 b, size_t size,
        ResultType /*worst_dist*/ = -1) const
    {
        ResultType result = ResultType();
        ResultType diff;
        for(size_t i = 0; i < size; i++)
        {
            diff = ( (*a++).getValue() - (*b++).getValue() ).norm2();
            result += diff * diff;
        }
        return result;
    }

    template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int) const
    {
        //a.rgesdbrtvfed();
        return (a-b)*(a-b);
    }
};

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Tree data type for RRT based planners.
    */
    template <class X>
    class RT_RRT_Tree
    {
    public:
        typedef X value_type;
        typedef RT_RRTNode<X> node_type;

        RT_RRT_Tree(const value_type& value)
        //: indexParams(), index(this->indexParams, RT_RRTNode_Flann<X>() )
        {
            //index.buildIndex();
            add(value, 0);
        }

        node_type& getRoot() const { return *_nodes[0]; }
        node_type& getLast() const { return *_nodes.back(); }
        node_type *getLastPtr() const { return _nodes.back(); }

        void clear_unsafe()
        {
            this->_nodes.clear();
        }

        void clear()
        {
            BOOST_FOREACH(node_type* node, _nodes) {
                delete node;
            }
            this->_nodes.clear();

        }

        node_type *get_random_node()
        {
            return this->_nodes[rand() % this->_nodes.size()]; //This is biased, but who gives a fuck?
        }

        void add(const value_type& value, node_type* parent)
        {
            auto tmp_node = new node_type(value, parent);
            //printf("Adding node with address %p and parent address %p to the tree..!\n", tmp_node, tmp_node->getParent());
            //validate_path(this->_nodes);

            _nodes.push_back(tmp_node);
            //flann::Matrix< RT_RRTNode<X> > new_points(_nodes.back(), 1, 1);
            //this->index.addPoints(new_points);
            //validate_path(this->_nodes);
        }

        typedef typename std::vector<node_type*>::const_iterator const_iterator;

        void add(node_type* node)
        {
            _nodes.push_back(node);
        }
        std::pair<const_iterator, const_iterator>
        getNodes() const
        { return std::make_pair(_nodes.begin(), _nodes.end()); }

        ~RT_RRT_Tree()
        {
            this->clear();
        }

        size_t size() const { return _nodes.size(); }

        // If speed is important, you could add a version that retrieves
        // pointers to the values instead.
        static
        void getRootPath(node_type& last, std::vector<value_type>& path)
        {
            node_type* node = &last;
            while (node) {
                path.push_back(node->getValue());
                node = node->getParent();
            }
        }

        std::vector<node_type *> get_n_nearest(rw::math::Q q, size_t n)
        {
            //std::cout << "test" << std::endl;
            node_type q_n(q, nullptr);
            std::sort(this->_nodes.begin(), this->_nodes.end(), q_n);
            if(n >= this->_nodes.size()) return this->_nodes;
            std::vector<node_type *> nearest;
            for(size_t i = 0; i < n; i++)
            {
                nearest.push_back(this->_nodes[i]);
            }
            return nearest;
        }

        std::vector<node_type *> get_nodes_within(rw::math::Q q, double dist)
        {
            node_type q_n(q, nullptr);
            std::sort(this->_nodes.begin(), this->_nodes.end(), q_n);
            //printf("sort in get_nodes_within done\n");
            std::vector<node_type *> nearest;
            while(nearest.size() < this->_nodes.size() )
            {
                if( (q - this->_nodes[nearest.size()]->getValue() ).norm2() > dist)
                {
                    break;
                }
                nearest.push_back(this->_nodes[nearest.size()]);
            }
            return nearest;
        }

        node_type *get_nearest(rw::math::Q q)
        {
            node_type n(q, nullptr);
            std::sort(this->_nodes.begin(), this->_nodes.end(), n);
            return this->_nodes[0];
        }


        /*flann::Index<RT_RRTNode_Flann<X> > *getFlannIndex()
        {   //We cheat a but, just return the index..
            return &this->index;
        }*/

    public:
        std::vector<node_type*> _nodes;
        //flann::KDTreeIndexParams indexParams;
        //flann::Index<RT_RRTNode_Flann<X> > index;

    private:
        RT_RRT_Tree(const RT_RRT_Tree&);
        RT_RRT_Tree& operator=(const RT_RRT_Tree&);
    };

    /*\}*/
}} // end namespaces
