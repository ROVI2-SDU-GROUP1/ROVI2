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
#include <boost/foreach.hpp>

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
    //typedef bool is_kdtree_distance;
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
        : index(indexParams)
        {
            add(value, 0);
        }

        node_type& getRoot() const { return *_nodes[0]; }
        node_type& getLast() const { return *_nodes.back(); }

        void add(const value_type& value, node_type* parent)
        {
            _nodes.push_back(new node_type(value, parent));
            flann::Matrix< RT_RRTNode<X> > new_points(_nodes.back(), 1, 1);
            this->index.addPoints(new_points);
        }

        typedef typename std::vector<node_type*>::const_iterator const_iterator;

        std::pair<const_iterator, const_iterator>
        getNodes() const
        { return std::make_pair(_nodes.begin(), _nodes.end()); }

        ~RT_RRT_Tree()
        {
            BOOST_FOREACH(node_type* node, _nodes) {
                delete node;
            }
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

    public:
        std::vector<node_type*> _nodes;
        flann::Index<RT_RRTNode_Flann<X> > index;
        flann::KDTreeIndexParams indexParams;

    private:
        RT_RRT_Tree(const RT_RRT_Tree&);
        RT_RRT_Tree& operator=(const RT_RRT_Tree&);
    };

    /*\}*/
}} // end namespaces
