/*
 * Common.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef COMMON_H
#define COMMON_H

/// PROJECT
#include <cslibs_path_planning/generic/Utils.hpp>

/// SYSTEM
#include <boost/noncopyable.hpp>
#include <vector>
#include <functional>
#include <math.h>
#include <assert.h>
#include <cstdlib>
#include <limits>

namespace lib_path
{
struct OutsideMapException : public std::exception
{
};

struct SearchOptions
{
    double penalty_backward = 2.5;
    double penalty_turn = 4.0;
    double oversearch_distance = 0.0;
};

/// INTERFACES

struct NeighborhoodBase {
    enum ProcessingResult {
        PR_ADDED_TO_OPEN_LIST, PR_IGNORED, PR_CLOSED
    };

    static double setResolution(double res) {return res;}
};

struct NoExpansion {
    static bool canExpand(...) {
        return false;
    }

    static void getPath(...) {
        assert(false);
    }
};

/**
 * @brief The Node struct augments a class by adding more fields for bookkeeping
 */
template <class PointT>
struct Node : public PointT {
    typedef PointT PointType;
    typedef Node<PointT> NodeType;

    static const char MARK_NONE = 0;
    static const char MARK_OPEN = 1;
    static const char MARK_CLOSED = 2;
    static const char MARK_WATCHED = 4;
    static const char MARK_EXPANDED = 8;

    void mark(char mark) {
        marked |= mark;
    }
    void unMark(char mark) {
        marked &= ~mark;
    }

    bool isMarked(char mark) const {
        return (marked & mark) == mark;
    }

    virtual double getTotalCost() const {
        return distance;
    }

    template <class AnyPoint>
    static void init(Node<PointT> &memory, const AnyPoint& p) {
        memory.x = p.x;
        memory.y = p.y;
        memory.distance = INFINITY;
        memory.prev = NULL;
        memory.marked = MARK_NONE;
        memory.custom = 0;
    }

    template <typename V>
    static void init(Node<PointT> &memory, V x, V y) {
        memory.x = x;
        memory.y = y;
        memory.distance = INFINITY;
        memory.prev = NULL;
        memory.marked = MARK_NONE;
        memory.custom = 0;
    }

    double distance;
    char marked;
    char custom;
    Node* prev;
};



/**
 * @brief The OrientedNode struct augments a class
 */
template <class Node>
struct OrientedNode : public Node {
    typedef typename Node::PointType PointType;
    typedef OrientedNode<Node> NodeType;

    template <class AnyPoint>
    static void init(OrientedNode<Node> &memory, const AnyPoint& p) {
        Node::init(memory, p);
        memory.theta = p.theta;
    }

    template <typename V>
    static void init(OrientedNode<Node> &memory, V x, V y, double theta = 0.0) {
        Node::init(memory, x, y);
        memory.theta = theta;
    }
};



/**
 * @brief The DirectionalNode struct augments a class
 */
template <class Node>
struct DirectionalNode : public Node {
    typedef typename Node::PointType PointType;
    typedef DirectionalNode<Node> NodeType;

    bool forward;

    template <class AnyPoint>
    static void init(DirectionalNode<Node> &memory, const AnyPoint& p) {
        Node::init(memory, p);
        memory.forward = true;
    }

    template <typename V>
    static void init(DirectionalNode<Node> &memory, V x, V y, double theta = 0.0, bool forward = true) {
        Node::init(memory, x, y, theta);
        memory.forward = forward;
    }

    template <class AnyPoint, class Map>
    static std::vector<NodeType*> getPoses(Map& map, const AnyPoint& p)
    {
        std::vector<NodeType*> res;

        NodeType forward;
        NodeType::init(forward, p);
        forward.forward = true;
        auto* start_forward = map.lookup(forward);
        start_forward->forward = true;
        res.push_back(start_forward);

        NodeType reverse;
        NodeType::init(reverse, p);
        reverse.forward = false;
        auto* start_reverse = map.lookup(reverse);
        start_reverse->forward = false;
        res.push_back(start_reverse);

        return res;
    }
};



/**
 * @brief The CompareNode struct can be used to compare nodes
 */
template <class Node>
struct CompareNode : public std::binary_function<Node*, Node*, bool> {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->getTotalCost() > rhs->getTotalCost();
    }
};

/**
 * @brief The NodeTraits class has type information for Nodes
 */
template <typename Any>
class NodeTraits
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small testH(__typeof__(&Class::h)) ;
    template <typename Class> static Big testH(...);

    template <typename Class> static Small testF(__typeof__(&Class::forward)) ;
    template <typename Class> static Big testF(...);


public:
    enum { HasHeuristicField = sizeof(testH<Any>(0)) == sizeof(Small) };
    enum { HasForwardField = sizeof(testF<Any>(0)) == sizeof(Small) };
};

}
#endif // COMMON_H
