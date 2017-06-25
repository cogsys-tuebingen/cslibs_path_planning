#ifndef STEERINGNODE_HPP
#define STEERINGNODE_HPP

#include <cslibs_path_planning/generic/Common.hpp>

namespace lib_path
{

template <class Node>
struct SteeringNode : public Node {
    typedef typename Node::PointType PointType;
    typedef SteeringNode<Node> NodeType;

//    float theta;
    int steering_angle;
//    bool forward;
    int depth;

    template <class AnyPoint>
    static void init(SteeringNode<Node> &memory, const AnyPoint& p) {
        Node::init(memory, p);
        memory.theta = 0.0f;
        memory.steering_angle = 0.0f;
//        memory.forward = true;
        memory.depth = 0;
    }

    template <typename V>
    static void init(SteeringNode<Node> &memory, V x, V y, double theta = 0.0, int steering_angle = 0, bool forward = true) {
        Node::init(memory, x, y);
        memory.theta = theta;
        memory.steering_angle = steering_angle;
//        memory.forward = forward;
        memory.depth = 0;
    }
};

}
#endif // STEERINGNODE_HPP
