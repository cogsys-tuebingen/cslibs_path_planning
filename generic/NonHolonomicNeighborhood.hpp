/*
 * NonHolonomicNeighborhood.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef NON_HOLONOMIC_NEIGHBORHOOD_H
#define NON_HOLONOMIC_NEIGHBORHOOD_H

/// COMPONENT
#include "Common.hpp"
#include "MapManager.hpp"

/// PROJECT
#include <utils_general/MathHelper.h>

namespace lib_path
{

struct NonHolonomicNeighborhoodMoves
{
    enum AllowedMoves {
        FORWARD = 3,
        FORWARD_BACKWARD = 6,
        FORWARD_BACKWARD_HALFSTEPS = 10
    };
};

template  <int distance, int steerangle, int allowed_moves, class Imp>
struct NonHolonomicNeighborhoodBase : public NeighborhoodBase
{
    enum { DISTANCE = distance };
    enum { STEER_ANGLE = steerangle };
    enum { SIZE = allowed_moves };
    //    enum { SIZE = 10 };

    template <class PointT>
    struct NodeHolder {
        typedef OrientedNode<PointT> NodeType;
    };

    static constexpr double DELTA_THETA = STEER_ANGLE / 10.0 * M_PI / 180.0;

    static double resolution;
    static constexpr double distance_step = distance / 100.0;
    static double distance_step_pixel;

    static double setResolution(double res) {
        resolution = res;
        distance_step_pixel = distance_step / resolution;
        return res;
    }

    template <class NodeType>
    static double advance(NodeType* reference, int i, double& x_, double& y_, double& theta_, bool& forward_, bool initial) {
        double t;

        double cost = distance_step_pixel;

        // choose steering direction
        switch(i) {
        default:
        case 0: case 3: // straight
            t = reference->theta;
            break;
        case 1: case 4: // right
            t = reference->theta - DELTA_THETA;
            // penalize driving curves
            //  cost *= 1.1 ;
            break;
        case 2: case 5: // left
            t = reference->theta + DELTA_THETA;
            // penalize driving curves
            //  cost *= 1.1 ;
            break;

        case 6: case 8: // right
            t = reference->theta - DELTA_THETA/2;
            // penalize driving curves
            // cost *= 1.1 ;
            break;
        case 7: case 9: // left
            t = reference->theta + DELTA_THETA/2;
            // penalize driving curves
            //  cost *= 1.1 ;
            break;
        }

        // normalize the angle
        t = MathHelper::AngleClamp(t);

        // check driving direction
        forward_ = (i < 3);

        bool direction_switch = reference->forward != forward_ && !initial;

        if(direction_switch) {
            // only allow to drive straight, if direction changes!
            if((i%3) != 0) {
                return -1;
            }
        }


        if(forward_) {
            x_ = reference->x + std::cos(t) * distance_step_pixel;
            y_ = reference->y + std::sin(t) * distance_step_pixel;

        } else {
            x_ = reference->x - std::cos(t) * distance_step_pixel;
            y_ = reference->y - std::sin(t) * distance_step_pixel;

            // penalize driving backwards
            cost *= 1.5;
        }

        // penalize directional changes
        if(direction_switch/* && !forward_*/) {
            cost *= 4.0;
        }

        theta_ = t;

        return cost * 1;
    }

    template <class T, class Map, class NodeType>
    static void iterateFreeNeighbors(T& algo, Map& map, NodeType* reference) {
        bool initial = algo.getStart() == reference;

        for(unsigned i = 0; i < SIZE; ++i) {
            double to_x,to_y, to_theta;
            bool forward;
            double cost = advance(reference, i, to_x,to_y,to_theta,forward, initial);

            if(cost < 0) {
                continue;
            }

            if(map.contains(to_x, to_y)) {
                bool free = map.isFree(reference->x,reference->y, to_x,to_y);
                bool free_or_unknown = map.isFreeOrUnknown(reference->x,reference->y, to_x,to_y);
                bool can_be_used = free_or_unknown;// free || (free_or_unknown && forward);
                if(can_be_used) {
                    NodeType* n = map.lookup(to_x, to_y, to_theta, forward);

                    if(n == NULL/* || map.isOccupied(n)*/) {
                        continue;
                    }

                    if(algo.processNeighbor(reference, n, cost) == PR_ADDED_TO_OPEN_LIST)  {
                        n->x = to_x;
                        n->y = to_y;
                        n->theta = to_theta;
                        n->forward = forward;
                    }
                }
            }
        }
    }

    template <class NodeType>
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance_step_pixel / 2 &&
                std::abs(goal->y - reference->y) <= distance_step_pixel / 2 &&
                std::abs(MathHelper::AngleClamp(goal->theta - reference->theta)) < M_PI / 8;
    }
};

template  <int d, int s, int moves, class I>
double NonHolonomicNeighborhoodBase<d,s,moves,I>::resolution = 0;

template  <int d, int s, int moves, class I>
double NonHolonomicNeighborhoodBase<d,s,moves,I>::distance_step_pixel = 0;



template <int distance = 100, int steerangle = 10, int moves = NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD>
struct NonHolonomicNeighborhood
        : public NonHolonomicNeighborhoodBase<distance, steerangle, moves,
        NonHolonomicNeighborhood<distance, steerangle> >
{
};

}
#endif // NON_HOLONOMIC_NEIGHBORHOOD_H
