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
#include <cslibs_navigation_utilities/MathHelper.h>

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

template  <int distance, int steerangle, int allowed_moves, bool _reversed, int straight_dir_switch, class Imp>
struct NonHolonomicNeighborhoodBase : public NeighborhoodBase
{
    enum { DISTANCE = distance };
    enum { STEER_ANGLE = steerangle };
    enum { SIZE = allowed_moves };

    static constexpr bool reversed = _reversed;

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
    static double advance(NodeType* reference, int i, double& x_, double& y_, double& theta_, bool& forward_, char& custom, bool initial, double map_rotation) {
        double t;

        if(initial && (i != 0 && i != 3)) {
            return -1;
        }

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

        // apply map rotation
        t += map_rotation;

        // normalize the angle
        t = MathHelper::AngleClamp(t);

        // check driving direction
        forward_ = (i < 3) || (i >= 6 && i <= 7);

        if(reference->custom > 0) {
            // if custom flag is positive, reuse the last command
            if((i%3) != 0) {
                return -1;
            }
        }

        bool direction_switch = reference->forward != forward_ && !initial;

        if(direction_switch) {
            if(reference->custom > 0) {
                return -1;
            }

            int expected_i = forward_ ? 0 : 3;

            // only allow to drive straight, if direction changes!
            if(i != expected_i) {
                return -1;
            }

            double expected_theta = reference->theta;

            const NodeType* test = reference;
            for(int straight_parts = 0; straight_parts < straight_dir_switch - 1; ++straight_parts) {
                if(!test->prev) {
                    return -1;
                }

                test = dynamic_cast<const NodeType*>(test->prev);
                if(!test) {
                    throw std::logic_error("cannot cast prev");
                    return -1;
                }

                if(test->theta != expected_theta) {
                    return -1;
                }
            }

            custom = straight_dir_switch - 1;

        } else {
            if(reference->custom > 0) {
                custom = reference->custom - 1;
            } else {
                custom = 0;
            }
        }

        double dir = forward_ ? 1.0 : -1.0;
        if(reversed) {
            dir *= -1.0;
        }

        x_ = reference->x + dir * std::cos(t) * distance_step_pixel;
        y_ = reference->y + dir * std::sin(t) * distance_step_pixel;

        if(!forward_) {
            // penalize driving backwards
            cost *= 1.5;
        }

        // penalize directional changes
        if(direction_switch/* && !forward_*/) {
            cost *= 4.0;
        }

        theta_ = MathHelper::AngleClamp(t - map_rotation);

        return cost * 1;
    }

    template <class T, class Map, class NodeType>
    static void iterateFreeNeighbors(T& algo, Map& map, NodeType* reference) {
        bool initial = algo.getStart() == reference || algo.getGoal() == reference;

        double map_rotation = map.getMap()->getRotation();

        for(unsigned i = 0; i < SIZE; ++i) {
            double to_x,to_y, to_theta;
            bool forward;
            char custom;
            double cost = advance(reference, i, to_x,to_y,to_theta,forward,custom, initial, map_rotation);

            if(cost < 0) {
                continue;
            }

            if(map.contains(to_x, to_y)) {
                //                bool free = map.isFree(reference->x,reference->y, to_x,to_y);
                bool free_or_unknown = map.isFreeOrUnknown(reference->x,reference->y, to_x,to_y);
                bool can_be_used = free_or_unknown;// free || (free_or_unknown && forward);
                if(can_be_used) {
                    NodeType* n = map.lookup(to_x, to_y, to_theta, forward);

                    if(n == NULL/* || map.isOccupied(n)*/) {
                        continue;
                    }

                    if(algo.processNeighbor(reference, n, cost) == PR_ADDED_TO_OPEN_LIST)  {
                        n->custom  = custom;

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
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance_step_pixel / 2 &&
                std::abs(goal->y - reference->y) <= distance_step_pixel / 2 &&
                std::abs(MathHelper::AngleClamp(goal->theta - reference->theta)) < M_PI / 8;
    }
};

template  <int distance, int steerangle, int moves, bool reversed, int straight_dir_switch, class I>
double NonHolonomicNeighborhoodBase<distance, steerangle,moves,reversed,straight_dir_switch,I>::resolution = 0;

template  <int distance, int steerangle, int moves, bool reversed, int straight_dir_switch, class I>
double NonHolonomicNeighborhoodBase<distance, steerangle,moves,reversed,straight_dir_switch,I>::distance_step_pixel = 0;



template <int distance = 100, int steerangle = 10, int moves = NonHolonomicNeighborhoodMoves::FORWARD_BACKWARD, bool _reversed=false, int straight_dir_switch = 1>
struct NonHolonomicNeighborhood
        : public NonHolonomicNeighborhoodBase<distance, steerangle, moves, _reversed, straight_dir_switch,
        NonHolonomicNeighborhood<distance, steerangle> >
{
};

}
#endif // NON_HOLONOMIC_NEIGHBORHOOD_H
