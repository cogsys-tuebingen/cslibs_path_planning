#ifndef DYNAMIC_STEERING_NEIGHBORHOOD_HPP
#define DYNAMIC_STEERING_NEIGHBORHOOD_HPP

#include <cslibs_path_planning/generic/NonHolonomicNeighborhood.hpp>
#include <cslibs_path_planning/generic/SteeringNode.hpp>
#include <cslibs_path_planning/generic/Common.hpp>

namespace lib_path
{

struct DynamicSteeringNeighborhood : public NeighborhoodBase
{
    template <class PointT>
    struct NodeHolder {
        typedef SteeringNode<PointT> NodeType;
    };

    static double resolution;
    static bool reversed;
    static double distance_step;
    static double distance_step_pixel;

    static int MAX_STEER_ANGLE;
    static double STEER_DELTA;
    static double LA;

    static bool allow_forward;
    static bool allow_backward;

    static double goal_dist_threshold;
    static double goal_angle_threshold;

    static unsigned int steer_steps;

    static double setResolution(double res) {
        resolution = res;
        distance_step_pixel = distance_step / resolution;
        return res;
    }

    template <class NodeType>
    static bool isGoal(const NodeType* goal, const NodeType* reference) {
        double delta_rot = std::atan2(std::sin(goal->theta - reference->theta),
                                      std::cos(goal->theta - reference->theta));
        if(std::abs(delta_rot) > goal_angle_threshold) {
            return false;
        }

        double eff_gx = goal->x;
        double eff_gy = goal->y;

//        if(reversed) {
//            if(reference->custom > 0) {
//                return false;
//            }

//            // if we begin at the goal and go backwards, we add a special straight segment at the start
//            if(!reference->forward) {
//                double theta = goal->theta;
//                double dx = -std::cos(theta) * LA / resolution;
//                double dy = -std::sin(theta) * LA / resolution;

//                eff_gx += dx;
//                eff_gy += dy;
//            }
//        }

//        int delta = 4 * distance_step_pixel;
//        if(std::abs(eff_gx - reference->x) > delta ||
//                std::abs(eff_gy - reference->y) > delta) {
//            return false;
//        }

        double cell_dist = hypot(eff_gx - reference->x, eff_gy - reference->y);
        double dist = cell_dist * resolution;


        // euclidean distance
        if(dist < goal_dist_threshold) {
            return true;
        }


        // check, if goal is near the segment through reference an its predecessor
//        if(reference->prev) {
//            Eigen::Vector2d p (reference->prev->x, reference->prev->y);
//            Eigen::Vector2d r (reference->x, reference->y);
//            Eigen::Vector2d g (eff_gx, eff_gy);

//            p *=  resolution;
//            r *=  resolution;
//            g *=  resolution;

//            // calculate distance of goal to the line segment through current and prev
//            //  if the projection of goal falls onto the segment, check if the distance is small enough
//            double line_distance;
//            double l2 = (p-r).squaredNorm();
//            if(l2 <= 0.0001) { // v ~= w
//                line_distance = (g - p).norm();
//            } else {
//                double t = (g - p).dot(r - p) / l2;

//                t = std::max(0.0, std::min(1.0, t));

//                Eigen::Vector2d project = p + t * (r-p);

//                line_distance = (g-project).norm();
//            }
//            return line_distance < 0.1;
//        }

        return false;
    }

    template <class NodeType>
    static double advance(const NodeType* reference, SearchOptions& so, int i, int step, double& x_, double& y_, double& theta_, bool& forward_, int& steering_angle_, char& custom, double map_rotation) {
        forward_ = (i < 3) ;
        bool initial = reference->depth < 1;

        if(initial &&  reference->forward != forward_) {
            // initially keep the last direction
            return -1;
        }


        int max_steer_angle_for_turn = 0;

        if(reference->custom > 0) {
            // if custom flag is positive, we need to go straight
            if(std::abs(reference->steering_angle) > max_steer_angle_for_turn) {
                return -1;
            }
        }
        if(!reversed) {
            if(initial) {
                // if we begin at the start and go backwards, we add a special straight segment here
                if (i > 3) {
                    return -1;
                }
                if (i == 3) {
                    if(step != 0) {
                        return -1;
                    }

                    forward_ = false;

                    double ds = -LA;
                    double ds_map = ds / resolution;
                    double dx = ds_map * std::cos(reference->theta);
                    double dy = ds_map * std::sin(reference->theta);

                    steering_angle_ = 0;
                    custom = 0;

                    x_ = reference->x + dx;
                    y_ = reference->y + dy;
                    theta_ = reference->theta;
                    return std::abs(ds_map) * so.penalty_backward;
                }
            }

        }

        double cost = distance_step_pixel;

        int delta = STEER_DELTA * step;

        int steering_angle;
        switch(i) {
        default:
        case 0: case 3: // keep angle
            steering_angle = reference->steering_angle;
            if(step > 0) {
                // only forward is done for step 0
                return -1;
            }
            break;
        case 1: case 4: // right
            if(step == 0) {
                // only forward is done for step 0
                return -1;
            }
            if(reference->steering_angle <= -MAX_STEER_ANGLE) {
                return -1;
            }
            steering_angle = reference->steering_angle - delta;
            break;
        case 2: case 5: // left
            if(step == 0) {
                // only forward is done for step 0
                return -1;
            }
            if(reference->steering_angle >= MAX_STEER_ANGLE) {
                return -1;
            }
            steering_angle = reference->steering_angle + delta;
            break;
        }

        if(steering_angle > MAX_STEER_ANGLE) {
            steering_angle = MAX_STEER_ANGLE;
        } else if(steering_angle < -MAX_STEER_ANGLE) {
            steering_angle = -MAX_STEER_ANGLE;
        }

        //        cost *= std::abs(steering_angle) / MAX_STEER_ANGLE * 1.1;

        // check driving direction
        double dir = forward_ ? 1.0 : -1.0;
        if(reversed) {
            dir *= -1.0;
        }

//        if(forward_) {
//            return -1;
//        }
        steering_angle_ = steering_angle;

        double r_world = LA / std::tan(steering_angle / 180. * M_PI);
        double ds_world = distance_step * dir;
        double ds_map = distance_step_pixel * dir;

        double r_map = r_world / resolution;

        //if(allow_forward && allow_backward) {

            bool direction_switch = (reference->forward != forward_) && !initial;
            if(direction_switch) {
                if(!allow_forward || !allow_backward) {
                    return -1;
                }

                if(reference->custom > 0) {
                    return -1;
                }
                // only allow to drive straight, if direction changes!
                if(std::abs(steering_angle) > max_steer_angle_for_turn) {
                    return -1;
                }


                int straight_dir_switch = 1;//std::round(2.0 / distance_step);
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

                    if(std::abs(test->steering_angle) > max_steer_angle_for_turn) {
                        return -1;
                    }
                }
                custom = straight_dir_switch - 1;

                cost *= so.penalty_turn;

            } else {
                if(reference->custom > 0) {
                    custom = reference->custom - 1;
                } else {
                    custom = 0;
                }
            }
        //}


        double dx, dy;
        double dtheta;
        if(steering_angle == 0) {
            dtheta = 0;
            dx = ds_map * std::cos(reference->theta);
            dy = ds_map * std::sin(reference->theta);

        } else {
            dtheta = ds_world / r_world;
            double t = reference->theta + dtheta;
            dx = r_map * (std::sin(t) - std::sin(reference->theta));
            dy = r_map * (-std::cos(t) + std::cos(reference->theta));
        }

        x_ = reference->x + dx;
        y_ = reference->y + dy;
        theta_ = MathHelper::AngleClamp(reference->theta + dtheta);

        if(!forward_) {
            // penalize driving backwards
            cost *= so.penalty_backward;
        };

        return cost * 1;
    }

    template <class T, class Map, class NodeType>
    static void iterateFreeNeighbors(T& algo, Map& map, NodeType* reference) {
        double map_rotation = map.getMap()->getRotation();

        unsigned start = allow_forward ? 0 : 3;
        unsigned end = allow_backward ? 6 : 3;

        for(unsigned i = start; i < end; ++i) {
            for(unsigned step = 0; step <= steer_steps; ++step) {
                double to_x,to_y, to_theta;
                bool forward;
                char custom;
                int steering_angle;
                double cost = advance(reference, algo.search_options, i, step, to_x,to_y,to_theta,forward,steering_angle,custom, map_rotation);

                if(cost < 0) {
                    continue;
                }

                if(map.contains(to_x, to_y)) {
//                    bool free = !map.isOccupied(reference->x,reference->y, to_x,to_y);
                    bool free_or_unknown = map.isFreeOrUnknown(reference->x,reference->y, to_x,to_y);
                    bool can_be_used = free_or_unknown;// free_or_unknown;// free || (free_or_unknown && forward);
                    if(can_be_used) {
                        NodeType* n = map.lookup(to_x, to_y, to_theta, steering_angle, forward);

                        if(n == NULL/* || map.isOccupied(n)*/) {
                            continue;
                        }

                        if(algo.processNeighbor(reference, n, cost) == PR_ADDED_TO_OPEN_LIST)  {
                            n->custom  = custom;
                            n->depth = reference->depth + 1;

                            n->steering_angle = steering_angle;

                            n->x = to_x;
                            n->y = to_y;
                            n->theta = to_theta;
                            n->forward = forward;
                        }

//                        // if we already have maximum steering, the following steps can be skipped
//                        if(steering_angle >= MAX_STEER_ANGLE) {
//                            step = steer_steps;
//                        } else if(steering_angle <= -MAX_STEER_ANGLE) {
//                            step = steer_steps;
//                        }

                    }
                }
            }
        }
    }
};

}

#endif // DYNAMIC_STEERING_NEIGHBORHOOD_HPP
