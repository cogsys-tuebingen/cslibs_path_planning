/*
 * SearchAlgorithm.hpp
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef SEARCH_ALGORITHM_H
#define SEARCH_ALGORITHM_H

/// COMPONENT
#include "Common.hpp"
#include "Heuristics.hpp"

/// PROJECT
#include <utils_generic/Intermission.hpp>
#include "../common/Path.h"

/// SYSTEM
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <stdexcept>

namespace lib_path
{

/**
 * @brief The GenericParameter struct combines types to be used by the search algorithm
 */
template <class PointT,
          class HeuristicT,
          class MapT,
          class NeighborhoodT,
          class AnalyticExpansionT,
          template <class> class OpenNodesPolicy,
          template <class> class MapManagerPolicy,
          int INTERMISSION_N_STEPS = 0,
          int INTERMISSION_START_STEPS = 0
          >
struct GenericParameter
{
    typedef PointT PointType;
    typedef HeuristicT HeuristicType;
    typedef MapT MapType;
    typedef NeighborhoodT NeighborhoodType;
    typedef AnalyticExpansionT AnalyticExpansionType;

private:
    typedef typename HeuristicType::template NodeHolder<PointT>::NodeType HeuristicNodeType;
    typedef typename NeighborhoodType::template NodeHolder<HeuristicNodeType>::NodeType NeighborhoodNodeType;
    typedef typename MapManagerPolicy<NeighborhoodNodeType>::template NodeHolder<NeighborhoodNodeType>::NodeType MapNodeType;

public:
    typedef MapNodeType NodeType;

    typedef MapManagerPolicy<NodeType> MapManager;
    typedef OpenNodesPolicy<NodeType> OpenNodesManager;

    enum { INTERMISSION_STEPS = INTERMISSION_N_STEPS, INTERMISSION_START = INTERMISSION_START_STEPS};
};


/**
 * @brief The GenericSearchAlgorithm class does all the common work
 */
template <class Param>
class GenericSearchAlgorithm
{
public:
    enum { INTERMISSION_N_STEPS = Param::INTERMISSION_STEPS };
    enum { INTERMISSION_START_STEPS = Param::INTERMISSION_START };

    typedef typename Param::OpenNodesManager OpenNodesManager;
    typedef typename Param::PointType PointT;
    typedef typename Param::NodeType NodeT;
    typedef typename Param::HeuristicType Heuristic;
    typedef typename Param::NeighborhoodType NeighborhoodType;
    typedef typename Param::MapManager MapManager;
    typedef typename Param::AnalyticExpansionType AnalyticExpansionType;

    typedef typename MapManager::MapT MapT;

    typedef std::vector<NodeT> PathT;

public:
    struct StartOutOfMapException : public std::logic_error
    {
        StartOutOfMapException()
            : std::logic_error("start cell is outside the map")
        { }
    };
    struct GoalOutOfMapException : public std::logic_error
    {
        GoalOutOfMapException()
            : std::logic_error("goal cell is outside the map")
        { }
    };
    struct StartNotFreeException : public std::logic_error
    {
        StartNotFreeException()
            : std::logic_error("start cell is not free")
        { }
    };
    struct GoalNotFreeException : public std::logic_error
    {
        GoalNotFreeException()
            : std::logic_error("goal cell is not free")
        { }
    };
    struct StartAndGoalNotFreeException : public std::logic_error
    {
        StartAndGoalNotFreeException()
            : std::logic_error("start and goal cell are not free")
        { }
    };

public:
    GenericSearchAlgorithm()
        : start(NULL), goal(NULL), has_cost_(false), expansions(0), multi_expansions(0), updates(0)
    {}

    /**
     * @brief empty
     * @return an empty path
     */
    static const PathT empty() {
        return PathT();
    }

    virtual void setMap(const MapT* map) {
        map_.setMap(map);
        NeighborhoodType::setResolution(map->getResolution());
    }

    virtual void setCostFunction(bool is_cost_function) {
        has_cost_ = is_cost_function;
    }

    virtual void setStart(const PointT& from) {
        assert(map_.getMap() != NULL);
        start = map_.lookup(from);
    }

    virtual void setGoal(const PointT& to) {
        assert(map_.getMap() != NULL);
        goal = map_.lookup(to);
    }

    NodeT* getStart() const
    {
        return start;
    }


    NodeT* getGoal() const
    {
        return goal;
    }


    OpenNodesManager& getOpenList()
    {
        return open;
    }

    MapManager& getMapManager()
    {
        return map_;
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to) {
        if(NeighborhoodType::reversed) {
            PathT path = findPathImp<generic::NoIntermission>(to, from, boost::function<void()>());
            std::reverse(path.begin(), path.end());
            return path;

        } else {
            return findPathImp<generic::NoIntermission>(from, to, boost::function<void()>());
        }
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @param intermission is a callback that is called every INTERMISSION_N_STEPS steps
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        if(NeighborhoodType::reversed) {
            PathT path = findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS> >(to, from, intermission);
            std::reverse(path.begin(), path.end());
            return path;

        } else {
            return findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS> >(from, to, intermission);
        }
    }

protected:
    template <class Intermission>
    PathT findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        expansions = 0;
        multi_expansions = 0;
        updates = 0;

        open.clear();

        // initialize start and goal nodes
        try {
            start = map_.lookup(from);
        } catch(const typename MapManager::OutsideMapException& e) {
            throw StartOutOfMapException();
        }

        try {
            goal = map_.lookup(to);
        } catch(const typename MapManager::OutsideMapException& e) {
            throw GoalOutOfMapException();
        }


        Heuristic::setMap(map_.getMap(), *goal);

        // search can be aborted, if either one is occupied
        bool start_blocked = map_.isOccupied(start);
        bool goal_blocked = map_.isOccupied(goal);

        if(start_blocked && !goal_blocked) {
            throw StartNotFreeException();
        } else if(goal_blocked && !start_blocked) {
            throw GoalNotFreeException();
        } else if(start_blocked && goal_blocked) {
            throw StartAndGoalNotFreeException();
        }

        // else init the border cases
        NodeT::init(*start, from);
        NodeT::init(*goal, to);

        // put start node in open data structure
        start->distance = 0;
        start->mark(NodeT::MARK_OPEN);


        start->theta = from.theta;
        goal->theta = to.theta;

        open.add(start);

        Intermission::call(intermission);

        // iterate until no more points can be looked at
        while(!open.empty()) {
            boost::this_thread::interruption_point();

            // mark the next node closed
            NodeT* current = open.next();
            current->unMark(NodeT::MARK_OPEN);
            current->mark(NodeT::MARK_CLOSED);

            Heuristic::compute(current, goal, map_.getMap()->getResolution());

            expansions ++;
            if(current->isMarked(NodeT::MARK_EXPANDED)) {
                ++multi_expansions;
            }
            current->mark(NodeT::MARK_EXPANDED);

            if(AnalyticExpansionType::canExpand(current, goal, map_.getMap())) {
                std::cout << "found an expansion: theta=" << current->theta << ", goal=" << goal->theta << std::endl;

                PathT expansion;
                AnalyticExpansionType::getPath(&expansion);

                current->theta = expansion.begin()->theta;

                PathT path = backtrack(start, current);
                path += expansion;
                return path;
            }

            // if we are close enough to the goal (policy decides), we can return
            if(NeighborhoodType::isNearEnough(goal, current)) {
                if(goal != current) {
                    // close, but not the real goal -> change prev pointer
                    if(current->prev) {
                        goal->prev = current->prev;
                    } else {
                        goal->prev = current;
                    }
                    std::cout << "near goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;
                }
                std::cout << "found goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;

                // generate the path
                return backtrack(start, current);
            }

            // look at every free neighbor
            NeighborhoodType::iterateFreeNeighbors(*this, map_, current);

            // intermission, if used
            Intermission::call(intermission);
        }

        std::cout << "done, no path found" << std::endl;

        // default: empty path
        return empty();
    }

public:
    double getCost(NodeT* node) {
        if(has_cost_) {
            double c = map_.getValue(node->x, node->y) / 254.0 * 2.0;
            if(c < 0) {
                throw std::runtime_error("negative costs are not allowed");
            }
            return c;
        } else {
            return 0;
        }
    }

    NeighborhoodBase::ProcessingResult
    processNeighbor(NodeT* current, NodeT* neighbor, double delta) {
        neighbor->mark(NodeT::MARK_WATCHED);

        double res = map_.getResolution();
        double cost = delta * res + getCost(neighbor);
        double distance = current->distance + cost;
        bool closer = distance < neighbor->distance;

        bool inOpenList = neighbor->isMarked(NodeT::MARK_OPEN);
        bool inClosedList = neighbor->isMarked(NodeT::MARK_CLOSED);

        if(inClosedList && !closer) {
            return NeighborhoodBase::PR_CLOSED;
        }

        if(!inOpenList || closer) {
            assert(neighbor != current);
            if(inOpenList) {
                //                open.remove(neighbor);
                //                neighbor->unMark(NodeT::MARK_OPEN);
                updates++;
            }


            neighbor->prev = current;

            neighbor->distance = distance;

            Heuristic::compute(neighbor, goal, res);

            neighbor->mark(NodeT::MARK_OPEN);

            open.add(neighbor);

            return NeighborhoodBase::PR_ADDED_TO_OPEN_LIST;
        }

        return NeighborhoodBase::PR_IGNORED;
    }

    PathT backtrack(NodeT* start, NodeT* goal) {
        PathT path;

        NodeT* current = goal;
        path.push_back(*goal);

        while(current != start){
            current = dynamic_cast<NodeT*>(current->prev);
            assert(current != NULL);
            assert(current != current->prev);
            path.push_back(*current);
        };

        std::reverse(path.begin(), path.end());

        return path;
    }

protected:
    OpenNodesManager open;
    MapManager map_;
    NodeT* start;
    NodeT* goal;

    bool has_cost_;

    int expansions;
    int multi_expansions;
    int updates;
};

template <class Param>
class GenericDynSearchAlgorithm:
        public GenericSearchAlgorithm<Param>
{
private:
    void TopKey(double &key1, double &key2){
        if(open.empty()){
            key1 = INFINITY;
            key2 = INFINITY;
        }else{
            NodeT* Top = open.top();
            Top->getKey(key1,key2);
        }
    }

    bool less_keys(double &left_key1, double &left_key2, double &right_key1, double &right_key2){
        if(left_key1<right_key1){
            return true;
        }else if(left_key1 == right_key1){
            if(left_key2<right_key2){
                return true;
            }else{
                return false;
            }
        }
        return false;
    }

    bool continue_while(){
        double topKey1,topKey2,startKey1,startKey2;

        TopKey(topKey1,topKey2);
        start->CalcKey(startKey1,startKey2);

        return less_keys(topKey1,topKey2,startKey1,startKey2) || start->rhs == start->distance;
    }
    void findPathImp(){
        double delta,dist;
        double res = map_.getResolution();
        while(continue_while()){
            NodeT* current = open.next();
            current->unMark(NodeT::MARK_OPEN);
            current->mark(NodeT::MARK_CLOSED);
            if(current->distance > current->rhs){
                current->distance = current->rhs;
                NeighborhoodType::iterateFreeNeighbors(*this, map_, current);
            }else{
                current->distance = INFINITY;
                delta = NeighborhoodType::getDelta(map_, dynamic_cast<NodeT*>(current->prev), current);
                dist = current->prev->distance + delta * res;
                UpdateVertex(current,dist);
                NeighborhoodType::iterateFreeNeighbors(*this, map_, current);
            }
        }
    }
public:

    typedef GenericSearchAlgorithm<Param> GenAlg;

    typedef typename GenAlg::MapManager MapManager;
    typedef typename GenAlg::Heuristic Heuristic;
    typedef typename GenAlg::NeighborhoodType NeighborhoodType;
    typedef typename GenAlg::PathT PathT;
    typedef typename GenAlg::NodeT NodeT;
    typedef typename GenAlg::PointT PointT;

    using GenAlg::start;
    using GenAlg::goal;
    using GenAlg::map_;
    using GenAlg::open;
    using GenAlg::empty;

    GenericDynSearchAlgorithm():GenAlg(){}

    void init(const PointT& from, const PointT& to) {
        //init von DSTAR
        open.clear();
        start = map_.lookup(from);
        goal = map_.lookup(to);

        NodeT::init(*start, from);
        NodeT::init(*goal, to);

        goal->rhs = 0;

        Heuristic::compute(goal, start, map_.getMap()->getResolution());
        Heuristic::compute(start, start, map_.getMap()->getResolution());

        goal->CalcKey();

        goal->mark(NodeT::MARK_OPEN);

        open.add(goal);
    }

    void UpdateVertex(NodeT* neighbor, double dist){
        if(neighbor!=goal){
           neighbor->rhs = dist;//only one antecesor
        }
        if (neighbor->isMarked(NodeT::MARK_OPEN)){
            open.remove(neighbor);
        }
        if(neighbor->distance != neighbor->rhs){
            neighbor->CalcKey();
            neighbor->mark(NodeT::MARK_OPEN);
            open.add(neighbor);
        }
    }

    NeighborhoodBase::ProcessingResult
    processNeighbor(NodeT* current, NodeT* neighbor, double delta) {

        neighbor->mark(NodeT::MARK_WATCHED);

        double res = map_.getResolution();
        //double cost = this->getCost(neighbor);//Costmap
        double distance = current->distance + delta * res;
        bool closer = distance < neighbor->distance;

        bool inOpenList = neighbor->isMarked(NodeT::MARK_OPEN);
        bool inClosedList = neighbor->isMarked(NodeT::MARK_CLOSED);

        if(inClosedList && !closer) {
            return NeighborhoodBase::PR_CLOSED;
        }

        if(!inOpenList || closer) {
            neighbor->prev = current;
            Heuristic::compute(neighbor, start, res);
            UpdateVertex(neighbor,distance);
            return NeighborhoodBase::PR_ADDED_TO_OPEN_LIST;
        }

        return NeighborhoodBase::PR_IGNORED;
    }

    PathT findPath(){
        PathT path;

        findPathImp();

        if(start->prev){
            NodeT* current = start;
            path.push_back(*current);
            while(current!=goal){
                current = dynamic_cast<NodeT*>(current->prev);
                path.push_back(*current);
            }
        }
        return path;
    }
};

}

#endif // SEARCH_ALGORITHM_H
