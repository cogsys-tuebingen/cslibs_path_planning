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
#include <cslibs_path_planning/generic/Intermission.hpp>
#include <cslibs_path_planning/common/Path.h>

/// SYSTEM
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <stdexcept>
#include <queue>
#include <chrono>
#include <functional>

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

    typedef std::tuple<const NodeT*, double> GoalCandidate;

    struct PairDistance
    {
        bool operator() (const GoalCandidate& g1, const GoalCandidate& g2) {
            return std::get<1>(g1) > std::get<1>(g2);
        }
    };

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

public:
    GenericSearchAlgorithm()
        : heuristic_goal(NULL), has_cost_(false), expansions(0), multi_expansions(0), touched(0), updates(0), time_limit_(-1.0)
    {}

    virtual void setMap(const MapT* map) {
        map_.setMap(map);
        NeighborhoodType::setResolution(map->getResolution());
    }

    virtual void setCostFunction(bool is_cost_function) {
        has_cost_ = is_cost_function;
    }

    void setTimeLimit(double seconds) {
        time_limit_ = seconds;
    }

//    virtual void setStart(const PointT& from) {
//        assert(map_.getMap() != NULL);
//        start = map_.lookup(from);
//    }

//    virtual void setGoal(const PointT& to) {
//        assert(map_.getMap() != NULL);
//        goal = map_.lookup(to);
//    }

//    NodeT* getStart() const
//    {
//        return start;
//    }


//    NodeT* getGoal() const
//    {
//        return goal;
//    }


    OpenNodesManager& getOpenList()
    {
        return open;
    }

    MapManager& getMapManager()
    {
        return map_;
    }

    int getExpansions() const
    {
        return expansions;
    }
    int getMultiExpansions() const
    {
        return multi_expansions;
    }

    int getTouchedNodes() const
    {
        return touched;
    }

    void addGoalCandidate(const NodeT* current, double cost)
    {
        if(goal_candidates.empty()) {
            first_candidate_weight = current->getTotalCost();
        }
        goal_candidates.push(std::make_tuple(current, cost));
    }


    /**
     * @brief findPath searches a path between to points
     * @param from start pose
     * @param to goal pose
     * @param oversearch_distance (meters) specify for how long the search continues after a
     *          first candidate solution has been found. The search stops at the first node
     *          that has a distance to the start larger than the first candidate + oversearch_distance.
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to,
                           SearchOptions so = SearchOptions()) {
        search_options = so;

        if(NeighborhoodType::reversed) {
            PathT path = findPathPose<generic::NoIntermission>(to, from, boost::function<void()>());
            std::reverse(path.begin(), path.end());
            return path;

        } else {
            return findPathPose<generic::NoIntermission>(from, to, boost::function<void()>());
        }
    }

    /**
     * @brief findPath searches a path between to points
     * @param from start pose
     * @param to goal pose
     * @param intermission is a callback that is called every INTERMISSION_N_STEPS steps     *
     * @param oversearch_distance (meters) specify for how long the search continues after a
     *          first candidate solution has been found. The search stops at the first node
     *          that has a distance to the start larger than the first candidate + oversearch_distance.
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to,
                           boost::function<void()> intermission,
                           SearchOptions so = SearchOptions())
    {
        search_options = so;
        if(NeighborhoodType::reversed) {
            PathT path = findPathPose<
                    generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS>
                    >
                    (to, from, intermission);
            reverse(path);
            return path;

        } else {
            return findPathPose<
                    generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS>
                    >
                    (from, to, intermission);
        }
    }

    template <class GoalTest>
    PathT findPath(const PointT& from, const GoalTest& goal_test,
                   SearchOptions so = SearchOptions())
    {
        search_options = so;
        PathT path = findPathMapPose<GoalTest, generic::NoIntermission>
                (from, goal_test, boost::function<void()>());

        if(NeighborhoodType::reversed) {
            reverse(path);
        }
        return path;
    }

    template <class GoalTest>
    PathT findPath(const PointT& from, const GoalTest& goal_test,
                   boost::function<void()> intermission,
                   SearchOptions so = SearchOptions())
    {
        search_options = so;
        PathT path = findPathMapPose<GoalTest, generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS>
                >
                (from, goal_test, intermission);

        if(NeighborhoodType::reversed) {
            reverse(path);
        }
        return path;
    }

    template <class GoalTest>
    PathT findPathWithStartConfiguration(const NodeT& from, const GoalTest& goal_test,
                                         SearchOptions so = SearchOptions())
    {
        search_options = so;
        PathT path = findPathMapNode<GoalTest, generic::NoIntermission>
                (from, goal_test, boost::function<void()>());

        if(NeighborhoodType::reversed) {
            reverse(path);
        }
        return path;
    }

protected:
    template <typename NodeType>
    void reverse(std::vector<NodeType>& path, typename boost::enable_if_c<NodeTraits<NodeType>::HasForwardField, NodeType>::type* = 0)
    {
        std::reverse(path.begin(), path.end());
//        for(auto& pt : path) {
//            pt.forward = !pt.forward;
//        }
    }
    template <typename NodeType>
    void reverse(std::vector<NodeType>& path, typename boost::enable_if_c<!NodeTraits<NodeType>::HasForwardField, NodeType>::type* = 0)
    {
        std::reverse(path.begin(), path.end());
    }


protected:
    template <class SearchAlgo, class NodeType>
    struct PoseGoalTest
    {
        PoseGoalTest(SearchAlgo* self)
            : self_(self)
        {

        }

        bool terminate(const NodeType* node) const
        {
            return self_->hasPoseBeenReached(node);
        }

        SearchAlgo* self_;
    };

    template <class Intermission>
    PathT findPathPose(const PointT& from, const PointT& to,
                       boost::function<void()> intermission)
    {
        init();
        initStartPose(from);
        initGoalPose(to);

        typedef PoseGoalTest<GenericSearchAlgorithm<Param>, NodeT> GoalTest;

        return findPathImp<GoalTest, Intermission>
                (GoalTest(this), intermission);
    }

    template <class GoalTest, class Intermission>
    PathT findPathMapPose(const PointT& from, const GoalTest& goal_test,
                          boost::function<void()> intermission)
    {
        init();
        initStartPose(from);

        heuristic_goal = goal_test.getHeuristicGoal();

        return findPathImp<GoalTest, Intermission>
                (goal_test, intermission);
    }


    template <class GoalTest, class Intermission>
    PathT findPathMapNode(const NodeT& from, const GoalTest& goal_test,
                          boost::function<void()> intermission)
    {
        init();
        initStartNode(from);

        heuristic_goal = goal_test.getHeuristicGoal();

        return findPathImp<GoalTest, Intermission>
                (goal_test, intermission);
    }

    template <class GoalTest, class Intermission>
    PathT findPathImp(const GoalTest& goal_test,
                      boost::function<void()> intermission)
    {
        if(time_limit_ > 0.0) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::milliseconds duration_milli{(int64_t) (time_limit_ * 1e3)};
            stamp_abort_ = now + duration_milli;
        }

        // iterate until no more points can be looked at
        while(!open.empty()) {
            boost::this_thread::interruption_point();

            if(time_limit_ > 0.0) {
                auto now = std::chrono::high_resolution_clock::now();
                if(now >= stamp_abort_) {
                    std::cerr << "abort, search took longer than " << time_limit_ << " seconds" << std::endl;
                    break;
                }
            }


            // mark the next node closed
            NodeT* current = open.next();

            current->unMark(NodeT::MARK_OPEN);
            current->mark(NodeT::MARK_CLOSED);

            expansions ++;
            if(current->isMarked(NodeT::MARK_EXPANDED)) {
                ++multi_expansions;
            }
            current->mark(NodeT::MARK_EXPANDED);

            if(!goal.empty()) {
                Heuristic::compute(current, *goal.begin(), map_.getMap()->getResolution());

            } else if(heuristic_goal) {
                Heuristic::compute(current, heuristic_goal, map_.getMap()->getResolution());
            }

            bool term = goal_test.terminate(current);
            if(term) {
                if(!result.empty()) {
                    return result;
                } else {
                    break;
                }
            }

            // check if we can return
            if(use_candidates && !goal_candidates.empty()) {
                if(current->getTotalCost() > first_candidate_weight + search_options.oversearch_distance) {
                    break;
                }
            }


            // look at every free neighbor
            NeighborhoodType::iterateFreeNeighbors(*this, map_, current);

            // intermission, if used
            Intermission::call(intermission);
        }

        if(!goal_candidates.empty()) {
            // generate the path
            GoalCandidate best = goal_candidates.top();
            auto res = backtrack(std::get<0>(best));
            return res;
        }

        return {};
    }

    void init()
    {
        expansions = 0;
        multi_expansions = 0;
        updates = 0;

        result = {};
        use_candidates = search_options.oversearch_distance != 0.0;
        goal_candidates = std::priority_queue<GoalCandidate, std::vector<GoalCandidate>, PairDistance>();
        open.clear();

        start.clear();
        goal.clear();
    }

    void initStartPose(const PointT& pose)
    {
        bool outside = false;
        bool occupied = false;
        auto start_poses = NodeT::getPoses(map_, pose);
        for(NodeT* s : start_poses) {
            try {
                if(!map_.isOccupied(s)) {
                    // put start node in open data structure
                    s->distance = 0;
                    s->mark(NodeT::MARK_OPEN);
                    s->theta = pose.theta;

                    open.add(s);
                    start.insert(s);
                }

            } catch(const OutsideMapException& e) {
                outside = true;
            }
        }

        if(start.empty()) {
            if(occupied) {
                throw StartNotFreeException();
            } else if(outside) {
                throw StartOutOfMapException();
            }
        }
    }


    void initStartNode(const NodeT& node)
    {
        bool outside = false;
        bool occupied = false;
        auto start_poses = NodeT::getPoses(map_, node);
        for(NodeT* s : start_poses) {
            try {
                if(!map_.isOccupied(s)) {
                    // put start node in open data structure
//                    *s = node;
                    s->distance = 0;
                    s->mark(NodeT::MARK_OPEN);
                    s->theta = node.theta;
                    s->depth = 0;

                    open.add(s);
                    start.insert(s);
                }

            } catch(const OutsideMapException& e) {
                outside = true;
            }
        }

        if(start.empty()) {
            if(occupied) {
                throw StartNotFreeException();
            } else if(outside) {
                throw StartOutOfMapException();
            }
        }
    }

    void initGoalPose(const PointT& pose)
    {
//        try {
//            goal = map_.lookup(pose);
//        } catch(const OutsideMapException& e) {
//            throw GoalOutOfMapException();
//        }
//        Heuristic::setMap(&map_, *goal);

//        if(map_.isOccupied(goal)) {
//            throw GoalNotFreeException();

//        } else {
//            NodeT::init(*goal, pose);
//            goal->theta = pose.theta;
//        }


        bool outside = false;
        bool occupied = false;
        auto goal_poses = NodeT::getPoses(map_, pose);
        for(NodeT* g : goal_poses) {
            try {
                if(!map_.isOccupied(g)) {
                    // put start node in open data structure
                    g->theta = pose.theta;

                    goal.insert(g);
                }

            } catch(const OutsideMapException& e) {
                outside = true;
            }
        }

        if(start.empty()) {
            if(occupied) {
                throw GoalNotFreeException();
            } else if(outside) {
                throw GoalOutOfMapException();
            }
        }
    }

    bool hasPoseBeenReached(const NodeT* current)
    {
        if(AnalyticExpansionType::canExpand(current, goal, map_.getMap())) {
            PathT expansion;
            AnalyticExpansionType::getPath(&expansion);

            NodeT tmp = *current;
            tmp.theta = expansion.begin()->theta;

            PathT path = backtrack(&tmp);
            path.insert(path.end(), expansion.begin(), expansion.end());
            result = path;
            return true;
        }

        // check if the current node is a goal candiate

        for(const auto& g : goal) {
            if(NeighborhoodType::isGoal(g, current)) {
                double dist_to_goal = std::hypot(g->x - current->x, g->y - current->y);
                if(use_candidates) {
                    addGoalCandidate(current, dist_to_goal);

                    if(goal_candidate_function) {
                        PathT p = backtrack(current);
                        if(goal_candidate_function(p)) {
                            result = backtrack(current);
                            return true;
                        }
                    }

                } else { // no candidates
                    result = backtrack(current);
                    return true;
                }
            }
        }

        return false;
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
        double cost = delta * res;// + getCost(neighbor);
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

            if(!goal.empty()) {
                Heuristic::compute(neighbor, *goal.begin(), res);
            }
            neighbor->mark(NodeT::MARK_OPEN);

            ++touched;
            open.add(neighbor);

            return NeighborhoodBase::PR_ADDED_TO_OPEN_LIST;
        }

        return NeighborhoodBase::PR_IGNORED;
    }

    PathT backtrack(const NodeT* goal) {
        PathT path;

        const NodeT* current = goal;
        path.push_back(*goal);

        std::set<const NodeT*> contained;

        while(start.find(current) == start.end()){
            if(contained.find(current) != contained.end()) {
                return {};
            }

            contained.insert(current);

            current = dynamic_cast<NodeT*>(current->prev);
            assert(current != NULL);
            assert(current != current->prev);
            path.push_back(*current);
        };

        std::reverse(path.begin(), path.end());

        return path;
    }

    std::vector<PathT> getPathCandidates() {
        std::vector<PathT> paths;

        std::priority_queue<GoalCandidate, std::vector<GoalCandidate>, PairDistance> tmp = goal_candidates;

        while(!tmp.empty()) {
            const GoalCandidate gc = tmp.top();
            tmp.pop();

            paths.emplace_back(backtrack(std::get<0>(gc)));
        }

        return paths;
    }

    template <typename CB>
    void setPathCandidateCallback(CB cb) {
        goal_candidate_function = cb;
    }

public:
    SearchOptions search_options;

protected:
    OpenNodesManager open;
    MapManager map_;
    std::set<const NodeT*> start;
    std::set<const NodeT*> goal;

    const PointT* heuristic_goal;

    bool use_candidates;
    std::priority_queue<GoalCandidate, std::vector<GoalCandidate>, PairDistance> goal_candidates;
    double first_candidate_weight;
    PathT result;

    bool has_cost_;

    int expansions;
    int multi_expansions;
    int touched;
    int updates;

    double time_limit_;

    std::chrono::time_point<std::chrono::high_resolution_clock> stamp_abort_;

    std::function<bool(const PathT&)> goal_candidate_function;
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
