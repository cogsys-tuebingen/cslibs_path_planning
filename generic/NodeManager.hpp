/*
 * NodeManager.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef NODEMANAGER_H
#define NODEMANAGER_H

/// COMPONENT
#include "Common.hpp"

/// PROJECT
#include <utils_general/DynamicPriorityQueue.hpp>

/// SYSTEM
#include <stack>

namespace lib_path
{

/**
 * @brief The QueueManager struct represents the structure of open nodes as a queue
 */
template <class Node>
struct QueueManager {
    void add(Node* node) {
        queue.push_back(node);
    }

    bool empty() {
        return queue.empty();
    }

    void remove(Node* element) {
    }

    void clear() {
        queue.clear();
    }

    Node* next() {
        assert(!queue.empty());
        Node* next = queue.front();
        queue.pop_front();
        return next;
    }

    std::size_t size() const
    {
        return queue.size();
    }

    typename std::deque<Node*>::const_iterator begin() const {
        return queue.begin();
    }

    typename std::deque<Node*>::const_iterator end() const {
        return queue.end();
    }

private:
    std::deque<Node*> queue;
};

/**
 * @brief The GenericManager struct represents the structure of open nodes as generic type
 */
template <class Node, class Container>
struct GenericManager {
    void add(Node* node) {
        container.push(node);
    }

    bool empty() {
        return container.empty();
    }

    void remove(Node* element) {
        container.remove(element);
    }

    void clear() {
        // workaround for queues
        while(!empty()) {
            next();
        }
    }

    std::size_t size() const
    {
        return container.size();
    }

    Node* next() {
        assert(!container.empty());
        Node* next = container.top();
        container.pop();
        return next;
    }
    //D-Lite
    Node* top() {
        return container.top();
    }

    typename Container::const_iterator begin() const {
        return container.begin();
    }

    typename Container::const_iterator end() const {
        return container.end();
    }

private:
    Container container;
};


/**
 * @brief The PriorityQueueManager struct represents the structure of open nodes as a prio queue
 */
template <class Node>
class PriorityQueueManager : public GenericManager<Node, DynamicPrioritySet<Node*, CompareNode<Node> > >
//        class PriorityQueueManager : public GenericManager<Node, DynamicPriorityQueue<Node*, std::vector<Node*>, CompareNode<Node> > >
{
};


/**
 * @brief The StackManager struct represents the structure of open nodes as a stack
 */
template <class Node>
class StackManager : public GenericManager<Node, std::stack<Node*> >
{
};

}

#endif // NODEMANAGER_H
