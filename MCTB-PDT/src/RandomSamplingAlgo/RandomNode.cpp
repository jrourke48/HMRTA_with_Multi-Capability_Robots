#include "RandomSamplingAlgo/RandomNode.h"

// Constructor - without next pointer
Random_Node::Random_Node(uint16_t id, Node* nodePtr, const std::vector<uint16_t> trueAPS, 
                          const std::vector<std::vector<uint8_t>>& allocations, std::vector<uint16_t> times)
    : nodeId(id), automatonState(nodePtr), trueAPs(trueAPS), 
      taskAllocations(allocations), times(times), next(nullptr) {
    setCurmakespan();
}

// Constructor - with next pointer
Random_Node::Random_Node(uint16_t id, Node* nodePtr, const std::vector<uint16_t> trueAPS, 
                          const std::vector<std::vector<uint8_t>>& allocations, std::vector<uint16_t> times, Random_Node* nextNode)
    : nodeId(id), automatonState(nodePtr), trueAPs(trueAPS), 
      taskAllocations(allocations), times(times), next(nextNode) {
    setCurmakespan();
}

// Set makespan based on times
void Random_Node::setCurmakespan() {
    // Makespan is the maximum time among all robots
    curmakespan = 0;
    for (uint16_t time : times) {
        if (time > curmakespan) {
            curmakespan = time;
        }
    }
}

// Node ID getters and setters
uint16_t Random_Node::getNodeId() const {
    return nodeId;
}

void Random_Node::setNodeId(uint16_t id) {
    nodeId = id;
}

// Automaton state getters and setters
Node* Random_Node::getautomatonState() const {
    return automatonState;
}

void Random_Node::setautomatonState(Node* nodePtr) {
    automatonState = nodePtr;
}

// Task allocation getters and setters
const std::vector<std::vector<uint8_t>>& Random_Node::getTaskAllocations() const {
    return taskAllocations;
}

void Random_Node::setTaskAllocations(const std::vector<std::vector<uint8_t>>& allocations) {
    taskAllocations = allocations;
}

const std::vector<uint8_t> Random_Node::getTaskAllocation(uint16_t trueAP) const {
    if (trueAP < taskAllocations.size()) {
        return taskAllocations[trueAP];
    }
    return std::vector<uint8_t>();
}

// True APs getters and setters
const std::vector<uint16_t>& Random_Node::getTrueAPs() const {
    return trueAPs;
}

void Random_Node::setTrueAPs(const std::vector<uint16_t>& aps) {
    trueAPs = aps;
}

// Times getters and setters
const std::vector<uint16_t>& Random_Node::getTimes() const {
    return times;
}

void Random_Node::setTimes(const std::vector<uint16_t>& newTimes) {
    times = newTimes;
    setCurmakespan();
}

// Current makespan getters and setters
uint16_t Random_Node::getCurmakespan() const {
    return curmakespan;
}

// Linked list navigation
Random_Node* Random_Node::getNext() const {
    return next;
}

void Random_Node::setNext(Random_Node* node) {
    next = node;
}
