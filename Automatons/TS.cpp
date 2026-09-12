#include "TS.h"
#include <algorithm>
#include <iostream>

TS::TS() {
}

// Deep copy constructor
TS::TS(const TS& other) 
    : initialStates(other.initialStates), 
      stateToNodeId(other.stateToNodeId) {
    // Copy numNodes and numEdges from parent
    numNodes = other.numNodes;
    numEdges = other.numEdges;
    
    // Deep copy all nodes in nodeMap
    for (const auto& pair : other.nodeMap) {
        uint16_t nodeId = pair.first;
        const Node* srcNode = pair.second;
        
        // Create a new Node that's a copy of the source node
        Node* newNode = new Node(*srcNode);
        nodeMap[nodeId] = newNode;
    }
}

TS::~TS() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void TS::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}
void TS::removeNode(uint16_t nodeId) {
    auto it = nodeMap.find(nodeId);
    if (it != nodeMap.end()) {
        delete it->second;
        nodeMap.erase(it);
        numNodes--;
    }
}

bool TS::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}

std::vector<uint16_t> TS::getAdjacent(uint16_t nodeId) const {
    std::vector<uint16_t> adjacentNodes;
    
    // Find the node
    auto it = nodeMap.find(nodeId);
    if (it == nodeMap.end()) return adjacentNodes;
    
    Node* node = it->second;
    
    // Get all edges from this node
    std::vector<Edge> edges = node->getEdges();
    
    // Extract destination IDs from each edge
    for (const auto& edge : edges) {
        adjacentNodes.push_back(edge.getDstId());
    }
    
    return adjacentNodes;
}

void TS::setInitial(uint16_t stateId) {
    // Add to initial states if not already present
    auto it = std::find(initialStates.begin(), initialStates.end(), stateId);
    if (it == initialStates.end()) {
        initialStates.push_back(stateId);
    }
}

bool TS::isInitial(uint16_t stateId) const {
    auto it = std::find(initialStates.begin(), initialStates.end(), stateId);
    return it != initialStates.end();
}

const std::vector<uint16_t>& TS::getInitialStates() const {
    return initialStates;
}

spot::twa_graph_ptr TS::toSpotAutomaton(spot::bdd_dict_ptr dict) const {
    // Use provided dictionary or create a new one
    if (!dict) {
        dict = spot::make_bdd_dict();
    }
    
    // Create a new Spot automaton with the dictionary
    spot::twa_graph_ptr aut = spot::make_twa_graph(dict);
    
    // Create a mapping from node ID to Spot state index
    std::unordered_map<uint16_t, unsigned> nodeIdToSpotState;
    unsigned spotStateIndex = 0;
    
    // Create states in the Spot automaton (one per node)
    for (uint32_t i = 0; i < numNodes; ++i) {
        aut->new_state();
    }
    
    // Build the mapping from node IDs to Spot state indices
    for (const auto& nodePair : nodeMap) {
        uint16_t nodeId = nodePair.first;
        nodeIdToSpotState[nodeId] = spotStateIndex++;
    }
    
    // Set initial state(s) using the mapping
    if (!initialStates.empty()) {
        uint32_t initNodeId = initialStates[0];
        if (nodeIdToSpotState.find(initNodeId) != nodeIdToSpotState.end()) {
            aut->set_init_state(nodeIdToSpotState[initNodeId]);
        }
    }
    
    // Add edges from the TS graph
    for (const auto& nodePair : nodeMap) {
        uint32_t srcId = nodePair.first;
        Node* srcNode = nodePair.second;
        
        if (srcNode == nullptr) continue;
        
        unsigned srcSpotState = nodeIdToSpotState[srcId];
        
        // Iterate through outgoing edges
        for (const auto& edge : srcNode->getEdges()) {
            uint32_t dstId = edge.getDstId();
            unsigned dstSpotState = nodeIdToSpotState[dstId];
            // Add edge with bddtrue (true transition) and NO acceptance mark
            aut->new_edge(srcSpotState, dstSpotState, bddtrue, spot::acc_cond::mark_t());
        }
    }
    
    return aut;
}

