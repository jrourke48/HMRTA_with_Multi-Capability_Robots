#include "ProductAutomaton.h"
#include <sstream>
#include <map>
#include <set>
#include <stack>
#include <queue>
#include <algorithm>
#include <functional>
#include "TarjansAlgorithm.cpp"
#include <spot/twaalgos/dot.hh>
#include <iostream>
#include <vector>
#include <climits>
using namespace std;

ProductAutomaton::ProductAutomaton() {
}

ProductAutomaton::ProductAutomaton(spot::twa_graph_ptr spotAutomaton) {
    if (!spotAutomaton) return;
    this->spotAutomaton = spotAutomaton;
    
    // Generate DOT representation from Spot automaton
    std::ostringstream dotStream;
    spot::print_dot(dotStream, spotAutomaton);
    std::string dotContent = dotStream.str();
    initStateMapping(dotContent);  // Initialize state mapping from DOT content
    // Parse the DOT content to build the automaton edges and accepting states
    parseProductFromDot(dotContent);
}

/**
 * Constructor to initialize the product automaton from the transition system (TS),
 * multi-robot system (MRS), and Buchi automaton.
 * This constructor builds the synchronized product of the TS for each robot and the Buchi automaton.
 */
ProductAutomaton::ProductAutomaton(const Environment& env, const MultiRobotSystem& mrs, const BuchiAutomaton& buchiAutomaton) {
    // Store pointers to env and mrs for later use
    envPtr = &env;
    mrsPtr = &mrs;
    
    // Initialize product automaton based on the individual components
    // This is a placeholder implementation and should be replaced with actual logic
    if (buchiAutomaton.getNumStates()*std::pow(5/6*env.getTransitionSystem()->getNumStates() , mrs.getNumRobots()) > UINT16_MAX) {
        std::cerr << "Product automaton too large to represent with uint16_t" << std::endl;
        return;
    }
    
    TS ts = *env.getTransitionSystem();

    // Get the number of robots in the multi-robot system 
    uint8_t numRobots = mrs.getNumRobots();
    
    // Convert the Buchi automaton to a Spot automaton
    spot::twa_graph_ptr buchiSpot = buchiAutomaton.getSpotAutomaton();
    // Retrieve the batch atomic proposition from the Buchi automaton
    std::vector<BatchAtomicProposition> batchAPs = buchiAutomaton.getLTLFormula()->getBatchAPs();
    
    // Initialize remaining TS states for each robot
    std::vector<std::vector<uint16_t>> robotStates;  // Robot r -> remaining state IDs
    std::vector<TS> allRobotTS;  // Store all robot TS objects
    
    for (uint8_t r = 0; r < numRobots; ++r) {
        // Create a copy of the TS for this robot
        TS ts_r = TS(ts);
        
        // Check each batch AP and remove incompatible nodes
        for (size_t i = 0; i < batchAPs.size(); ++i) {
            BatchAtomicProposition batchAP = batchAPs[i];
            std::vector<bool> Reqcapabilities = batchAP.getCapabilities();
            std::vector<bool> robotCapabilities = mrs.getRobotCapabilities(r);
            
            // Check if there are any matching capabilities
            bool hasMatchingCapability = false;
            for (size_t j = 0; j < Reqcapabilities.size() && j < robotCapabilities.size(); ++j) {
                if (Reqcapabilities[j] && robotCapabilities[j]) {
                    hasMatchingCapability = true;
                    break;
                } 
            }
            uint16_t AP = batchAP.getAP();
            if (!hasMatchingCapability && AP != 0) {
                ts_r.removeNode(AP);
            }
        }
        
        // Collect remaining state IDs for this robot
        std::vector<uint16_t> remainingStates;
        for (const auto& nodePair : ts_r.getNodes()) {
            remainingStates.push_back(nodePair.first);
        }
        
        robotStates.push_back(remainingStates);
        allRobotTS.push_back(ts_r);  // Store for later use in transition building
    }

    // Build the joint state space as a cartesian product of all robot TS's
    std::vector<std::vector<uint16_t>> cartesianStates;
    
    // Generate cartesian product combinations directly from remaining states
    std::function<void(int, std::vector<uint16_t>&)> generateCombinations = 
        [&](int robotIdx, std::vector<uint16_t>& currentCombo) {
        if (robotIdx == numRobots) {
            cartesianStates.push_back(currentCombo);
            return;
        }
        for (uint16_t state : robotStates[robotIdx]) {
            currentCombo.push_back(state);
            generateCombinations(robotIdx + 1, currentCombo);
            currentCombo.pop_back();
        }
    };
    
    std::vector<uint16_t> emptyCombo;
    generateCombinations(0, emptyCombo);
    
    std::cout << "Generated " << cartesianStates.size() << " cartesian combinations" << std::endl;
    
    // Create a combined automaton that represents the cartesian product
    spot::twa_graph_ptr cartesianProduct = spot::make_twa_graph(buchiSpot->get_dict());
    
    // Create a state for each cartesian combination
    for (size_t i = 0; i < cartesianStates.size(); ++i) {
        cartesianProduct->new_state();
    }
    
    // Set the initial state (all robots at initial state 0)
    cartesianProduct->set_init_state(0);
    
    // Add transitions: for each cartesian state, enumerate possible moves
    // Robots can move in ANY COMBINATION (in parallel), not just one at a time
    for (size_t cartIdx = 0; cartIdx < cartesianStates.size(); ++cartIdx) {
        const auto& currentCombo = cartesianStates[cartIdx];
        
        // Generate all non-empty subsets of robots (each subset can move together)
        for (uint8_t mask = 1; mask < (1 << numRobots); ++mask) {
            // mask represents which robots move: bit i set means robot i moves
            
            // Lambda to recursively generate all destination combinations for the moving robots
            std::function<void(int, std::vector<uint16_t>&)> generateDestinations =
                [&](int robotIdx, std::vector<uint16_t>& nextCombo) {
                if (robotIdx == numRobots) {
                    // Found a complete destination combo - now create edge to it
                    auto it = std::find(cartesianStates.begin(), cartesianStates.end(), nextCombo);
                    if (it != cartesianStates.end()) {
                        uint16_t dstCartIdx = std::distance(cartesianStates.begin(), it);
                        
                        // Label edge with COMPLETE proposition evaluation at destination state
                        bdd edgeLabel = bddtrue;
                        
                        for (const auto& batchAP : batchAPs) {
                            uint16_t apNodeId = batchAP.getAP();
                            uint16_t apId = batchAP.getAPId();
                            bool apSatisfied = false;
                            
                            // Check if ANY robot is at the AP's node
                            for (uint16_t robotPos : nextCombo) {
                                if (robotPos == apNodeId) {
                                    apSatisfied = true;
                                    break;
                                }
                            }
                            
                            std::string apName = "p" + std::to_string(apId);
                            spot::formula ap_formula = spot::formula::ap(apName);
                            auto& vm = buchiSpot->get_dict()->var_map;
                            auto it_ap = vm.find(ap_formula);
                            
                            if (it_ap != vm.end()) {
                                int varNum = it_ap->second;
                                bdd apVar = bdd_ithvar(varNum);
                                
                                if (apSatisfied) {
                                    edgeLabel &= apVar;
                                } else {
                                    edgeLabel &= !apVar;
                                }
                            }
                        }
                        
                        cartesianProduct->new_edge(cartIdx, dstCartIdx, edgeLabel);
                    }
                    return;
                }
                
                if (mask & (1 << robotIdx)) {
                    // Robot robotIdx is in the moving set - try all its destination states
                    uint16_t srcRobotState = currentCombo[robotIdx];
                    for (uint16_t dstRobotState : robotStates[robotIdx]) {
                        if (dstRobotState == srcRobotState) continue;  // Skip staying in same state
                        nextCombo[robotIdx] = dstRobotState;
                        generateDestinations(robotIdx + 1, nextCombo);
                    }
                } else {
                    // Robot robotIdx is not moving - it stays in place
                    nextCombo[robotIdx] = currentCombo[robotIdx];
                    generateDestinations(robotIdx + 1, nextCombo);
                }
            };
            
            std::vector<uint16_t> nextCombo = currentCombo;
            generateDestinations(0, nextCombo);
        }
    }
    
    spot::twa_graph_ptr productSpot = cartesianProduct;
    
    // Initialize state mapping with cartesian states
    initCartesianStateMapping(cartesianStates);
    
    // do the final synchronization of the product automaton
    productSpot = spot::product(buchiSpot, productSpot);
    // Export and update state mapping for final product
    std::ostringstream finalDotStream;
    spot::print_dot(finalDotStream, productSpot);
    std::string finalDotContent = finalDotStream.str();
    updateStateMapping(finalDotContent);
    
    // Store the Spot automaton and parse it using the already-populated stateMapping
    this->spotAutomaton = productSpot;
    
    // Now parse to build the internal representation (nodes, edges, accepting states)
    // This will use the stateMapping we just populated above
    std::ostringstream dotStream;
    spot::print_dot(dotStream, productSpot);
    parseProductFromDot(dotStream.str());
}

// Returns the optimal accepting path and its cumulative cost in the product automaton using a modified Dijkstra's algorithm.
// For Product automata: finds path with 2 loops to accepting state, then one additional node.
std::tuple<std::vector<uint16_t>, uint32_t> ProductAutomaton::OptimalAcceptingPath() {
    // Check if the product automaton has any accepting states
    if (acceptingStates.empty()) {
        std::cerr << "No accepting states in the product automaton!" << std::endl;
        return std::make_tuple(std::vector<uint16_t>(), UINT32_MAX);
    }

    if (nodeMap.empty()) {
        std::cerr << "Product automaton is empty!" << std::endl;
        return std::make_tuple(std::vector<uint16_t>(), UINT32_MAX);
    }

    // Map node IDs to indices for distance tracking
    std::map<uint16_t, int> nodeIdToIndex;
    std::vector<uint16_t> indexToNodeId;
    int idx = 0;
    for (const auto& pair : nodeMap) {
        nodeIdToIndex[pair.first] = idx;
        indexToNodeId.push_back(pair.first);
        idx++;
    }

    int numNodes = indexToNodeId.size();
    
    // Helper lambda: Dijkstra from a specific start node to find accepting state
    // If skipStartNode=true, skips the starting node and finds a different accepting state
    // Returns: (targetIdx, parents, distances)
    auto dijkstraToAccepting = [&](int startIdx, bool skipStartNode = false) -> std::tuple<int, std::vector<int>, std::vector<uint32_t>> {
        std::vector<uint32_t> dist(numNodes, UINT32_MAX);
        std::vector<int> parent(numNodes, -1);
        std::priority_queue<std::pair<uint32_t, int>, std::vector<std::pair<uint32_t, int>>, std::greater<>> pq;

        dist[startIdx] = 0;
        pq.push({0, startIdx});

        int targetIdx = -1;
        bool firstIteration = true;

        while (!pq.empty()) {
            auto [currDist, currIdx] = pq.top();
            pq.pop();

            if (currDist > dist[currIdx])
                continue;

            // Check if current node is accepting (but skip start node if requested)
            uint16_t currNodeId = indexToNodeId[currIdx];
            if (!firstIteration || !skipStartNode) {  // Allow start node on first iteration only if skipStartNode=false
                if (std::find(acceptingStates.begin(), acceptingStates.end(), currNodeId) != acceptingStates.end()) {
                    targetIdx = currIdx;
                    break;  // Found accepting state
                }
            }
            firstIteration = false;

            // Explore neighbors
            Node* currNode = nodeMap[currNodeId];
            if (currNode) {
                for (const auto& edge : currNode->getEdges()) {
                    uint16_t neighborNodeId = edge.getDstId();
                    if (nodeIdToIndex.find(neighborNodeId) == nodeIdToIndex.end())
                        continue;

                    int neighborIdx = nodeIdToIndex[neighborNodeId];
                    uint32_t edgeWeight = edge.getWeight();

                    if (dist[currIdx] != UINT32_MAX && dist[currIdx] + edgeWeight < dist[neighborIdx]) {
                        dist[neighborIdx] = dist[currIdx] + edgeWeight;
                        parent[neighborIdx] = currIdx;
                        pq.push({dist[neighborIdx], neighborIdx});
                    }
                }
            }
        }

        return {targetIdx, parent, dist};
    };

    // Helper lambda: get next node from current
    auto getNextNode = [&](int currIdx) -> int {
        Node* currNode = nodeMap[indexToNodeId[currIdx]];
        if (currNode && !currNode->getEdges().empty()) {
            uint16_t nextNodeId = currNode->getEdges()[0].getDstId();
            if (nodeIdToIndex.find(nextNodeId) != nodeIdToIndex.end()) {
                return nodeIdToIndex[nextNodeId];
            }
        }
        return -1;
    };

    // Step 1: Find path from initial state (0) to first accepting state
    auto [firstAcceptingIdx, parent1, dist1] = dijkstraToAccepting(nodeIdToIndex[0], false);
    if (firstAcceptingIdx == -1) {
        std::cerr << "No path to accepting state found!" << std::endl;
        return std::make_tuple(std::vector<uint16_t>(), UINT32_MAX);
    }

    // Step 2: Find path from first accepting state to second accepting state (first loop)
    // Skip the first accepting state itself, find a different accepting state
    auto [secondAcceptingIdx, parent2, dist2] = dijkstraToAccepting(firstAcceptingIdx, true);
    if (secondAcceptingIdx == -1) {
        std::cerr << "No path to second accepting state found!" << std::endl;
        return std::make_tuple(std::vector<uint16_t>(), UINT32_MAX);
    }

    // Step 3: Find path from second accepting state to third accepting state (second loop)
    // Skip the second accepting state itself, find a different accepting state
    auto [thirdAcceptingIdx, parent3, dist3] = dijkstraToAccepting(secondAcceptingIdx, true);
    if (thirdAcceptingIdx == -1) {
        std::cerr << "No path to third accepting state found!" << std::endl;
        return std::make_tuple(std::vector<uint16_t>(), UINT32_MAX);
    }

    // Step 4: Get one additional node from third accepting state
    int additionalNodeIdx = getNextNode(thirdAcceptingIdx);

    // Reconstruct full path with weights: initial → first accepting → second accepting → third accepting → +1 node
    std::vector<uint16_t> result;
    std::vector<uint32_t> edgeWeights;  // weights between consecutive nodes

    // Path to first accepting state
    std::vector<int> segment1Path;
    int current = firstAcceptingIdx;
    while (current != -1) {
        segment1Path.insert(segment1Path.begin(), current);
        current = parent1[current];
    }
    for (int idx : segment1Path) {
        result.push_back(indexToNodeId[idx]);
    }

    // Path from first to second accepting state (skip first node to avoid duplication)
    std::vector<int> segment2Path;
    current = secondAcceptingIdx;
    while (current != -1 && current != firstAcceptingIdx) {
        segment2Path.insert(segment2Path.begin(), current);
        current = parent2[current];
    }
    for (int idx : segment2Path) {
        result.push_back(indexToNodeId[idx]);
    }

    // Path from second to third accepting state (skip first node to avoid duplication)
    std::vector<int> segment3Path;
    current = thirdAcceptingIdx;
    while (current != -1 && current != secondAcceptingIdx) {
        segment3Path.insert(segment3Path.begin(), current);
        current = parent3[current];
    }
    for (int idx : segment3Path) {
        result.push_back(indexToNodeId[idx]);
    }

    // Add one additional node if found
    if (additionalNodeIdx != -1) {
        result.push_back(indexToNodeId[additionalNodeIdx]);
    }

    // Calculate edge weights
    std::vector<uint32_t> weights;
    for (size_t i = 0; i < result.size() - 1; ++i) {
        uint16_t srcNodeId = result[i];
        uint16_t dstNodeId = result[i + 1];
        
        Node* srcNode = nodeMap[srcNodeId];
        uint32_t weight = 0;
        
        if (srcNode) {
            for (const auto& edge : srcNode->getEdges()) {
                if (edge.getDstId() == dstNodeId) {
                    weight = edge.getWeight();
                    break;
                }
            }
        }
        weights.push_back(weight);
    }

    // Output path with weights
    std::cout << "\nOptimal Accepting Path (with edge weights):" << std::endl;
    std::cout << "  Accepting states found: " << indexToNodeId[firstAcceptingIdx] 
              << ", " << indexToNodeId[secondAcceptingIdx] 
              << ", " << indexToNodeId[thirdAcceptingIdx] << std::endl;
    std::cout << "  Path: ";
    for (size_t i = 0; i < result.size(); ++i) {
        if (i > 0) std::cout << " -(" << weights[i-1] << ")-> ";
        std::cout << result[i];
        
        // Also show product states (robot positions)
        Node* node = nodeMap[result[i]];
        if (node) {
            auto productStates = node->getProductStates().second;
            std::cout << "[";
            for (size_t j = 0; j < productStates.size(); ++j) {
                if (j > 0) std::cout << ",";
                std::cout << productStates[j];
            }
            std::cout << "]";
        }
    }
    std::cout << std::endl;
    
    uint32_t totalWeight = 0;
    for (uint32_t w : weights) totalWeight += w;
    std::cout << "  Total weight: " << totalWeight << std::endl;

    return std::make_tuple(result, totalWeight);
}


// Parse product automaton from DOT representation (uses member variables envPtr and mrsPtr)
void ProductAutomaton::parseProductFromDot(const std::string& dotContent) {
    std::istringstream stream(dotContent);
    std::string line;
    std::map<unsigned, std::vector<std::pair<unsigned, std::string>>> edges;
    std::set<unsigned> acceptingNodeIds;

    // Create nodes based on all entries in stateMapping (already populated)
    for (const auto& mappingPair : stateMapping) {
        uint16_t nodeId = mappingPair.first;
        const std::string& label = mappingPair.second;
        Node* node = new Node(nodeId, label, true);
        add_Node(node);
    }

    while (std::getline(stream, line)) {
        
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for accepting state marker: peripheries=2
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX && line.find("peripheries=2") != std::string::npos) {
                    acceptingNodeIds.insert(nodeId);
                }
            }
            continue;
        }
        
        // Check for edge definition: <src> -> <dst> [label="..."]
        size_t arrow_pos = line.find("->");
        if (arrow_pos != std::string::npos) {
            // Extract source
            unsigned src;
            std::istringstream src_stream(line.substr(0, arrow_pos));
            if (!(src_stream >> src)) continue;
            
            // Extract destination
            size_t after_arrow = arrow_pos + 2;
            size_t bracket_start = line.find('[', after_arrow);
            if (bracket_start == std::string::npos) continue;
            
            std::string dst_str = line.substr(after_arrow, bracket_start - after_arrow);
            dst_str.erase(0, dst_str.find_first_not_of(" \t"));
            dst_str.erase(dst_str.find_last_not_of(" \t") + 1);
            
            unsigned dst;
            if (!(std::istringstream(dst_str) >> dst)) continue;
            
            // Skip initial edge (I -> state)
            if (src == UINT_MAX) continue;
            
            edges[src].push_back(std::make_pair(dst, ""));
        }
    }
    
    // Add edges - use member variables (envPtr and mrsPtr must be valid)
    if (envPtr == nullptr || mrsPtr == nullptr) {
        std::cerr << "ERROR: Environment or MultiRobotSystem pointers are null in parseProductFromDot!" << std::endl;
        return;
    }
    
    for (const auto& srcEntry : edges) {
        Node* srcNode = getNode(static_cast<uint16_t>(srcEntry.first));
        if (srcNode != nullptr) {
            for (const auto& dstLabelPair : srcEntry.second) {
                unsigned dst = dstLabelPair.first;
                
                if (dst <= UINT16_MAX) {
                    Node* dstNode = getNode(static_cast<uint16_t>(dst));
                    if (dstNode != nullptr) {
                        uint32_t wght = getEdgeWeight(srcNode, dstNode);
                        // Only add edge if weight is positive
                        if (wght > 0) {
                            Edge edge_obj(static_cast<uint16_t>(dst), wght);
                            srcNode->addEdge(edge_obj);
                            numEdges++;
                        }
                    }
                }
            }
        }
    }
    // Mark accepting states (peripheries=2 in DOT)
    for (unsigned nodeId : acceptingNodeIds) {
        if (nodeId <= UINT16_MAX) {
            setAccepting(static_cast<uint16_t>(nodeId));
        }
    }
}

// Extract label from DOT bracket content
std::string ProductAutomaton::extractLabelFromDotBrackets(const std::string& content) const {
    size_t label_pos = content.find("label=");
    if (label_pos == std::string::npos) return "true";
    
    label_pos += 6;  // strlen("label=")
    
    // Skip whitespace and opening quote
    while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '"')) {
        label_pos++;
    }
    
    std::string label;
    while (label_pos < content.length()) {
        char c = content[label_pos];
        
        if (c == '\\' && label_pos + 1 < content.length()) {
            char next = content[label_pos + 1];
            if (next == '"') {
                label += '"';
                label_pos += 2;
            } else if (next == 'n') {
                label += ':';
                label_pos += 2;
                
                // Skip whitespace after newline
                while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '\t')) {
                    label_pos++;
                }
                
                // Grab acceptance marks
                while (label_pos < content.length()) {
                    char c = content[label_pos];
                    if (c == '"') break;
                    label += c;
                    label_pos++;
                }
                break;
            } else {
                label += c;
                label_pos++;
            }
        } else if (c == '"') {
            break;
        } else {
            label += c;
            label_pos++;
        }
    }
    
    // Clean up label
    label.erase(label.find_last_not_of(" \t") + 1);
    return label.empty() ? "true" : label;
}

ProductAutomaton::~ProductAutomaton() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void ProductAutomaton::setAccepting(uint16_t stateId) {
    // Add to accepting states if not already present
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    if (it == acceptingStates.end()) {
        acceptingStates.push_back(stateId);
    }
}

bool ProductAutomaton::isAccepting(uint16_t stateId) const {
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    return it != acceptingStates.end();
}

const std::vector<uint16_t>& ProductAutomaton::getAcceptingStates() const {
    return acceptingStates;
}

void ProductAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool ProductAutomaton::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}
void ProductAutomaton::addStateMapping(uint16_t productState, const std::string& label) {
    // Store the label string at this product state key
    stateMapping[productState] = label;
}

void ProductAutomaton::initStateMapping(const std::string& dotContent) {
    std::istringstream stream(dotContent);
    std::string line;
    
    while (std::getline(stream, line)) {
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for node definition: <id> [label="..." peripheries=2 ...]
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX) {
                    // Extract label from node definition
                    size_t bracket_end = line.find(']', bracket_start);
                    if (bracket_end == std::string::npos) bracket_end = line.length();
                    
                    std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    std::string nodeLabel = extractLabelFromDotBrackets(bracket_content);
                    
                    stateMapping[static_cast<uint16_t>(nodeId)] = nodeLabel;
                }
            }
            continue;
        }
    }
}
void ProductAutomaton::updateStateMapping(const std::string& dotContent) {
    // Save a copy of the current stateMapping to reference old labels
    std::map<uint16_t, std::string> oldStateMapping = stateMapping;
    
    std::istringstream stream(dotContent);
    std::string line;
    std::set<unsigned> seenNodes;
    
     while (std::getline(stream, line)) {
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for node definition: <id> [label="..." peripheries=2 ...]
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX) {
                    seenNodes.insert(nodeId);
                    
                    // Extract label from node definition
                    size_t bracket_end = line.find(']', bracket_start);
                    if (bracket_end == std::string::npos) bracket_end = line.length();
                    
                    std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    std::string nodeLabel = extractLabelFromDotBrackets(bracket_content);
                    
                    // The node label needs to be updated in the mapping
                    // Look up the old ID from the saved oldStateMapping
                    uint16_t oldId = getIdFromLabel(nodeLabel);
                    if (oldStateMapping.find(oldId) != oldStateMapping.end()) {
                        std::string oldLabel = oldStateMapping[oldId];
                        std::string expanded = replaceLabel(nodeLabel, oldLabel);
                        stateMapping[nodeId] = expanded;
                    } else {
                        stateMapping[nodeId] = nodeLabel;
                    }
                }
            }
            continue;
        }
        
    }
}

uint16_t ProductAutomaton::getIdFromLabel(const std::string& label) const {
    // The ID is the number after the comma
    size_t commaPos = label.find(',');
    if (commaPos != std::string::npos) {
        // Extract everything after the comma
        std::string afterComma = label.substr(commaPos + 1);
        // Trim whitespace
        afterComma.erase(0, afterComma.find_first_not_of(" \t"));
        afterComma.erase(afterComma.find_last_not_of(" \t") + 1);
        return static_cast<uint16_t>(std::stoi(afterComma));
    } else {
        // No comma found, return the whole label as the ID
        return static_cast<uint16_t>(std::stoi(label));
    }
}

std::string ProductAutomaton::replaceLabel(const std::string& oldLabel, const std::string& additionalLabel) {
    // Find the first comma
    size_t commaPos = oldLabel.find(',');
    if (commaPos != std::string::npos) {
        // Keep everything up to and including the comma, replace everything after
        return oldLabel.substr(0, commaPos + 1) + additionalLabel;
    } else {
        // No comma found, append with comma
        std::string newLabel = oldLabel;
        if (!newLabel.empty()) {
            newLabel += ",";
        }
        newLabel += additionalLabel;
        return newLabel;
    }
}


std::string ProductAutomaton::getStateMapping(uint16_t productState) const {
    auto it = stateMapping.find(productState);
    if (it != stateMapping.end()) {
        return it->second;
    }
    
    // Return empty string if state not found
    return "";
}

void ProductAutomaton::initCartesianStateMapping(std::vector<std::vector<uint16_t>> cartesianStates) {
    for (const auto& stateVector : cartesianStates) {
        std::ostringstream labelStream;
        for (size_t i = 0; i < stateVector.size(); ++i) {
            if (i > 0) {
                labelStream << ",";
            }
            labelStream << stateVector[i];
        }
        std::string label = labelStream.str();
        uint16_t productState = static_cast<uint16_t>(stateMapping.size());
        stateMapping[productState] = label;
    }
}
uint32_t ProductAutomaton::getEdgeWeight(Node* srcNode, Node* dstNode) const {
    // Defensive checks
    if (srcNode == nullptr || dstNode == nullptr) {
        std::cerr << "ERROR: getEdgeWeight called with null node!" << std::endl;
        return 0;
    }
    
    // MUST use member variables - env and mrs must be valid at this point
    if (envPtr == nullptr || mrsPtr == nullptr) {
        std::cerr << "ERROR: Environment or MultiRobotSystem pointers are null in getEdgeWeight!" << std::endl;
        return 0;
    }
    
    uint16_t maxtime = 0;
    
    try {
        std::vector<uint16_t> srcProductStates = srcNode->getProductStates().second;
        std::vector<uint16_t> dstProductStates = dstNode->getProductStates().second;
        
        // If product states are empty, return 0 (no weight can be computed)
        if (srcProductStates.empty() || dstProductStates.empty()) {
            return 0;
        }
        
        for (size_t i = 0; i < srcProductStates.size(); ++i) {
            uint16_t srcState = srcProductStates[i];
            uint16_t dstState = (i < dstProductStates.size()) ? dstProductStates[i] : srcState;
            
            if (srcState != dstState) {
                // Bounds check on robot index
                if (i >= mrsPtr->getNumRobots()) {
                    std::cerr << "ERROR: Robot index " << i << " exceeds " << mrsPtr->getNumRobots() << " robots!" << std::endl;
                    continue;
                }
                
                auto robot = mrsPtr->getRobot(i);
                if (robot == nullptr) {
                    std::cerr << "ERROR: getRobot returned nullptr for robot " << i << std::endl;
                    continue;
                }
                
                auto srcGridCenter = envPtr->TSStateIdToGridCenter(srcState);
                auto dstGridCenter = envPtr->TSStateIdToGridCenter(dstState);
                
                uint16_t weight = robot->getTravelTime(srcGridCenter, dstGridCenter);
                
                if (weight > maxtime) {
                    maxtime = weight;
                }
            }
        }
    } catch (const std::exception& e_ex) {
        std::cerr << "ERROR in getEdgeWeight: " << e_ex.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "ERROR in getEdgeWeight: Unknown exception!" << std::endl;
        return 0;
    }
    
    return maxtime;
}


