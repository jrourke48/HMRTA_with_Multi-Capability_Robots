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
            if (!hasMatchingCapability || ts_r.isInitial(AP)) {
                ts_r.removeNode(AP);
                std::cout << "Robot " << static_cast<int>(r) << " does not have the required capabilities for AP " << AP << std::endl;
            }
        }
        
        // Collect remaining state IDs for this robot
        std::vector<uint16_t> remainingStates;
        for (const auto& nodePair : ts_r.getNodes()) {
            remainingStates.push_back(nodePair.first);
        }
        std::cout << "Robot " << static_cast<int>(r) << " remaining states: ";
        for (uint16_t s : remainingStates) std::cout << s << " ";
        std::cout << std::endl;
        
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
    for (size_t cartIdx = 0; cartIdx < cartesianStates.size(); ++cartIdx) {
        const auto& currentCombo = cartesianStates[cartIdx];
        
        // Try moving each robot independently
        for (uint8_t robot = 0; robot < numRobots; ++robot) {
            uint16_t srcRobotState = currentCombo[robot];
            
            // Assume TS is fully connected: each robot can transition from any state to any other state it has
            for (uint16_t dstRobotState : robotStates[robot]) {
                if (dstRobotState == srcRobotState) continue;  // Skip self-loop
                
                // Build the destination cartesian state
                std::vector<uint16_t> nextCombo = currentCombo;
                nextCombo[robot] = dstRobotState;
                
                // Find the index of this new state in cartesianStates
                auto it = std::find(cartesianStates.begin(), cartesianStates.end(), nextCombo);
                if (it != cartesianStates.end()) {
                    uint16_t dstCartIdx = std::distance(cartesianStates.begin(), it);
                    // Add transition with true (any AP)
                    cartesianProduct->new_edge(cartIdx, dstCartIdx, bddtrue);
                }
            }
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

std::vector<uint16_t> ProductAutomaton::OptimalAcceptingPath() {
    std::vector<uint16_t> result;
    
    if (nodeMap.empty() || acceptingStates.empty()) return result;
    
    // Step 1: Create mapping from node IDs to indices for Tarjan
    std::vector<uint16_t> indexToNodeId;
    std::map<uint16_t, int> nodeIdToIndex;
    
    for (const auto& pair : nodeMap) {
        nodeIdToIndex[pair.first] = indexToNodeId.size();
        indexToNodeId.push_back(pair.first);
    }
    
    int numNodes = indexToNodeId.size();
    
    // Step 2: Build adjacency list and edge weights
    std::vector<std::vector<int>> adj(numNodes);
    std::vector<std::vector<std::pair<int, uint16_t>>> weightedAdj(numNodes);  // {neighbor, weight}
    
    for (int i = 0; i < numNodes; ++i) {
        uint16_t nodeId = indexToNodeId[i];
        Node* node = nodeMap[nodeId];
        
        for (const auto& edge : node->getEdges()) {
            uint16_t destId = edge.getDstId();
            int destIndex = nodeIdToIndex[destId];
            adj[i].push_back(destIndex);
            weightedAdj[i].push_back({destIndex, edge.getWeight()});
        }
    }
    
    // Step 3: Run Tarjan's algorithm to find all SCCs
    std::vector<std::vector<int>> sccs = getSCCs(adj);
    
    // Step 4: Identify which SCCs contain accepting states
    std::map<int, bool> sccHasAccepting;
    std::map<int, int> nodeIndexToSccId;
    
    for (int sccIdx = 0; sccIdx < static_cast<int>(sccs.size()); ++sccIdx) {
        sccHasAccepting[sccIdx] = false;
        for (int nodeIdx : sccs[sccIdx]) {
            nodeIndexToSccId[nodeIdx] = sccIdx;
            uint16_t nodeId = indexToNodeId[nodeIdx];
            
            if (std::find(acceptingStates.begin(), acceptingStates.end(), nodeId) != acceptingStates.end()) {
                sccHasAccepting[sccIdx] = true;
            }
        }
    }
    
    // Step 5: Use Dijkstra's algorithm to find minimum cost path to accepting SCC
    uint16_t initialState = 0;
    int initialStateIdx = nodeIdToIndex[initialState];
    
    std::vector<uint32_t> dist(numNodes, UINT32_MAX);
    std::vector<int> parent(numNodes, -1);
    std::priority_queue<std::pair<uint32_t, int>, std::vector<std::pair<uint32_t, int>>, std::greater<>> pq;
    
    dist[initialStateIdx] = 0;
    pq.push({0, initialStateIdx});
    
    int targetStateIdx = -1;
    int targetSccId = -1;
    
    while (!pq.empty()) {
        auto [currDist, currIdx] = pq.top();
        pq.pop();
        
        if (currDist > dist[currIdx]) continue;
        
        // Check if this state's SCC has accepting states
        int currSccId = nodeIndexToSccId[currIdx];
        if (sccHasAccepting[currSccId]) {
            targetStateIdx = currIdx;
            targetSccId = currSccId;
            break;
        }
        
        // Explore neighbors with weights
        for (const auto& [neighbor, weight] : weightedAdj[currIdx]) {
            if (dist[currIdx] != UINT32_MAX && dist[currIdx] + weight < dist[neighbor]) {
                dist[neighbor] = dist[currIdx] + weight;
                parent[neighbor] = currIdx;
                pq.push({dist[neighbor], neighbor});
            }
        }
    }
    
    if (targetStateIdx == -1) return result;
    
    // Step 6: Reconstruct minimum cost path to target
    std::vector<uint16_t> pathToAccepting;
    int current = targetStateIdx;
    while (current != -1) {
        pathToAccepting.insert(pathToAccepting.begin(), indexToNodeId[current]);
        current = parent[current];
    }
    
    // Step 7: Find minimum cost cycle within the SCC containing an accepting state
    // Using Dijkstra from the target node within the SCC
    std::vector<uint32_t> cycleDist(numNodes, UINT32_MAX);
    std::vector<int> cycleParent(numNodes, -1);
    std::priority_queue<std::pair<uint32_t, int>, std::vector<std::pair<uint32_t, int>>, std::greater<>> cyclePq;
    
    cycleDist[targetStateIdx] = 0;
    cyclePq.push({0, targetStateIdx});
    
    uint32_t minCycleCost = UINT32_MAX;
    int cycleReturnNode = -1;
    
    while (!cyclePq.empty()) {
        auto [currDist, currIdx] = cyclePq.top();
        cyclePq.pop();
        
        if (currDist > cycleDist[currIdx]) continue;
        
        // Check if we can return to target node (forming a cycle)
        for (const auto& [neighbor, weight] : weightedAdj[currIdx]) {
            if (neighbor == targetStateIdx && nodeIndexToSccId[currIdx] == targetSccId) {
                uint32_t totalCycleCost = currDist + weight;
                if (totalCycleCost < minCycleCost) {
                    minCycleCost = totalCycleCost;
                    cycleReturnNode = currIdx;
                }
            }
            
            // Only explore within SCC
            if (nodeIndexToSccId[neighbor] == targetSccId && 
                cycleDist[currIdx] != UINT32_MAX && 
                cycleDist[currIdx] + weight < cycleDist[neighbor]) {
                cycleDist[neighbor] = cycleDist[currIdx] + weight;
                cycleParent[neighbor] = currIdx;
                cyclePq.push({cycleDist[neighbor], neighbor});
            }
        }
    }
    
    // Step 8: Reconstruct minimum cost cycle
    std::vector<uint16_t> cycle;
    if (cycleReturnNode != -1) {
        int current = cycleReturnNode;
        while (current != -1 && current != targetStateIdx) {
            cycle.insert(cycle.begin(), indexToNodeId[current]);
            current = cycleParent[current];
        }
        cycle.insert(cycle.begin(), indexToNodeId[targetStateIdx]);
    }
    
    // Step 9: Build final result with minimum cost path + minimum cost cycle
    for (uint16_t s : pathToAccepting) {
        result.push_back(s);
    }
    
    if (!cycle.empty()) {
        // Add minimum cost cycle 3 times to show repeating pattern
        for (int rep = 0; rep < 3; rep++) {
            for (size_t i = 1; i < cycle.size(); ++i) {  // Skip first node to avoid duplication
                result.push_back(cycle[i]);
            }
        }
    } else {
        // No cycle found - repeat target state
        uint16_t targetNodeId = indexToNodeId[targetStateIdx];
        for (int i = 0; i < 5; i++) {
            result.push_back(targetNodeId);
        }
    }
    
    return result;
}

// Parse product automaton from DOT representation with Edge weights
void ProductAutomaton::parseProductFromDot(const std::string& dotContent, const Environment& env, const MultiRobotSystem& mrs) {
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
    
    // Add edges
    for (const auto& srcEntry : edges) {
        Node* srcNode = getNode(static_cast<uint16_t>(srcEntry.first));
        if (srcNode != nullptr) {
            for (const auto& dstLabelPair : srcEntry.second) {
                unsigned dst = dstLabelPair.first;
                
                if (dst <= UINT16_MAX) {
                    Edge e(static_cast<uint16_t>(dst), getEdgeWeight(srcNode, getNode(static_cast<uint16_t>(dst)), env, mrs));
                    srcNode->addEdge(e);
                    numEdges++;
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
uint32_t ProductAutomaton::getEdgeWeight(Node* srcNode, Node* dstNode, const Environment& env, const MultiRobotSystem& mrs) const {
    uint16_t maxtime = 0;
    std::vector<uint16_t> srcProductStates = srcNode->getProductStates().second;  //get the src nodes product states
    std::vector<uint16_t> dstProductStates = dstNode->getProductStates().second;  //get the dst nodes product states
    for (size_t i = 0; i < srcProductStates.size(); ++i) {
        uint16_t srcState = srcProductStates[i];
        uint16_t dstState = (i < dstProductStates.size()) ? dstProductStates[i] : srcState;
        if (srcState != dstState) {
            uint16_t weight = mrs.getRobot(i)->getTravelTime(env.TSStateIdToGridCenter(srcState), env.TSStateIdToGridCenter(dstState));  // Example logic to get the weight from the environment and multi-robot system
            if (weight > maxtime) {
                maxtime = weight;
            }
        }
    }
    return maxtime;
}

// Parse product automaton from DOT representation
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
    
    // Add edges
    for (const auto& srcEntry : edges) {
        Node* srcNode = getNode(static_cast<uint16_t>(srcEntry.first));
        if (srcNode != nullptr) {
            for (const auto& dstLabelPair : srcEntry.second) {
                unsigned dst = dstLabelPair.first;
                
                if (dst <= UINT16_MAX) {
                    Edge e(static_cast<uint16_t>(dst));
                    srcNode->addEdge(e);
                    numEdges++;
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


