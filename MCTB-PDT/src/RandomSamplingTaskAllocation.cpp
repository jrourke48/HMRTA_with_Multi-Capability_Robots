#include "RandomSamplingAlgo/RandomSamplingTaskAllocation.h"
#include "RandomSamplingAlgo/RandomNode.h"
#include <algorithm>
#include <map>
#include <ctime>

//========================
// CONSTRUCTORS & DESTRUCTOR
//========================

// Constructor with iteration parameter
RandomSamplingTaskAllocation::RandomSamplingTaskAllocation(
    BuchiAutomaton* nbaPtr, 
    Environment* envPtr, 
    MultiRobotSystem* robotSysPtr, 
    uint16_t maxIterations)
    : nba(nbaPtr), 
      environment(envPtr), 
      multiRobotSystem(robotSysPtr),
      maxIterations(maxIterations),
      Iterations(0),
      timeLimit(0.0),
      computationTime(0.0),
      numSCCs(0) {
}

// Constructor with timelimit parameter
RandomSamplingTaskAllocation::RandomSamplingTaskAllocation(
    BuchiAutomaton* nbaPtr, 
    Environment* envPtr, 
    MultiRobotSystem* robotSysPtr, 
    double timeLimit)
    : nba(nbaPtr), 
      environment(envPtr), 
      multiRobotSystem(robotSysPtr),
      timeLimit(timeLimit),
      maxIterations(0),
      Iterations(0),
      computationTime(0.0),
      numSCCs(0) {
}

// Run the random sampling task allocation algorithm
void RandomSamplingTaskAllocation::run() {
    // Extract accepting SCCs from the Büchi automaton before starting the iterations
    setAcceptingSCCs();
    clock_t startTime = clock();
    while(Iterations < maxIterations && computationTime < timeLimit) {
        // Perform one iteration of random sampling task allocation
        uint16_t i = 0;
        for (const std::vector<uint16_t>& scc : AcceptingSCCs) {
            // Perform random sampling for this SCC
            if (Iterations == 0) {
                // Initialize the start node for this SCC
                Random_Node* startNode = new Random_Node(0, nba->getNode(scc[0]), std::vector<uint16_t>(1, 0), std::vector<std::vector<uint8_t>>(1, std::vector<uint8_t>(multiRobotSystem->getNumRobots(), 0)), std::vector<uint16_t>(multiRobotSystem->getNumRobots(), 0));
            }
            for (uint16_t j = 1; j < scc.size(); j++) {
                // Access the current node in the SCC using nodeId
                Node* prevNode = nba->getNode(scc[j-1]);
                Node* curNode = nba->getNode(scc[j]);
                // Get a random feasible task allocation for the transition from curNode to newNode
                auto [robotsByAP, satisfiedTrueAPs] = getRandomFeasibleTaskAllocation(prevNode, curNode);
                //times to goal for each robot (using default time)
                std::vector<uint16_t> curtimes(multiRobotSystem->getNumRobots(), 0);

                // Perform random sampling for this node in the SCC
                if (Iterations == 0) {
                    // Initialize the path for this SCC if it's the first iteration
                    Random_Node* newRandomNode = new Random_Node(j, curNode, satisfiedTrueAPs, robotsByAP, curtimes);
                }



            }

        }

        // Increment the iteration counter
        Iterations++;
        computationTime = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
    }
}

// Destructor
RandomSamplingTaskAllocation::~RandomSamplingTaskAllocation() {
    // Member variables are not owned by this class, so no cleanup needed
    // BuchiAutomaton, Environment, and MultiRobotSystem are managed elsewhere
}

//
// set Accepting SCCs - Extract accepting SCCs from Büchi automaton using Tarjan's algorithm
//
void RandomSamplingTaskAllocation::setAcceptingSCCs() {
    if (!nba) {
        AcceptingSCCs.clear();
        numSCCs = 0;
        return;
    }

    // Get all nodes and accepting states from Büchi automaton
    const auto& nodeMap = nba->getNodes();
    const auto& acceptingStates = nba->getAcceptingStates();

    if (nodeMap.empty()) {
        AcceptingSCCs.clear();
        numSCCs = 0;
        return;
    }
    // Create adjacency list for the NBA
    std::vector<uint16_t> indexToNodeId;
    std::vector<std::vector<int>> adj = createAdjacencyList(indexToNodeId);

    // Get all SCCs using Tarjan's algorithm
    std::vector<std::vector<int>> allSCCs = getSCCs(adj);

    // Filter SCCs to keep only those containing accepting states
    AcceptingSCCs.clear();
    for (const auto& scc : allSCCs) {
        // Check if this SCC contains any accepting state
        bool hasAcceptingState = false;
        for (int idx_val : scc) {
            uint16_t nodeId = indexToNodeId[idx_val];
            if (std::find(acceptingStates.begin(), acceptingStates.end(), nodeId) != acceptingStates.end()) {
                hasAcceptingState = true;
                break;
            }
        }

        // If this SCC has an accepting state, add it to AcceptingSCCs
        if (hasAcceptingState) {
            std::vector<uint16_t> acceptingSCC;
            for (int idx_val : scc) {
                acceptingSCC.push_back(indexToNodeId[idx_val]);
            }
            AcceptingSCCs.push_back(acceptingSCC);
        }
    }

    // Update numSCCs
    numSCCs = AcceptingSCCs.size();
}

//create adjacency list for the NBA
std::vector<std::vector<int>> RandomSamplingTaskAllocation::createAdjacencyList(std::vector<uint16_t>& outIndexToNodeId) {
    const auto& nodeMap = nba->getNodes();
    // Create mapping from node IDs to indices for adjacency list
    std::map<uint16_t, int> nodeIdToIndex;
    outIndexToNodeId.clear();
    int idx = 0;
    for (const auto& pair : nodeMap) {
        nodeIdToIndex[pair.first] = idx;
        outIndexToNodeId.push_back(pair.first);
        idx++;
    }

    int n = outIndexToNodeId.size();

    // Build adjacency list from Büchi automaton edges
    std::vector<std::vector<int>> adj(n);
    for (int i = 0; i < n; ++i) {
        uint16_t nodeId = outIndexToNodeId[i];
        Node* node = nba->getNode(nodeId);
        if (node) {
            const auto& edges = node->getEdges();
            for (const auto& edge : edges) {
                uint16_t dstId = edge.getDstId();
                if (nodeIdToIndex.find(dstId) != nodeIdToIndex.end()) {
                    int dstIdx = nodeIdToIndex[dstId];
                    adj[i].push_back(dstIdx);
                }
            }
        }
    }
    return adj;
}

//get a random feasible task allocation for the robots based on the trueAPs and destination product states
//returns a pair of (robotsByAP, satisfiedTrueAPs)
//robotsByAP[i] = vector of robot indices assigned to satisfy apSet[i]
//satisfiedTrueAPs = the AP set (conjunction) that was satisfied
//
//Example: If satisfiedTrueAPs = [5, 7] (AP 5 AND AP 7 must be satisfied)
//         and robotsByAP = [[0, 1], [2]]
//         Then: robots 0,1 are assigned to satisfy AP 5
//               robot 2 is assigned to satisfy AP 7
//
std::pair<std::vector<std::vector<uint8_t>>, std::vector<uint16_t>> RandomSamplingTaskAllocation::getRandomFeasibleTaskAllocation(Node* curNode, Node* newNode) {
    //need to get a vector of all possible true ap sets from all the edges to the new node
    std::vector<std::vector<uint16_t>> trueAPs = nba->getTrueAPs(curNode->getId(), newNode->getId());
    bool isFeasible = false;
    std::vector<std::vector<uint8_t>> robotsByAP;
    std::vector<uint16_t> satisfiedApSet;
    
    // Keep trying until a feasible allocation is found
    while (!isFeasible) {
        for (const auto& apSet : trueAPs) {
            robotsByAP.clear();
            robotsByAP.resize(apSet.size());
            //need to get a random allocation of robots for each ap in the set
            std::vector<std::vector<uint8_t>> randomAllocation = getRandomAllocation(apSet);
            // For each AP in this conjunction, allocate robots to satisfy it
            bool allAPsSatisfied = true;
            for (size_t apIdx = 0; apIdx < apSet.size(); ++apIdx) {
                uint16_t ap = apSet[apIdx];
                std::vector<bool> requiredCapabilities = nba->getLTLFormula()->getRequiredCapabilities(ap);
               
                // Get the robots assigned to this AP
                std::vector<uint8_t> assignedRobots = randomAllocation[apIdx];
                
                // Combine the capabilities of all robots assigned to this AP using OR
                std::vector<bool> combinedCapabilities(requiredCapabilities.size(), false);
                for (uint8_t robotId : assignedRobots) {
                    std::vector<bool> roboCaps = multiRobotSystem->getRobotCapabilities(robotId + 1);  // Convert 0-based index to 1-based robot ID
                    
                    // OR the robot's capabilities with the combined capabilities
                    for (size_t j = 0; j < roboCaps.size() && j < combinedCapabilities.size(); ++j) {
                        combinedCapabilities[j] = combinedCapabilities[j] || roboCaps[j];
                    }
                }
                
                // Check if all required capabilities are satisfied by the combined capabilities
                bool canSatisfy = true;
                for (size_t j = 0; j < requiredCapabilities.size(); ++j) {
                    if (requiredCapabilities[j] && (j >= combinedCapabilities.size() || !combinedCapabilities[j])) {
                        canSatisfy = false;
                        break;
                    }
                }
                
                // If this AP cannot be satisfied, the entire set is not feasible
                if (!canSatisfy) {
                    allAPsSatisfied = false;
                    break;
                }
            }
            
            // If all APs in this conjunction can be satisfied, this conjunction is feasible
            if (allAPsSatisfied) {
                satisfiedApSet = apSet;
                robotsByAP = randomAllocation;
                isFeasible = true;
                break;
            }
        }
    }
    
    return std::make_pair(robotsByAP, satisfiedApSet);
}
std::vector<std::vector<uint8_t>> RandomSamplingTaskAllocation::getRandomAllocation(std::vector<uint16_t> apSet) {
    // Get the number of robots in the system
    uint8_t numRobots = multiRobotSystem->getNumRobots();
    
    // Initialize a vector to hold robot assignments for each AP
    std::vector<std::vector<uint8_t>> robotsByAP(apSet.size());
    
    // Randomly assign each robot to one of the APs
    for (uint8_t robotId = 0; robotId < numRobots; ++robotId) {
        // Randomly select an AP for this robot
        uint16_t randomApIndex = std::rand() % apSet.size();
        if (std::rand() % 2 == 0) {  // 50% chance to assign this robot to the selected AP
            robotsByAP[randomApIndex].push_back(robotId);
        }
    }
    return robotsByAP;
}

//========================
// SYSTEM COMPONENT SETTERS
//========================

void RandomSamplingTaskAllocation::setNBA(BuchiAutomaton* nbaPtr) {
    nba = nbaPtr;
}

void RandomSamplingTaskAllocation::setEnvironment(Environment* envPtr) {
    environment = envPtr;
}

void RandomSamplingTaskAllocation::setMultiRobotSystem(MultiRobotSystem* robotSysPtr) {
    multiRobotSystem = robotSysPtr;
}

//========================
// SYSTEM COMPONENT GETTERS
//========================

BuchiAutomaton* RandomSamplingTaskAllocation::getNBA() const {
    return nba;
}

Environment* RandomSamplingTaskAllocation::getEnvironment() const {
    return environment;
}

MultiRobotSystem* RandomSamplingTaskAllocation::getMultiRobotSystem() const {
    return multiRobotSystem;
}

//========================
// ACCEPTING SCCS METHODS
//========================

std::vector<std::vector<uint16_t>> RandomSamplingTaskAllocation::getAcceptingSCCs() const {
    return AcceptingSCCs;
}

//========================
// PATHS GETTERS & SETTERS
//========================

const std::vector<std::vector<Random_Node*>>& RandomSamplingTaskAllocation::getPaths() const {
    return paths;
}

void RandomSamplingTaskAllocation::setPaths(const std::vector<std::vector<Random_Node*>>& newPaths) {
    paths = newPaths;
}

//========================
// OPTIMAL MAKESPANS GETTERS & SETTERS
//========================

const std::vector<uint16_t>& RandomSamplingTaskAllocation::getOptimalMakespans() const {
    return optimalMakespans;
}

void RandomSamplingTaskAllocation::setOptimalMakespans(const std::vector<uint16_t>& makespans) {
    optimalMakespans = makespans;
}

//========================
// NUMBER OF SCCS GETTERS & SETTERS
//========================

uint16_t RandomSamplingTaskAllocation::getNumSCCs() const {
    return numSCCs;
}

void RandomSamplingTaskAllocation::setNumSCCs(uint16_t num) {
    numSCCs = num;
}

//========================
// MAX ITERATIONS GETTERS & SETTERS
//========================

uint16_t RandomSamplingTaskAllocation::getMaxIterations() const {
    return maxIterations;
}

void RandomSamplingTaskAllocation::setMaxIterations(uint16_t maxIter) {
    maxIterations = maxIter;
}

//========================
// ITERATIONS GETTERS & SETTERS
//========================

uint16_t RandomSamplingTaskAllocation::getIterations() const {
    return Iterations;
}

void RandomSamplingTaskAllocation::setIterations(uint16_t iters) {
    Iterations = iters;
}

//========================
// TIME LIMIT GETTERS & SETTERS
//========================

double RandomSamplingTaskAllocation::getTimeLimit() const {
    return timeLimit;
}

void RandomSamplingTaskAllocation::setTimeLimit(double limit) {
    timeLimit = limit;
}

//========================
// COMPUTATION TIME GETTERS & SETTERS
//========================

double RandomSamplingTaskAllocation::getComputationTime() const {
    return computationTime;
}

void RandomSamplingTaskAllocation::setComputationTime(double time) {
    computationTime = time;
}
