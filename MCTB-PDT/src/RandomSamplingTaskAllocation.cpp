#include "RandomSamplingTaskAllocation.h"
#include <algorithm>
#include <map>

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
    std::clock_t startTime = std::clock();
    while(Iterations < maxIterations && computationTime < timeLimit) {
        // Perform one iteration of random sampling task allocation
        uint16_t i = 0;
        for (const std::vector<uint16_t>& scc : AcceptingSCCs) {
            // Perform random sampling for this SCC
            uint16_t j = 0;
            for (uint16_t nodeId : scc) {
                // Access the current node in the SCC using nodeId
                Node* curNode = nba->getNode(nodeId);
                Node* newNode = nba->getNode(scc[j+1]);
                // Get a random feasible task allocation for the transition from curNode to newNode
                auto [robotsByAP, satisfiedTrueAPs] = getRandomFeasibleTaskAllocation(curNode, newNode);

                // Perform random sampling for this node in the SCC
                if (Iterations == 0) {
                    // Initialize the path for this SCC if it's the first iteration


                }


            }

        }

        // Increment the iteration counter
        Iterations++;
        computationTime = static_cast<double>(std::clock() - startTime) / CLOCKS_PER_SEC;
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
    std::vector<std::vector<int>> adj = createAdjacencyList();

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
std::vector<std::vector<int>> RandomSamplingTaskAllocation::createAdjacencyList() {
    const auto& nodeMap = nba->getNodes();
    // Create mapping from node IDs to indices for adjacency list
    std::map<uint16_t, int> nodeIdToIndex;
    std::vector<uint16_t> indexToNodeId;
    int idx = 0;
    for (const auto& pair : nodeMap) {
        nodeIdToIndex[pair.first] = idx;
        indexToNodeId.push_back(pair.first);
        idx++;
    }

    int n = indexToNodeId.size();

    // Build adjacency list from Büchi automaton edges
    std::vector<std::vector<int>> adj(n);
    for (int i = 0; i < n; ++i) {
        uint16_t nodeId = indexToNodeId[i];
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
    std::vector<std::vector<uint16_t>> trueAPs = buchiPtr->getTrueAPs(curNode->getId(), newNode->getId());
    bool isFeasible = false;
    std::vector<std::vector<uint8_t>> robotsByAP;
    std::vector<uint16_t> satisfiedApSet;
    
    // Keep trying until a feasible allocation is found
    while (!isFeasible) {
        for (const auto& apSet : trueAPs) {
            robotsByAP.clear();
            robotsByAP.resize(apSet.size());
            
            // For each AP in this conjunction, allocate robots to satisfy it
            bool allAPsSatisfied = true;
            for (size_t apIdx = 0; apIdx < apSet.size(); ++apIdx) {
                uint16_t ap = apSet[apIdx];
                std::vector<bool> requiredCapabilities = buchiPtr->getLTLFormula()->getRequiredCapabilities(ap);
                
                // Find robots that can satisfy this AP
                std::vector<uint8_t> capableRobots;
                for (uint8_t i = 0; i < multiRobotSystem->getNumRobots(); ++i) {
                    std::vector<bool> roboCaps = mrsPtr->getRobotCapabilities(i + 1);  // Convert 0-based index to 1-based robot ID
                    
                    // Check if this robot's capabilities satisfy required capabilities (superset)
                    bool canSatisfy = true;
                    for (size_t j = 0; j < requiredCapabilities.size(); ++j) {
                        if (requiredCapabilities[j] && (j >= roboCaps.size() || !roboCaps[j])) {
                            canSatisfy = false;
                            break;
                        }
                    }
                    if (canSatisfy) {
                        capableRobots.push_back(i);
                    }
                }
                
                // If no capable robots found, this AP set cannot be satisfied
                if (capableRobots.empty()) {
                    allAPsSatisfied = false;
                    break;
                }
                
                // Randomly select one robot from capable robots to satisfy this AP
                uint8_t selectedRobot = capableRobots[std::rand() % capableRobots.size()];
                robotsByAP[apIdx].push_back(selectedRobot);
            }
            
            // If all APs in this conjunction can be satisfied, this conjunction is feasible
            if (allAPsSatisfied) {
                satisfiedApSet = apSet;
                isFeasible = true;
                break;
            }
        }
    }
    
    return std::make_pair(robotsByAP, satisfiedApSet);
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

const std::vector<std::vector<Tree_Node*>>& RandomSamplingTaskAllocation::getPaths() const {
    return path;
}

void RandomSamplingTaskAllocation::setPaths(const std::vector<std::vector<Tree_Node*>>& paths) {
    path = paths;
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
