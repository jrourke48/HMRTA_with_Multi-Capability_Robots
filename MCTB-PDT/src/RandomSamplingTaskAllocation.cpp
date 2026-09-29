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
      paths(),
      optimalPath(),
      optimalMakespan(0),
      numNBAStates(nbaPtr->getNumStates()),
      prunedNBA(nullptr),
      maxIterations(maxIterations),
      Iterations(0),
      timeLimit(0.0),
      computationTime(0.0) {
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
      paths(),
      optimalPath(),
      optimalMakespan(0),
      numNBAStates(nbaPtr->getNumStates()),
      prunedNBA(nullptr),
      maxIterations(0),
      Iterations(0),
      timeLimit(timeLimit),
      computationTime(0.0) {
}

// Destructor
RandomSamplingTaskAllocation::~RandomSamplingTaskAllocation() {
    // Clean up allocated Random_Node pointers in paths
    for (auto& [acceptingState, prefixSuffix] : paths) {
        for (Random_Node* node : prefixSuffix.first.path) {
            delete node;
        }
        for (Random_Node* node : prefixSuffix.second.path) {
            delete node;
        }
    }
    
    // Clean up optimalPath
    for (Random_Node* node : optimalPath) {
        delete node;
    }
    
    // Clean up pruned NBA if it was created
    if (prunedNBA != nullptr) {
        delete prunedNBA;
    }
    
    // BuchiAutomaton, Environment, and MultiRobotSystem are managed elsewhere
}

// Run the random sampling task allocation algorithm
void RandomSamplingTaskAllocation::run() {
    pruneInfeasibleNBAPaths();
    //determine whether the buchi is finite
    if (nba->isFinite()) {
        runFinitePathPlanner();
    } else {
        runInfinitePathPlanner();
    }
}

void RandomSamplingTaskAllocation::runFinitePathPlanner() {
    // Use pruned NBA for path planning
    BuchiAutomaton* searchNBA = (prunedNBA != nullptr) ? prunedNBA : nba;
    // Get the accepting states and initial state from the NBA
    std::vector<uint16_t> acceptingStates = searchNBA->getAcceptingStates();
    // Initialize a vector to store the minimum length paths for each accepting state
    std::vector<uint16_t> minlengthPath = std::vector<uint16_t>(acceptingStates.size(), 0);
    uint16_t initialState = searchNBA->getInitialState();
    
    // Store minimum length paths for each accepting state
    uint16_t i = 0;
    for (uint16_t acceptingState : acceptingStates) {
        //get the minimum length path from initial state to accepting state
        TaskAllocPath minPath = getMinLengthPath(searchNBA->getNode(initialState), searchNBA->getNode(acceptingState));
        minlengthPath[i] = minPath.pathLength;
        addNoSufPath(acceptingState, minPath);
        i++;
    }
    
    // Start the timer for computation time measurement for the random iterative search
    clock_t startTime = clock();
    while (Iterations < maxIterations && computationTime < timeLimit) {
        // Iterate through each accepting state
        uint16_t i = 0;
        for (uint16_t acceptingState : acceptingStates) {
            uint16_t minLength = minlengthPath[i];
            i++;

            // Pick a random target path length (number of nodes) in between [minLength, numNBAStates]
            uint16_t targetLength = minLength;
            if (numNBAStates > minLength) {
                targetLength = minLength + std::rand() % (numNBAStates - minLength + 1);
            }

            // Incrementally build a random path from the initial state to the accepting state
            Node* goalNode = searchNBA->getNode(acceptingState);
            Node* curNode = searchNBA->getNode(initialState);
            std::vector<Random_Node*> newPath;
            newPath.push_back(new Random_Node(0, curNode,
                std::vector<uint16_t>(),
                std::vector<std::vector<uint8_t>>(),
                std::vector<uint16_t>(multiRobotSystem->getNumRobots(), 0)));
            uint16_t newMakespan = 0;
            // Current best path for this accepting state, used to stop early if the new path is already worse
            TaskAllocPath& bestPath = paths[acceptingState].first;

            while (curNode->getId() != acceptingState) {
                // Candidate next nodes must still reach the goal within the remaining length
                uint16_t remaining = targetLength - newPath.size();
                std::vector<Node*> candidates;
                for (const auto& edge : curNode->getEdges()) {
                    Node* nextNode = searchNBA->getNode(edge.getDstId());
                    if (!nextNode) continue;
                    uint16_t distToGoal = getMinLengthPath(nextNode, goalNode).pathLength;
                    if (distToGoal != 0 && distToGoal <= remaining) {
                        candidates.push_back(nextNode);
                    }
                }
                if (candidates.empty()) break;

                // Step to a random candidate with a random feasible task allocation
                Node* nextNode = candidates[std::rand() % candidates.size()];
                auto [robotsByAP, satisfiedTrueAPs] = getRandomFeasibleTaskAllocation(curNode, nextNode);
                // Placeholder times, should compute actual times //TO DO COMPUTE ACTUAL TIMES
                std::vector<uint16_t> curtimes(multiRobotSystem->getNumRobots(), 0);
                Random_Node* newRandomNode = new Random_Node(newPath.size(), nextNode, satisfiedTrueAPs, robotsByAP, curtimes);
                newPath.back()->setNext(newRandomNode);
                newPath.push_back(newRandomNode);
                newMakespan += newRandomNode->getCurmakespan();
                curNode = nextNode;

                // Stop building once the partial path can no longer beat the best makespan
                if (!bestPath.path.empty() && newMakespan > bestPath.makespan) break;
            }

            // Keep the new path only if it reached the goal with a better makespan
            bool reachedGoal = curNode->getId() == acceptingState;
            if (reachedGoal && (bestPath.path.empty() || newMakespan < bestPath.makespan)) {
                for (Random_Node* node : bestPath.path) delete node;
                bestPath = TaskAllocPath{static_cast<uint16_t>(newPath.size()), newPath, newMakespan};
            } else {
                for (Random_Node* node : newPath) delete node;
            }
        }
        
        // Increment the iteration counter
        Iterations++;
        computationTime = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
    }
}

void RandomSamplingTaskAllocation::runInfinitePathPlanner() {
    // Use pruned NBA for path planning
    BuchiAutomaton* searchNBA = (prunedNBA != nullptr) ? prunedNBA : nba;
    // Get the accepting states and initial state from the NBA
    std::vector<uint16_t> acceptingStates = searchNBA->getAcceptingStates();
    uint16_t initialState = searchNBA->getInitialState();
    // Minimum prefix length (initial -> accepting) and minimum suffix length (accepting -> accepting cycle)
    std::vector<uint16_t> minPrefixLength(acceptingStates.size(), 0);
    std::vector<uint16_t> minSuffixLength(acceptingStates.size(), 0);

    // Store minimum prefix and suffix lengths for each accepting state
    uint16_t i = 0;
    for (uint16_t acceptingState : acceptingStates) {
        Node* acceptingNode = searchNBA->getNode(acceptingState);

        // Shortest cycle is one step to a successor plus the shortest path from that successor back
        for (const auto& edge : acceptingNode->getEdges()) {
            Node* nextNode = searchNBA->getNode(edge.getDstId());
            if (!nextNode) continue;
            uint16_t distBack = getMinLengthPath(nextNode, acceptingNode).pathLength;
            if (distBack != 0 && (minSuffixLength[i] == 0 || distBack + 1 < minSuffixLength[i])) {
                minSuffixLength[i] = distBack + 1;
            }
        }
        TaskAllocPath minSufPath{minSuffixLength[i], std::vector<Random_Node*>(), 0};

        if (acceptingState == initialState) {
            // Accepting state is the initial state, so no prefix is needed
            addNoPrePath(acceptingState, minSufPath);
        } else {
            //get the minimum length path from initial state to accepting state
            TaskAllocPath minPrePath = getMinLengthPath(searchNBA->getNode(initialState), acceptingNode);
            minPrefixLength[i] = minPrePath.pathLength;
            addPath(acceptingState, minPrePath, minSufPath);
        }
        i++;
    }

    // Start the timer for computation time measurement for the random iterative search
    clock_t startTime = clock();
    while (Iterations < maxIterations && computationTime < timeLimit) {
        // Iterate through each accepting state
        uint16_t i = 0;
        for (uint16_t acceptingState : acceptingStates) {
            uint16_t minPrefix = minPrefixLength[i];
            uint16_t minSuffix = minSuffixLength[i];
            i++;

            // Skip accepting states that are not on a cycle or are unreachable from the initial state
            bool needsPrefix = acceptingState != initialState;
            if (minSuffix == 0 || (needsPrefix && minPrefix == 0)) continue;

            Node* acceptingNode = searchNBA->getNode(acceptingState);
            // Current best prefix and suffix for this accepting state, used to stop early if the new path is already worse
            TaskAllocPath& bestPrefix = paths[acceptingState].first;
            TaskAllocPath& bestSuffix = paths[acceptingState].second;
            bool hasBest = !bestSuffix.path.empty();
            uint16_t bestMakespan = bestPrefix.makespan + bestSuffix.makespan;

            // Build the prefix from the initial state to the accepting state
            TaskAllocPath newPrefix{0, std::vector<Random_Node*>(), 0};
            if (needsPrefix) {
                // Pick a random target prefix length (number of nodes) in between [minPrefix, numNBAStates]
                uint16_t targetLength = minPrefix;
                if (numNBAStates > minPrefix) {
                    targetLength = minPrefix + std::rand() % (numNBAStates - minPrefix + 1);
                }
                newPrefix = buildRandomPath(searchNBA, searchNBA->getNode(initialState), acceptingNode, targetLength, hasBest, bestMakespan);
                // Prefix failed to reach the accepting state or is already worse than the best
                if (newPrefix.path.empty()) continue;
            }

            // Pick a random target suffix length (number of nodes) in between [minSuffix, numNBAStates + 1]
            // A simple cycle can visit every state once and then return to the start
            uint16_t targetLength = minSuffix;
            if (numNBAStates + 1 > minSuffix) {
                targetLength = minSuffix + std::rand() % (numNBAStates + 1 - minSuffix + 1);
            }
            // Build the suffix cycle from the accepting state back to itself, bounded by what is left after the prefix
            TaskAllocPath newSuffix = buildRandomPath(searchNBA, acceptingNode, acceptingNode, targetLength, hasBest, bestMakespan - newPrefix.makespan);
            if (newSuffix.path.empty()) {
                for (Random_Node* node : newPrefix.path) delete node;
                continue;
            }

            // Keep the new prefix and suffix only if their combined makespan is better
            uint16_t newMakespan = newPrefix.makespan + newSuffix.makespan;
            if (!hasBest || newMakespan < bestMakespan) {
                for (Random_Node* node : bestPrefix.path) delete node;
                for (Random_Node* node : bestSuffix.path) delete node;
                bestPrefix = newPrefix;
                bestSuffix = newSuffix;
            } else {
                for (Random_Node* node : newPrefix.path) delete node;
                for (Random_Node* node : newSuffix.path) delete node;
            }
        }

        // Increment the iteration counter
        Iterations++;
        computationTime = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
    }
}

// Incrementally build a random path from srcNode to goalNode with at most targetLength nodes
// If srcNode == goalNode this builds a cycle (at least one step is taken)
// Returns an empty path if the goal is not reached or the makespan exceeds makespanBound (when hasBound is true)
RandomSamplingTaskAllocation::TaskAllocPath RandomSamplingTaskAllocation::buildRandomPath(
    BuchiAutomaton* searchNBA, Node* srcNode, Node* goalNode, uint16_t targetLength, bool hasBound, uint16_t makespanBound) {
    Node* curNode = srcNode;
    std::vector<Random_Node*> newPath;
    newPath.push_back(new Random_Node(0, curNode,
        std::vector<uint16_t>(),
        std::vector<std::vector<uint8_t>>(),
        std::vector<uint16_t>(multiRobotSystem->getNumRobots(), 0)));
    uint16_t newMakespan = 0;
    bool reachedGoal = false;

    while (newPath.size() < targetLength) {
        // Candidate next nodes must still reach the goal within the remaining length
        uint16_t remaining = targetLength - newPath.size();
        std::vector<Node*> candidates;
        for (const auto& edge : curNode->getEdges()) {
            Node* nextNode = searchNBA->getNode(edge.getDstId());
            if (!nextNode) continue;
            uint16_t distToGoal = getMinLengthPath(nextNode, goalNode).pathLength;
            if (distToGoal != 0 && distToGoal <= remaining) {
                candidates.push_back(nextNode);
            }
        }
        if (candidates.empty()) break;

        // Step to a random candidate with a random feasible task allocation
        Node* nextNode = candidates[std::rand() % candidates.size()];
        auto [robotsByAP, satisfiedTrueAPs] = getRandomFeasibleTaskAllocation(curNode, nextNode);
        // Placeholder times, should compute actual times //TO DO COMPUTE ACTUAL TIMES
        std::vector<uint16_t> curtimes(multiRobotSystem->getNumRobots(), 0);
        Random_Node* newRandomNode = new Random_Node(newPath.size(), nextNode, satisfiedTrueAPs, robotsByAP, curtimes);
        newPath.back()->setNext(newRandomNode);
        newPath.push_back(newRandomNode);
        newMakespan += newRandomNode->getCurmakespan();
        curNode = nextNode;

        // Stop building once the partial path can no longer beat the best makespan
        if (hasBound && newMakespan > makespanBound) break;

        if (curNode->getId() == goalNode->getId()) {
            reachedGoal = true;
            break;
        }
    }

    if (!reachedGoal || (hasBound && newMakespan > makespanBound)) {
        for (Random_Node* node : newPath) delete node;
        return TaskAllocPath{0, std::vector<Random_Node*>(), 0};
    }
    return TaskAllocPath{static_cast<uint16_t>(newPath.size()), newPath, newMakespan};
}

// Get a minimum length (fewest nodes) path from srcNode to goalNode using BFS
RandomSamplingTaskAllocation::TaskAllocPath RandomSamplingTaskAllocation::getMinLengthPath(Node* srcNode, Node* goalNode) {
    if (!srcNode || !goalNode) {
        return TaskAllocPath{0, std::vector<Random_Node*>(), 0};
    }
    
    if (srcNode->getId() == goalNode->getId()) {
        // Source is already the goal
        return TaskAllocPath{1, {}, 0};
    }
    
    // Search the pruned NBA so only feasible edges are followed
    BuchiAutomaton* searchNBA = (prunedNBA != nullptr) ? prunedNBA : nba;

    // BFS to find shortest path
    std::queue<std::pair<Node*, std::vector<Node*>>> q;
    std::set<uint16_t> visited;
    
    q.push({srcNode, {srcNode}});
    visited.insert(srcNode->getId());
    
    while (!q.empty()) {
        auto [currentNode, path] = q.front();
        q.pop();
        
        // Check all outgoing edges
        for (const auto& edge : currentNode->getEdges()) {
            Node* nextNode = searchNBA->getNode(edge.getDstId());
            if (!nextNode) continue;
            
            uint16_t nextNodeId = nextNode->getId();
            
            // Found goal
            if (nextNodeId == goalNode->getId()) {
                std::vector<Node*> completePath = path;
                completePath.push_back(nextNode);
                
                // Create TaskAllocPath with empty Random_Node vector (will be filled during execution)
                TaskAllocPath resultPath;
                resultPath.pathLength = completePath.size();
                resultPath.path = std::vector<Random_Node*>();
                resultPath.makespan = 0;
                return resultPath;
            }
            
            // Visit unvisited neighbors
            if (visited.find(nextNodeId) == visited.end()) {
                visited.insert(nextNodeId);
                std::vector<Node*> newPath = path;
                newPath.push_back(nextNode);
                q.push({nextNode, newPath});
            }
        }
    }
    
    // No path found
    return TaskAllocPath{0, std::vector<Random_Node*>(), 0};
}

//prune the NBA to remove infeasible edges based on the robot capabilities and the task requirements
void RandomSamplingTaskAllocation::pruneInfeasibleNBAPaths() {
    // Create a deep copy of the NBA to avoid modifying the original
    prunedNBA = new BuchiAutomaton(*nba);
    
    // Iterate through all edges in the pruned NBA and remove edges that cannot be satisfied
    // by any robot combination in the system
    const auto& nodeMap = prunedNBA->getNodes();
    
    for (const auto& [nodeId, node] : nodeMap) {
        if (!node) continue;
        
        std::vector<Edge> validEdges;
        for (const auto& edge : node->getEdges()) {
            // Get the true APs for this edge
            std::vector<std::vector<uint16_t>> trueAPs = prunedNBA->getTrueAPs(node->getId(), edge.getDstId());
            
            // Check if any AP set can be satisfied
            bool edgeIsFeasible = false;
            for (const auto& apSet : trueAPs) {
                // Try to find a feasible allocation for this AP set
                std::vector<std::vector<uint8_t>> randomAllocation = getRandomAllocation(apSet);
                bool apSetFeasible = true;
                
                for (size_t apIdx = 0; apIdx < apSet.size() && apSetFeasible; ++apIdx) {
                    uint16_t ap = apSet[apIdx];
                    std::vector<bool> requiredCapabilities = prunedNBA->getLTLFormula()->getRequiredCapabilities(ap);
                    std::vector<uint8_t> assignedRobots = randomAllocation[apIdx];
                    
                    // Check if assigned robots can satisfy required capabilities
                    std::vector<bool> combinedCapabilities(requiredCapabilities.size(), false);
                    for (uint8_t robotId : assignedRobots) {
                        std::vector<bool> roboCaps = multiRobotSystem->getRobotCapabilities(robotId + 1);
                        for (size_t j = 0; j < roboCaps.size() && j < combinedCapabilities.size(); ++j) {
                            combinedCapabilities[j] = combinedCapabilities[j] || roboCaps[j];
                        }
                    }
                    
                    // Verify all required capabilities are met
                    for (size_t j = 0; j < requiredCapabilities.size(); ++j) {
                        if (requiredCapabilities[j] && (j >= combinedCapabilities.size() || !combinedCapabilities[j])) {
                            apSetFeasible = false;
                            break;
                        }
                    }
                }
                
                if (apSetFeasible) {
                    edgeIsFeasible = true;
                    break;
                }
            }
            
            if (edgeIsFeasible) {
                validEdges.push_back(edge);
            }
        }
        
        // Update pruned NBA node with only valid edges
        node->setEdges(validEdges);
    }
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

// gets a complete random allocation of robots to the given set of APs
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
BuchiAutomaton* RandomSamplingTaskAllocation::getPrunedNBA() const {
    return prunedNBA;
}


Environment* RandomSamplingTaskAllocation::getEnvironment() const {
    return environment;
}

MultiRobotSystem* RandomSamplingTaskAllocation::getMultiRobotSystem() const {
    return multiRobotSystem;
}

//========================
// PATHS GETTERS & ADDERS
//========================

const std::pair<RandomSamplingTaskAllocation::TaskAllocPath, RandomSamplingTaskAllocation::TaskAllocPath>& RandomSamplingTaskAllocation::getPath(uint16_t acceptingState) const {
    static const std::pair<TaskAllocPath, TaskAllocPath> empty{};
    auto it = paths.find(acceptingState);
    return (it != paths.end()) ? it->second : empty;
}

const std::map<uint16_t, std::pair<RandomSamplingTaskAllocation::TaskAllocPath, RandomSamplingTaskAllocation::TaskAllocPath>>& RandomSamplingTaskAllocation::getAllPaths() const {
    return paths;
}

//========================
// OPTIMAL PATH GETTERS & SETTERS
//========================

const std::vector<Random_Node*>& RandomSamplingTaskAllocation::getOptimalPath() const {
    return optimalPath;
}

void RandomSamplingTaskAllocation::setOptimalPath(const std::vector<Random_Node*>& path) {
    optimalPath = path;
}

//========================
// OPTIMAL MAKESPAN GETTERS & SETTERS
//========================

uint16_t RandomSamplingTaskAllocation::getOptimalMakespan() const {
    return optimalMakespan;
}

void RandomSamplingTaskAllocation::setOptimalMakespan(uint16_t makespan) {
    optimalMakespan = makespan;
}

//========================
// NUMBER OF NBA STATES GETTERS & SETTERS
//========================

uint16_t RandomSamplingTaskAllocation::getNumNBAStates() const {
    return numNBAStates;
}

void RandomSamplingTaskAllocation::setNumNBAStates(uint16_t num) {
    numNBAStates = num;
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

//========================
// PATH MANAGEMENT
//========================

void RandomSamplingTaskAllocation::addPath(uint16_t acceptingState, TaskAllocPath prePath, TaskAllocPath sufPath) {
    paths[acceptingState] = {prePath, sufPath};
}

void RandomSamplingTaskAllocation::addNoPrePath(uint16_t acceptingState, TaskAllocPath sufPath) {
    // Create an empty prefix path (no nodes, zero length, zero makespan)
    TaskAllocPath emptyPrePath{0, std::vector<Random_Node*>(), 0};
    paths[acceptingState] = {emptyPrePath, sufPath};
}

void RandomSamplingTaskAllocation::addNoSufPath(uint16_t acceptingState, TaskAllocPath prePath) {
    // Create an empty suffix path (no nodes, zero length, zero makespan)
    TaskAllocPath emptySufPath{0, std::vector<Random_Node*>(), 0};
    paths[acceptingState] = {prePath, emptySufPath};
}
