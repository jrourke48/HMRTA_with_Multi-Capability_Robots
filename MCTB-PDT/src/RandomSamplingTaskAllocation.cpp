#include "RandomSamplingAlgo/RandomSamplingTaskAllocation.h"
#include "RandomSamplingAlgo/RandomNode.h"
#include <algorithm>
#include <map>
#include <ctime>

//========================
// CONSTRUCTORS & DESTRUCTOR
//========================

/**
 * @brief Construct a planner bounded by a number of sampling iterations.
 * @param nbaPtr        Buchi automaton for the specification; not owned.
 * @param envPtr        Environment supplying region geometry; not owned.
 * @param robotSysPtr   Robot team; not owned.
 * @param maxIterations Iterations to sample before stopping.
 * @note The time limit is left unset, so only the iteration count bounds the search.
 */
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

/**
 * @brief Construct a planner bounded by a wall clock budget.
 * @param nbaPtr      Buchi automaton for the specification; not owned.
 * @param envPtr      Environment supplying region geometry; not owned.
 * @param robotSysPtr Robot team; not owned.
 * @param timeLimit   Seconds to sample before stopping.
 * @note The iteration count is left unset, so only the time budget bounds the search.
 */
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

/**
 * @brief Release the sampled paths and the pruned automaton.
 * @note The Buchi automaton, environment and robot team belong to the caller and are left alone.
 */
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

/**
 * @brief Prune the automaton, then plan with the strategy its acceptance condition calls for.
 *
 * A finite specification needs only a prefix reaching an accepting state. An infinite one needs
 * a lasso: a prefix followed by a cycle that revisits the accepting state forever.
 */
void RandomSamplingTaskAllocation::run() {
    // Taken once, so every sampled path starts from the same configuration even if something
    // else moves the team while planning
    initialPositions = multiRobotSystem->getRobotPositions();
    pruneInfeasibleNBAPaths();
    //determine whether the buchi is finite
    if (nba->isFinite()) {
        runFinitePathPlanner();
    } else {
        runInfinitePathPlanner();
    }
}

/**
 * @brief Sample prefix paths to every accepting state, keeping the cheapest found per state.
 *
 * Each iteration draws a target length in [shortest, |Q|] and grows a random path to the
 * accepting state, abandoning it as soon as its makespan passes the incumbent. Randomising the
 * length matters because the path with fewest states is not generally the quickest one.
 * @note No suffix is stored: a finite run reaches its accepting state once and stops.
 */
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
    // A limit left at 0 is unset, so only the limit the caller gave bounds the search
    while ((maxIterations == 0 || Iterations < maxIterations) &&
           (timeLimit == 0.0 || computationTime < timeLimit)) {
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
            // Robot clocks and placements, carried along the path as teams are dispatched
            std::vector<uint16_t> times(multiRobotSystem->getNumRobots(), 0);
            std::vector<Point> positions = initialPositions;
            std::vector<Random_Node*> newPath;
            newPath.push_back(new Random_Node(0, curNode,
                std::vector<uint16_t>(),
                std::vector<std::vector<uint8_t>>(),
                times));
            newPath.back()->setPositions(positions);
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
                // Send each team to its AP's region and advance the clocks of the robots that went
                applyStep(satisfiedTrueAPs, robotsByAP, times, positions);
                Random_Node* newRandomNode = new Random_Node(newPath.size(), nextNode, satisfiedTrueAPs, robotsByAP, times);
                newRandomNode->setPositions(positions);
                newPath.back()->setNext(newRandomNode);
                newPath.push_back(newRandomNode);
                newMakespan = newRandomNode->getCurmakespan();
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

/**
 * @brief Sample lassos for every accepting state that is reachable and lies on a cycle.
 *
 * For an accepting state q the shortest cycle is 1 + dist(successor, q) and the shortest prefix
 * is dist(initial, q). Each iteration samples a prefix and then a suffix, the suffix continuing
 * from the clocks and placements the prefix left behind so the lasso is costed as one timeline.
 * A sampled pair replaces the incumbent only when it finishes sooner overall.
 * @note Accepting states that are unreachable, or lie on no cycle, are skipped entirely.
 */
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

        // Starting on the accepting state does not count as visiting it, so a run beginning there
        // still owes a first loop. Its prefix is a cycle of the same minimum length as its suffix,
        // which is what the tree planner walks for the same specification
        if (acceptingState == initialState) {
            minPrefixLength[i] = minSuffixLength[i];
        } else {
            minPrefixLength[i] = getMinLengthPath(searchNBA->getNode(initialState), acceptingNode).pathLength;
        }
        TaskAllocPath minPrePath{minPrefixLength[i], std::vector<Random_Node*>(), 0};
        addPath(acceptingState, minPrePath, minSufPath);
        i++;
    }

    // Start the timer for computation time measurement for the random iterative search
    clock_t startTime = clock();
    // A limit left at 0 is unset, so only the limit the caller gave bounds the search
    while ((maxIterations == 0 || Iterations < maxIterations) &&
           (timeLimit == 0.0 || computationTime < timeLimit)) {
        // Iterate through each accepting state
        uint16_t i = 0;
        for (uint16_t acceptingState : acceptingStates) {
            uint16_t minPrefix = minPrefixLength[i];
            uint16_t minSuffix = minSuffixLength[i];
            i++;

            // Skip accepting states that are not on a cycle or are unreachable from the initial state
            if (minSuffix == 0 || minPrefix == 0) continue;

            Node* acceptingNode = searchNBA->getNode(acceptingState);
            // Current best prefix and suffix for this accepting state, used to stop early if the new path is already worse
            TaskAllocPath& bestPrefix = paths[acceptingState].first;
            TaskAllocPath& bestSuffix = paths[acceptingState].second;
            bool hasBest = !bestSuffix.path.empty();
            uint16_t bestMakespan = bestPrefix.makespan + bestSuffix.makespan;

            // Build the prefix from the initial state to the accepting state. When the two are the
            // same state buildRandomPath still takes a transition, so this becomes the first loop
            uint16_t prefixTarget = minPrefix;
            if (numNBAStates + 1 > minPrefix) {
                prefixTarget = minPrefix + std::rand() % (numNBAStates + 1 - minPrefix + 1);
            }
            TaskAllocPath newPrefix = buildRandomPath(searchNBA, searchNBA->getNode(initialState), acceptingNode,
                                                      prefixTarget, hasBest, bestMakespan);
            // Prefix failed to reach the accepting state or is already worse than the best
            if (newPrefix.path.empty()) continue;

            // Pick a random target suffix length (number of nodes) in between [minSuffix, numNBAStates + 1]
            // A simple cycle can visit every state once and then return to the start
            uint16_t targetLength = minSuffix;
            if (numNBAStates + 1 > minSuffix) {
                targetLength = minSuffix + std::rand() % (numNBAStates + 1 - minSuffix + 1);
            }
            // The cycle continues from wherever the prefix left the robots, in time and in place,
            // so its cost follows on from the prefix instead of restarting
            std::vector<uint16_t> handoffTimes;
            std::vector<Point> handoffPositions;
            if (!newPrefix.path.empty()) {
                handoffTimes = newPrefix.path.back()->getTimes();
                handoffPositions = newPrefix.path.back()->getPositions();
            }
            // Build the suffix cycle from the accepting state back to itself, bounded by what is left after the prefix
            TaskAllocPath newSuffix = buildRandomPath(searchNBA, acceptingNode, acceptingNode, targetLength, hasBest,
                                                      bestMakespan - newPrefix.makespan, handoffTimes, handoffPositions);
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

/**
 * @brief Grow a random path towards a goal state, allocating robots at every step.
 *
 * At each step the successors that can still reach the goal within the remaining length are
 * collected, one is chosen uniformly at random, and a team is assigned to that transition.
 * @param searchNBA      Automaton to walk, normally the pruned copy.
 * @param srcNode        State to start from; pass the goal state to build a cycle.
 * @param goalNode       State the path must end at.
 * @param targetLength   Largest number of states the path may contain.
 * @param hasBound       Whether @p makespanBound applies.
 * @param makespanBound  Abandon the path once this segment costs more than this.
 * @param startTimes     Robot clocks to continue from; empty starts them at zero.
 * @param startPositions Robot placements to continue from; empty uses their current ones.
 * @return The path and the makespan it adds, or an empty path when the goal was not reached or
 *         the bound was passed.
 * @note With @p srcNode equal to @p goalNode at least one transition is taken, so a cycle is
 *       built rather than the trivial empty path.
 */
RandomSamplingTaskAllocation::TaskAllocPath RandomSamplingTaskAllocation::buildRandomPath(
    BuchiAutomaton* searchNBA, Node* srcNode, Node* goalNode, uint16_t targetLength, bool hasBound, uint16_t makespanBound,
    const std::vector<uint16_t>& startTimes, const std::vector<Point>& startPositions) {
    Node* curNode = srcNode;

    // Robot clocks and placements this path continues from
    std::vector<uint16_t> times = startTimes.empty()
        ? std::vector<uint16_t>(multiRobotSystem->getNumRobots(), 0) : startTimes;
    std::vector<Point> positions = startPositions.empty()
        ? (initialPositions.empty() ? multiRobotSystem->getRobotPositions() : initialPositions)
        : startPositions;
    // Makespan is reported as this segment's own contribution, so a prefix and its suffix add up
    uint16_t startMakespan = times.empty() ? 0 : *std::max_element(times.begin(), times.end());

    std::vector<Random_Node*> newPath;
    newPath.push_back(new Random_Node(0, curNode,
        std::vector<uint16_t>(),
        std::vector<std::vector<uint8_t>>(),
        times));
    newPath.back()->setPositions(positions);
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
        // Send each team to its AP's region and advance the clocks of the robots that went
        applyStep(satisfiedTrueAPs, robotsByAP, times, positions);
        Random_Node* newRandomNode = new Random_Node(newPath.size(), nextNode, satisfiedTrueAPs, robotsByAP, times);
        newRandomNode->setPositions(positions);
        newPath.back()->setNext(newRandomNode);
        newPath.push_back(newRandomNode);
        newMakespan = newRandomNode->getCurmakespan() - startMakespan;
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

/**
 * @brief Breadth-first search for the fewest states between two automaton states.
 * @param srcNode  State to start from.
 * @param goalNode State to reach.
 * @return A path whose @c pathLength counts the states on a shortest route, 1 when the two are
 *         the same state, or 0 when the goal is unreachable or either argument is null.
 * @note Walks the pruned automaton when one exists, so only feasible transitions are counted.
 */
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

/**
 * @brief Build a copy of the automaton with every transition the team cannot execute removed.
 *
 * A transition survives when disjoint teams can be assigned to all propositions of at least one
 * of its conjunctions. That is exactly the question the allocator answers later, so a surviving
 * transition can always be allocated during sampling.
 * @note The copy owns its own states, leaving the original automaton untouched. Pruning a second
 *       time discards the previous copy.
 */
void RandomSamplingTaskAllocation::pruneInfeasibleNBAPaths() {
    // Create a deep copy of the NBA to avoid modifying the original.
    // Pruning again replaces the previous copy, which owns its own nodes
    delete prunedNBA;
    prunedNBA = new BuchiAutomaton(*nba);
    
    // Iterate through all edges in the pruned NBA and remove edges that cannot be satisfied
    // by any robot combination in the system
    const auto& nodeMap = prunedNBA->getNodes();

    for (const auto& [nodeId, node] : nodeMap) {
        if (!node) continue;
        
        std::vector<Edge> validEdges;
        for (const auto& edge : node->getEdges()) {
            // Read the AP sets off the edge itself, not through BuchiAutomaton::getTrueAPs,
            // so multi-AP conjunctions survive for the team to satisfy together
            std::vector<std::vector<uint16_t>> trueAPs = edge.getTrueAPs();

            // An edge is feasible when disjoint teams can be assigned to every AP of at least one
            // of its conjunctions. This is the same question the allocator answers when the path
            // builder takes the edge, so an edge kept here can always be allocated later
            bool edgeIsFeasible = false;
            std::vector<std::vector<uint8_t>> teams;
            for (const auto& apSet : trueAPs) {
                if (assignTeamsToConjunction(apSet, teams)) {
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

/**
 * @brief Assign robots to the propositions discharged by one transition.
 * @param curNode Source automaton state.
 * @param newNode Destination automaton state.
 * @return The teams, one per proposition, together with the conjunction they satisfy. Both come
 *         back empty when no conjunction of the transition can be staffed.
 *
 * Example: a returned conjunction of [5, 7] with teams [[1, 2], [3]] puts robots 1 and 2 on
 * proposition 5 and robot 3 on proposition 7.
 * @note AP sets are read off the transitions themselves rather than through
 *       BuchiAutomaton::getTrueAPs, which discards multi-proposition conjunctions. The automaton
 *       is nondeterministic, so every transition joining the two states offers its own options.
 */
std::pair<std::vector<std::vector<uint8_t>>, std::vector<uint16_t>> RandomSamplingTaskAllocation::getRandomFeasibleTaskAllocation(Node* curNode, Node* newNode) {
    // Read the AP sets off the edges themselves, not through BuchiAutomaton::getTrueAPs,
    // so multi-AP conjunctions survive. The NBA is nondeterministic, so every edge between
    // these two states contributes its own options
    std::vector<std::vector<uint16_t>> trueAPs;
    for (const Edge& edge : curNode->getEdgestoNode(newNode->getId())) {
        for (const auto& apSet : edge.getTrueAPs()) {
            trueAPs.push_back(apSet);
        }
    }
    std::vector<std::vector<uint8_t>> robotsByAP;
    std::vector<uint16_t> satisfiedApSet;

    // Take the first conjunction that can be staffed. The assignment is randomised inside, so
    // repeated calls on the same edge give different teams without drawing and retesting
    for (const auto& apSet : trueAPs) {
        if (assignTeamsToConjunction(apSet, robotsByAP)) {
            satisfiedApSet = apSet;
            break;
        }
    }

    return std::make_pair(robotsByAP, satisfiedApSet);
}

/**
 * @brief Advance robot clocks and placements across one step of a path.
 *
 * Every robot serving a proposition travels to that proposition's region, and the whole team is
 * charged the last arrival, since the task is not discharged until its slowest member is in
 * place. Robots serving nothing on this step neither move nor age.
 * @param apSet     Propositions discharged on this step.
 * @param teams     Robots assigned to each proposition, by 1-based id.
 * @param times     Per-robot clocks, updated in place.
 * @param positions Per-robot placements, updated in place.
 */
void RandomSamplingTaskAllocation::applyStep(const std::vector<uint16_t>& apSet,
                                             const std::vector<std::vector<uint8_t>>& teams,
                                             std::vector<uint16_t>& times,
                                             std::vector<Point>& positions) const {
    for (size_t a = 0; a < apSet.size() && a < teams.size(); ++a) {
        // The region this proposition is discharged in
        uint16_t region = nba->getLTLFormula()->getBatchAP(apSet[a]).getAP();
        Point target = environment->TSStateIdToGridCenter(region);

        // The task is only complete once its slowest member arrives, so the whole team is
        // charged that arrival and none of them is free any earlier
        uint16_t arrival = 0;
        for (uint8_t robotId : teams[a]) {
            if (robotId == 0 || robotId > times.size() || robotId > positions.size()) continue;
            Robot* robot = multiRobotSystem->getRobot(robotId);
            if (!robot) continue;
            uint16_t reached = times[robotId - 1] + robot->getTravelTime(positions[robotId - 1], target);
            if (reached > arrival) arrival = reached;
        }
        for (uint8_t robotId : teams[a]) {
            if (robotId == 0 || robotId > times.size() || robotId > positions.size()) continue;
            times[robotId - 1] = arrival;
            positions[robotId - 1] = target;
        }
    }
}

/**
 * @brief Recursively cover the capabilities still outstanding across a conjunction.
 *
 * Takes a proposition with an uncovered requirement and branches over every unused robot that
 * carries it. Any valid assignment must cover that requirement with one of those robots, so the
 * search cannot miss a solution that exists.
 * @param required   Required capabilities per proposition.
 * @param caps       Capabilities per robot, indexed from zero.
 * @param order      Order in which robots are tried; randomised by the caller.
 * @param used       Robots already committed; updated as the search descends and backtracks.
 * @param covered    Requirements already met; updated as the search descends and backtracks.
 * @param robotsByAP Teams built so far, carrying 1-based robot ids.
 * @return True once every proposition is fully covered.
 */
static bool coverRemainingCapabilities(const std::vector<std::vector<bool>>& required,
                                       const std::vector<std::vector<bool>>& caps,
                                       const std::vector<uint8_t>& order,
                                       std::vector<bool>& used,
                                       std::vector<std::vector<bool>>& covered,
                                       std::vector<std::vector<uint8_t>>& robotsByAP) {
    for (size_t i = 0; i < required.size(); ++i) {
        for (size_t j = 0; j < required[i].size(); ++j) {
            if (!required[i][j] || covered[i][j]) continue;

            // Proposition i still needs capability j, so some unused robot carrying j must take it
            for (uint8_t r : order) {
                if (used[r] || j >= caps[r].size() || !caps[r][j]) continue;

                // Giving r to proposition i covers every capability of i that r happens to carry
                std::vector<size_t> newlyCovered;
                for (size_t k = 0; k < required[i].size(); ++k) {
                    if (required[i][k] && !covered[i][k] && k < caps[r].size() && caps[r][k]) {
                        covered[i][k] = true;
                        newlyCovered.push_back(k);
                    }
                }
                used[r] = true;
                robotsByAP[i].push_back(r + 1);  // teams carry 1-based robot ids

                if (coverRemainingCapabilities(required, caps, order, used, covered, robotsByAP)) {
                    return true;
                }

                robotsByAP[i].pop_back();
                used[r] = false;
                for (size_t k : newlyCovered) covered[i][k] = false;
            }
            return false;  // nothing left can supply this capability
        }
    }
    return true;  // every proposition fully covered
}

/**
 * @brief Assign a disjoint team to every proposition of a conjunction.
 * @param apSet      Propositions that must all be discharged together.
 * @param robotsByAP Receives one team per proposition, by 1-based robot id.
 * @return True when an assignment exists, false when none does.
 * @note A robot serves one proposition only, so conjunctions asking for more robots carrying
 *       some capability than the team holds are rejected before any search. The robot order is
 *       randomised, so repeated calls give different assignments, but the search is exhaustive:
 *       false means genuinely infeasible rather than unlucky. An empty conjunction, which comes
 *       from a purely negative transition label, is satisfied by the empty assignment.
 */
bool RandomSamplingTaskAllocation::assignTeamsToConjunction(
        const std::vector<uint16_t>& apSet,
        std::vector<std::vector<uint8_t>>& robotsByAP) const {
    robotsByAP.assign(apSet.size(), std::vector<uint8_t>());
    // A conjunction with no APs asks for nothing, so the empty assignment serves it
    if (apSet.empty()) return true;

    //get the total required capabilities
    std::vector<std::vector<bool>> required(apSet.size());
    for (size_t i = 0; i < apSet.size(); ++i) {
        required[i] = nba->getLTLFormula()->getRequiredCapabilities(apSet[i]);
    }

    //get a 2d vector of robot capabilities
    uint8_t numRobots = multiRobotSystem->getNumRobots();
    std::vector<std::vector<bool>> caps(numRobots);
    for (uint8_t r = 0; r < numRobots; ++r) {
        caps[r] = multiRobotSystem->getRobotCapabilities(r + 1);  // robot ids are 1-based
    }

    // Cheap rejects before searching: a robot serves one proposition, so for every capability
    // there must be at least as many robots carrying it as propositions asking for it
    std::vector<uint16_t> supply, demand;
    for (const auto& roboCaps : caps) {
        if (roboCaps.size() > supply.size()) supply.resize(roboCaps.size(), 0);
        for (size_t j = 0; j < roboCaps.size(); ++j) {
            if (roboCaps[j]) supply[j]++;
        }
    }
    for (const auto& req : required) {
        if (req.size() > demand.size()) demand.resize(req.size(), 0);
        for (size_t j = 0; j < req.size(); ++j) {
            if (req[j]) demand[j]++;
        }
    }
    for (size_t j = 0; j < demand.size(); ++j) {
        if (demand[j] > (j < supply.size() ? supply[j] : 0)) return false;
    }

    // Randomise which robot is tried first so repeated calls give different assignments
    std::vector<uint8_t> order(numRobots);
    for (uint8_t r = 0; r < numRobots; ++r) order[r] = r;
    for (size_t i = order.size(); i > 1; --i) {
        std::swap(order[i - 1], order[std::rand() % i]);
    }

    std::vector<bool> used(numRobots, false);
    std::vector<std::vector<bool>> covered(apSet.size());
    for (size_t i = 0; i < apSet.size(); ++i) {
        covered[i].assign(required[i].size(), false);
    }

    if (coverRemainingCapabilities(required, caps, order, used, covered, robotsByAP)) {
        return true;
    }
    robotsByAP.assign(apSet.size(), std::vector<uint8_t>());
    return false;
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

/**
 * @brief Record the prefix and suffix planned for an accepting state.
 */
void RandomSamplingTaskAllocation::addPath(uint16_t acceptingState, TaskAllocPath prePath, TaskAllocPath sufPath) {
    paths[acceptingState] = {prePath, sufPath};
}

/**
 * @brief Record a plan for a finite specification.
 * @note A finite run stops at its accepting state, so the suffix is stored empty.
 */
void RandomSamplingTaskAllocation::addNoSufPath(uint16_t acceptingState, TaskAllocPath prePath) {
    // Create an empty suffix path (no nodes, zero length, zero makespan)
    TaskAllocPath emptySufPath{0, std::vector<Random_Node*>(), 0};
    paths[acceptingState] = {prePath, emptySufPath};
}
