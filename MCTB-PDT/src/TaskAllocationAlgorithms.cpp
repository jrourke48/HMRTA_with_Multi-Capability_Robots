#include "TaskAllocationAlgorithms.h"
#include <algorithm>
#include <queue>
#include <set>
#include <limits>
#include <cmath>
#include <map>
#include <fstream>
#include <iostream>
#include <stdexcept>

/**
 * TaskAllocationAlgorithms - Constructor
 * Initializes the algorithm with references to NBA, Environment, and MultiRobotSystem
 */
TaskAllocationAlgorithms::TaskAllocationAlgorithms(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr)
    : nba(nbaPtr), environment(envPtr), planningTree(nullptr), traversedTree(nullptr), multiRobotSystem(robotSysPtr) {
    metrics = new AlgorithmMetrics();
}

/**
 * TaskAllocationAlgorithms - Destructor
 */
TaskAllocationAlgorithms::~TaskAllocationAlgorithms() {
    // Clean up metrics
    if (metrics != nullptr) {
        delete metrics;
        metrics = nullptr;
    }
    // Note: Other pointers are managed externally
}

//  * Algorithm 1: Intensive Inter-Task Relationship Tree Search
//  * Converts LTL formula to Büchi automaton and initializes tree planning
//  */


/**
 * setNBA - Set the NBA reference
 */
void TaskAllocationAlgorithms::setNBA(BuchiAutomaton* nbaPtr) {
    nba = nbaPtr;
}

/**
 * setEnvironment - Set the Environment reference
 */
void TaskAllocationAlgorithms::setEnvironment(Environment* envPtr) {
    environment = envPtr;
}

/**
 * setMultiRobotSystem - Set the MultiRobotSystem reference
 */
void TaskAllocationAlgorithms::setMultiRobotSystem(MultiRobotSystem* robotSysPtr) {
    multiRobotSystem = robotSysPtr;
}

/**
 * getNBA - Get the NBA reference
 */
BuchiAutomaton* TaskAllocationAlgorithms::getNBA() const {
    return nba;
}

/**
 * getEnvironment - Get the Environment reference
 */
Environment* TaskAllocationAlgorithms::getEnvironment() const {
    return environment;
}

/**
 * getMultiRobotSystem - Get the MultiRobotSystem reference
 */
MultiRobotSystem* TaskAllocationAlgorithms::getMultiRobotSystem() const {
    return multiRobotSystem;
}

/**
 * setPlanningTree - Set the planning tree reference
 */
void TaskAllocationAlgorithms::setPlanningTree(PlanningDecisionTree* tree) {
    planningTree = tree;
}

/**
 * getPlanningTree - Get the planning tree reference
 */
PlanningDecisionTree* TaskAllocationAlgorithms::getPlanningTree() const {
    return planningTree;
}

/**
 * setTraversedTree - Set the traversed tree reference
 */
void TaskAllocationAlgorithms::setTraversedTree(PlanningDecisionTree* tree) {
    traversedTree = tree;
}

/**
 * getTraversedTree - Get the traversed tree reference
 */
PlanningDecisionTree* TaskAllocationAlgorithms::getTraversedTree() const {
    return traversedTree;
}

PlanningDecisionTree* TaskAllocationAlgorithms::intensiveInterTaskRelationshipTreeSearch(
    BuchiAutomaton* nbaPtr,
    Environment* envPtr,
    MultiRobotSystem* multiRobotSystemPtr) {
    
    // Initialize metrics: clear to ensure clean state
    metrics->clearMetrics();
    
    // ===== SET INDEPENDENT VARIABLES =======================================
    //========================================================================
    // Initialize independent variables for algorithm metrics
    AlgorithmMetrics::IndependentVariables indVars;
    
    // Automaton characteristics
    indVars.num_automaton_states = nbaPtr->getNumStates();
    indVars.num_automaton_edges = nbaPtr->getNumEdges();
    indVars.total_required_capabilities = nbaPtr->getLTLFormula()->getTotalRequiredCapabilities();
    indVars.independent_required_capabilities = nbaPtr->getLTLFormula()->getIndependentRequiredCapabilities();
    if (!nbaPtr->getLTLFormula()) {
        std::cerr << "[ERROR] LTL formula is null" << std::endl;
        return nullptr;
    }
    indVars.num_atomic_propositions = nbaPtr->getLTLFormula()->getBatchAtomicPropositions().size();
    
    // Robot fleet characteristics
    indVars.num_robots = multiRobotSystemPtr->getNumRobots();
    indVars.num_ts_regions = envPtr->getNumStates();
    // Total capabilities across all robots
    indVars.total_robot_capabilities = multiRobotSystemPtr->getTotalRobotCapabilities();
    indVars.independent_robot_capabilities = multiRobotSystemPtr->getIndependentCapabilities();
    
   
    //Set inter-task constraints from LTL formula
    indVars.num_inter_task_constraints = 0;
    
    // Store in metrics (must be BEFORE any runtime metric updates)
    metrics->setIndependentVariables(indVars);
    
    // Start timing the algorithm execution
    metrics->startTimer();
    //=======================================================================
    //=======================================================================


    // Get initial automaton state from the Büchi automaton
    Node* initialAutomatonState = nbaPtr->getNode(nbaPtr->getInitialState());
    if (!initialAutomatonState) {
        std::cerr << "[ERROR] Failed to get initial automaton state" << std::endl;
        return nullptr;
    }
    // Get environment initial state ID
    uint16_t initialEnvStateId = envPtr->getInitialState();

    // Allocate on heap to avoid dangling pointer
    Node* initialEnvNodePtr = new Node(initialEnvStateId);
    
    // Create initial task allocation and times vectors
    std::vector<bool> initialAllocation;
    std::vector<uint16_t> initialTimes;
    std::vector<Point> initialPositions = multiRobotSystemPtr ? multiRobotSystemPtr->getRobotPositions() : std::vector<Point>();
    if (multiRobotSystemPtr) {
        for (size_t i = 0; i < multiRobotSystemPtr->getRobots().size(); ++i) {
            initialAllocation.push_back(false);
            initialTimes.push_back(0);
        }
    }
    
    // Initialize planning tree with initial state
    PlanningDecisionTree* tree = new PlanningDecisionTree(                             // rootId
        initialAutomatonState,          // automaton state
        initialEnvNodePtr,              // ts state node (heap-allocated)
        initialAllocation,              // task allocation
        initialTimes,                   // times
        initialPositions,               // robot positions
        0,                              // batch
        Tree_Node::TASK_PROGRESS::PRE   // progress
    );
    setPlanningTree(tree);
    setTraversedTree(new PlanningDecisionTree());
    
    
    // Initialize untraversed queue with root node
    Tree_Node* rootNode = tree->getRoot();
    if (!rootNode) {
        std::cerr << "[ERROR] rootNode is nullptr!" << std::endl;
        return nullptr;
    }
    
    addUntraversedPlanningNode(rootNode);

    // Process nodes from untraversed queue until empty
    Tree_Node* currentNode = nullptr;
    
    //Algorithm 1:: main tree construction/search
    while ((currentNode = getNextUntraversedNode()) != nullptr) {
        
        // Create subtree for current node
        PlanningDecisionTree* subtree = new PlanningDecisionTree(currentNode);
        if (!subtree) {
            std::cerr << "[ERROR] Failed to create subtree" << std::endl;
            break;
        }
        
        uint16_t buchisize = nbaPtr->getNumStates();
        std::vector<uint16_t> acceptingStates = nbaPtr->getAcceptingStates();

        // Iterate through all automaton states
        for (uint16_t i = 0; i < buchisize; ++i) {
            Node* nbaState = nbaPtr->getNode(i);
            if (!nbaState) {
                continue;  // Skip if node not found or if already visited with the same progress
            }
            uint16_t nbaId = nbaState->getId();
            
            uint16_t currentStateId = currentNode->getAutomatonState()->getId();
            
            // Get edge labels from current automaton state to this state
            //toggle this if you want to check for multiple true aps on the same edge
            std::vector<std::vector<uint16_t>> apIds = nbaPtr->getTrueAPs(currentStateId, nbaId);
        
            for (const auto& apIdVec : apIds) {
                    for (const auto& apId : apIdVec) {
                    // Validate LTL formula access
                    if (!nbaPtr->getLTLFormula()) {
                        std::cerr << "[ERROR] LTL formula is null during AP processing" << std::endl;
                        continue;
                    }
                    int8_t batchVal = nbaPtr->getLTLFormula()->getBatchVal(apId);
                
                    // Validate transition system access
                    if (!envPtr->getTransitionSystem()) {
                        std::cerr << "[ERROR] Transition system is null" << std::endl;
                        continue;
                    }
                    Node* TSState = envPtr->getTransitionSystem()->getNode(nbaPtr->getLTLFormula()->getTSState(apId));
                if (!TSState) {
                    continue;
                }
                
                // Create new tree node with automaton state and task state
                // Note: nodeId is a placeholder; insertNode will auto-assign the correct ID
                Tree_Node* newNode = new Tree_Node(0, currentNode, nbaState, 
                                                   TSState, batchVal);
                //std::cout << "Current Node: NBA: " <<  nbaState->getId() << ", TS: " << currentNode->getTSState()->getId() << std::endl;
            
                
                if (!newNode) {
                    continue;
                }
                // Route based on batch value using the correct search algorithm
                switch (GetCorrectSearchAlgorithm(batchVal)) {
                    case SearchAlgorithmSelector::US:
                        // Algorithm 2: Unrelated Task Search
                        unrelatedTaskSearch(newNode, TSState, currentNode, apId);
                        break;
                    case SearchAlgorithmSelector::CS:
                        // Algorithm 3: Compatible Task Search
                        compatibleTaskSearch(newNode, TSState, currentNode, apId);
                        break;
                    case SearchAlgorithmSelector::ES:
                        // Algorithm 4: Exclusive Task Search
                        exclusiveTaskSearch(newNode, TSState, currentNode, apId);
                        break;
                    default:
                        std::cerr << "[ERROR] Invalid batch value for search algorithm selection" << std::endl;
                        break;
                }
                // Add to subtree (insertNode will auto-assign the correct nodeId)
                subtree->insertNode(newNode);
            }
            }
        }
    //set the current node as traversed
    traversedTree->insertNode(currentNode);
    
    // Prune subtree
    std::vector<Tree_Node*> pruningResult = pruneSubtree(subtree);
    // Add non-pruned nodes to untraversed queue for further processing
    for (Tree_Node* node : pruningResult) {
            addUntraversedPlanningNode(node);
    }

    // Add non-pruned nodes to the main planning tree as children of currentNode
    planningTree->insertNodes(currentNode, pruningResult);
    }
    // After processing all nodes, get the final optimal frontier node  
    Tree_Node* finalOptimalNode = planningTree->getOptimalFrontierNode(nba->isFinite()); // Final optimal node after search completion
    //get the total number of nodes traversed and in expanded in the planning tree
    metrics->subtree_efficiency_.total_nodes_traversed = traversedTree->getNumNodes();
    metrics->subtree_efficiency_.total_nodes_planning = planningTree->getNumNodes();
    // Final tree state
    // Reassign node IDs to ensure proper hierarchy order (root = 0, then by depth)
    tree->reassignNodeIds();
    
    // Stop timing the algorithm execution
    metrics->stopTimer();
    //Compute derived metrics and store the solution quality metrics after stopping the timer
    metrics->computeDerivedMetrics();
    
    // Update independent variables with results from algorithm execution
    metrics->iv_.num_inter_task_constraints = static_cast<int>(treebatchvals.size());
    
    // Store solution quality metrics
    metrics->setSolutionMakespan(finalOptimalNode->getMaxTime());
    metrics->setSumOfTravelTimes(finalOptimalNode->getSumOfTimes());
    metrics->setRobotsUtilized(finalOptimalNode->getNumUtilizedRobots());
    
    return tree;
}

/**
 * Algorithm 2: Unrelated-Task Search (US)
 * Searches for unrelated tasks that can be executed independently
 * Mutates newNode with search results
 * 
 * NOTE: Uses apId (unique AP identifier) instead of tsStateId for capability lookup
 * This is critical because multiple APs can map to the same TS state but require different capabilities
 */
void TaskAllocationAlgorithms::unrelatedTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode,
    uint16_t apId) {

    if (!newNode || !TSState || !multiRobotSystem || !nba || !environment) {
        return;
    }
    // current task state information
    uint16_t tsStateId = TSState->getId();
    // Get task location from environment mapping p_an
    Point taskLocation = environment->TSStateIdToGridCenter(tsStateId);
    
    multiRobotSystem->setRobotPositions(currentNode->getRobotPositions()); // Ensure current node has robot positions stored
    
    //times to goal for each robot
    std::vector<uint16_t> timesToGoal = multiRobotSystem->updateRobotTimesToGoal(currentNode->getTimes(), taskLocation);

    // Get the sort of the times vector and get the corresponding robot indices to find the best robot assignment
    std::vector<std::pair<uint16_t, uint16_t>> sortedTimes = Tree_Node::getSortedTimes(timesToGoal);
    
    // Get required capabilities from the BatchAtomicProposition using apId (not tsStateId)
    // CRITICAL: Different APs can map to same TS state but have different required capabilities
    BatchAtomicProposition batchAP = nba->getLTLFormula()->getBatchAP(apId);
    std::vector<bool> requiredCapabilities = batchAP.getCapabilities();
    
    // Find all permutations of robots that satisfy all required capabilities
    const std::vector<Robot*>& allRobots = multiRobotSystem->getRobots();
    
    auto [taskAllocation, maxTime] = getTaskAllocation(allRobots, requiredCapabilities, sortedTimes);
    
    // Check if we found a valid allocation
    bool hasAllocation = false;
    for (bool allocated : taskAllocation) {
        if (allocated) {
            hasAllocation = true;
            break;
        }
    }
    
    if (!hasAllocation) {
        std::cout << "No valid task allocation found for AP ID " << apId << std::endl;
        return;
    }
    
    // Start with parent node's times and update all allocated robots to the same max time
    std::vector<uint16_t> updatedTimes = currentNode->getTimes();
    updateAllocatedRobotsToMaxTime(taskAllocation, timesToGoal, updatedTimes);
    
    // Update newNode with the task allocation and updated times
    newNode->setRoboTaskAllocation(taskAllocation);
    newNode->setTimes(updatedTimes);
    
    // Update robot positions to the task location for all allocated robots
    std::vector<Point> updatedPositions = currentNode->getRobotPositions();
    placeAllocatedRobotsAtAdjacentCells(taskAllocation, taskLocation, updatedPositions);
    
    if (!newNode) {
        return;
    }
    
    newNode->setRobotPositions(updatedPositions);
    
    // Update node progress based on automaton state
    updateNodeProgress(newNode, currentNode);
 }

/**
 * Algorithm 3: Compatible-Task Search (CS)
 * Searches for compatible tasks considering both sub-tasks and main tasks
 * Mutates newNode with search results
 * 
 */
void TaskAllocationAlgorithms::compatibleTaskSearch(
    Tree_Node* newNode,
    [[maybe_unused]] Node* TSState,
    [[maybe_unused]] Tree_Node* currentNode, 
    [[maybe_unused]] uint16_t apId) {
    
    if (!newNode || !TSState || !multiRobotSystem || !nba || !environment) {
        return;
    }
    
    // current task state information
    uint16_t tsStateId = TSState->getId();
    
    // Get task location from environment mapping p_an
    Point taskLocation = environment->TSStateIdToGridCenter(tsStateId);
    
    std::vector<Point> currentPositions = currentNode->getRobotPositions();
    
    multiRobotSystem->setRobotPositions(currentPositions);
    
    std::vector<uint16_t> currentTimes = currentNode->getTimes();

    // Update robot times to goal for the new task location
    std::vector<uint16_t> timesToGoal = multiRobotSystem->updateRobotTimesToGoal(currentTimes, taskLocation);

    // Get the sort of the times vector and get the corresponding robot indices to find the best robot assignment
    std::vector<std::pair<uint16_t, uint16_t>> sortedTimes = Tree_Node::getSortedTimes(timesToGoal);
    
    //Get the batch value for the newnode
    uint16_t batchValue = newNode->getBatch();
    
    //get the allocation of the other node with the same batch value
    if (!planningTree) {
        return;
    }
    std::vector<bool> taskAllocation = planningTree->getAllocationfromBatchValue(batchValue);

    // Start with parent node's times and only update allocated robots with their individual travel times
        std::vector<uint16_t> updatedTimes = currentNode->getTimes();
    
    if (taskAllocation.empty()) {
        BatchAtomicProposition batchAP = nba->getLTLFormula()->getBatchAP(apId);
        std::vector<bool> requiredCapabilities = batchAP.getCapabilities();
        
        // Find all permutations of robots that satisfy all required capabilities
        const std::vector<Robot*>& allRobots = multiRobotSystem->getRobots();
        auto [taskAllocation, maxTime] = getTaskAllocation(allRobots, requiredCapabilities, sortedTimes);
        // Check if we found a valid allocation
        bool hasAllocation = false;
        for (bool allocated : taskAllocation) {
            if (allocated) {
                hasAllocation = true;
                break;
            }
        }
        
        if (!hasAllocation) {
            std::cout << "No valid task allocation found for AP ID " << apId << std::endl;
            return;
        }
        
        for (size_t i = 0; i < taskAllocation.size() && i < updatedTimes.size(); ++i) {
            if (taskAllocation[i]) {
                // Use individual calculated travel time for each allocated robot
                updatedTimes[i] = maxTime;
            }
        }
    } else {
        // Update all allocated robots to the same max time
        updateAllocatedRobotsToMaxTime(taskAllocation, timesToGoal, updatedTimes);
    }
    // Update newNode with the task allocation and updated times
    newNode->setRoboTaskAllocation(taskAllocation);
    newNode->setTimes(updatedTimes);
    
    // Update robot positions to the task location for all allocated robots
    std::vector<Point> updatedPositions = currentNode->getRobotPositions();
    
    // CRITICAL: If positions vector is empty or too small, initialize with default positions
    if (updatedPositions.size() != taskAllocation.size()) {
        updatedPositions.resize(taskAllocation.size(), Point(0, 0));
    }
    
    placeAllocatedRobotsAtAdjacentCells(taskAllocation, taskLocation, updatedPositions);
    
    newNode->setRobotPositions(updatedPositions);
    
    // Update node progress based on automaton state
    updateNodeProgress(newNode, currentNode);
}

/**
 * Algorithm 4: Exclusive-task Search (ES)
 * Searches for exclusive tasks that have conflicting batches
 * Mutates newNode with search results
 * 
*/
void TaskAllocationAlgorithms::exclusiveTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode,
    uint16_t apId) {
    
    // Check for null pointers
    if (!newNode || !TSState || !multiRobotSystem || !nba || !environment) {
        return;
    }
    
    // current task state information
    uint16_t tsStateId = TSState->getId();
    
    // Get task location from environment mapping p_an
    Point taskLocation = environment->TSStateIdToGridCenter(tsStateId);
    multiRobotSystem->setRobotPositions(currentNode->getRobotPositions()); // Ensure current node has robot positions stored
    
    std::vector<uint16_t> timesToGoal = multiRobotSystem->updateRobotTimesToGoal(currentNode->getTimes(), taskLocation);

    // Get the sort of the times vector and get the corresponding robot indices to find the best robot assignment
    std::vector<std::pair<uint16_t, uint16_t>> sortedTimes = Tree_Node::getSortedTimes(timesToGoal);
    
    // Get required capabilities from the BatchAtomicProposition using apId (not tsStateId)
    BatchAtomicProposition batchAP = nba->getLTLFormula()->getBatchAP(apId);
    std::vector<bool> requiredCapabilities = batchAP.getCapabilities();
   
    // Create copy of available robots (exclude robots allocated to opposite batch)
    std::vector<Robot*> availableRobots = multiRobotSystem->getRobots();
    
    // Get the opposite batch value for the newnode
    int8_t currentBatch = newNode->getBatch();
    uint16_t oppositeBatch = static_cast<uint16_t>(std::abs(currentBatch));
    std::vector<bool> exclusiveAllocation = planningTree->getAllocationfromBatchValue(oppositeBatch);
    
    // Remove allocated robots (loop backward to avoid index shifts)
    for (int i = static_cast<int>(exclusiveAllocation.size()) - 1; i >= 0; --i) {
        if (i < static_cast<int>(availableRobots.size()) && exclusiveAllocation[i]) {
            availableRobots.erase(availableRobots.begin() + i);
        }
    }
    

    
    auto [taskAllocation, maxTime] = getTaskAllocation(availableRobots, requiredCapabilities, sortedTimes);
    
    // Check if we found a valid allocation
    bool hasAllocation = false;
    for (bool allocated : taskAllocation) {
        if (allocated) {
            hasAllocation = true;
            break;
        }
    }
    
    if (!hasAllocation) {
        std::cout << "NO allocation found for AP ID " << apId << std::endl;
        return;
    }
    // Start with parent node's times and update all allocated robots to the same max time
    std::vector<uint16_t> updatedTimes = currentNode->getTimes();
    updateAllocatedRobotsToMaxTime(taskAllocation, timesToGoal, updatedTimes);
    
    // Update newNode with the task allocation and updated times
    newNode->setRoboTaskAllocation(taskAllocation);
    newNode->setTimes(updatedTimes);
    
    // Update robot positions to the task location for all allocated robots
    std::vector<Point> updatedPositions = currentNode->getRobotPositions();
    
    // CRITICAL: If positions vector is empty or too small, initialize with default positions
    if (updatedPositions.size() != taskAllocation.size()) {
        updatedPositions.resize(taskAllocation.size(), Point(0, 0));
    }
    
    placeAllocatedRobotsAtAdjacentCells(taskAllocation, taskLocation, updatedPositions);
    newNode->setRobotPositions(updatedPositions);
    
    // Update node progress based on automaton state
    updateNodeProgress(newNode, currentNode);
}




/**
 * updateAllocatedRobotsToMaxTime - Helper function
 * Finds max time among allocated robots and sets all allocated robots to that time
 * Uses taskAllocation vector as a bitmask to filter which robots to consider
 */
void TaskAllocationAlgorithms::updateAllocatedRobotsToMaxTime(
    const std::vector<bool>& taskAllocation,
    const std::vector<uint16_t>& timesToGoal,
    std::vector<uint16_t>& updatedTimes) {
    
    // Find max time among allocated robots (bitmask operation)
    uint16_t maxTime = 0;
    for (size_t i = 0; i < taskAllocation.size() && i < timesToGoal.size(); ++i) {
        if (taskAllocation[i]) {
            maxTime = std::max(maxTime, timesToGoal[i]);
        }
    }
    
    // Set all allocated robots to the same max time
    for (size_t i = 0; i < taskAllocation.size() && i < updatedTimes.size(); ++i) {
        if (taskAllocation[i]) {
            updatedTimes[i] = maxTime;
        }
    }
}

/**
 * placeAllocatedRobotsAtAdjacentCells - Helper function
 * Places allocated robots at adjacent cells around task location (N, E, S, W, NE, NW, SE, SW)
 * Uses taskAllocation as a bitmask to determine which robots to place
 * Allocates robots in spiral order around the task
 */
void TaskAllocationAlgorithms::placeAllocatedRobotsAtAdjacentCells(
    const std::vector<bool>& taskAllocation,
    const Point& taskLocation,
    std::vector<Point>& updatedPositions) {
    
    // Pattern: N, E, S, W, NE, NW, SE, SW
    std::vector<std::pair<int, int>> adjacentOffsets = {
        {0, -1},   // North
        {1, 0},    // East
        {0, 1},    // South
        {-1, 0},   // West
        {1, -1},   // Northeast
        {-1, -1},  // Northwest
        {1, 1},    // Southeast
        {-1, 1}    // Southwest
    };
    
    int robotIndex = 0;
    for (size_t i = 0; i < taskAllocation.size(); ++i) {
        if (taskAllocation[i]) {
            if (i >= updatedPositions.size()) {
                continue;  // Skip if out of bounds
            }
            
            int offsetIdx = robotIndex % adjacentOffsets.size();
            int newX = taskLocation.getX() + adjacentOffsets[offsetIdx].first;
            int newY = taskLocation.getY() + adjacentOffsets[offsetIdx].second;
            
            // Clamp to valid grid coordinates [0, 20]
            newX = std::max(0, std::min(newX, 20));
            newY = std::max(0, std::min(newY, 20));
            
            updatedPositions[i] = Point(newX, newY);
            robotIndex++;
        }
    }
}

/**
 * updateNodeProgress - Helper function
 * Updates node progress based on whether it enters an accepting automaton state
 * Increments progress when entering accepting state, or handles SUF->OTH transition
 */
void TaskAllocationAlgorithms::updateNodeProgress(
    Tree_Node* newNode,
    Tree_Node* currentNode) {
    
    if (!newNode || !nba) {
        return;
    }
    
    bool acceptingState = nba->isAccepting(newNode->getAutomatonState()->getId());
    
    if (acceptingState) {
        // Increment progress when entering accepting state
        int newProgressInt = static_cast<int>(currentNode->getProgress()) + 1;
        Tree_Node::TASK_PROGRESS newProgress = static_cast<Tree_Node::TASK_PROGRESS>(newProgressInt);
        newNode->setProgress(newProgress);
    } else if (currentNode->getProgress() == Tree_Node::TASK_PROGRESS::SUF) {
        // If the current node is at SUF progress, increment to OTH
        newNode->setProgress(Tree_Node::TASK_PROGRESS::OTH);
    }
    else {
        // Not entering accepting: keep progress the same
        newNode->setProgress(currentNode->getProgress());
    }
}

/**
 * addVisitedAutomatonState - Add state to visited collection for given progress level
 * Per Rule 2: Track which automaton states have been visited at each progress level
 */
void TaskAllocationAlgorithms::addVisitedAutomatonState(uint16_t state, uint8_t progress) {
    std::vector<uint16_t>* targetVector = nullptr;
    if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::PRE)) {
        targetVector = &visitedAutomatonStates_PRE;
    } else if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::TRA)) {
        targetVector = &visitedAutomatonStates_TRA;
    } else if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::SUF)) {
        targetVector = &visitedAutomatonStates_SUF;
    }
    
    if (targetVector && std::find(targetVector->begin(), targetVector->end(), state) == targetVector->end()) {
        targetVector->push_back(state);
    }
}

/**
 * isAutomatonStateVisited - Check if state was visited at given progress level
 * Returns true only if this state was visited at this specific progress level
 */
bool TaskAllocationAlgorithms::isAutomatonStateVisited(uint16_t state, uint8_t progress) const {
    const std::vector<uint16_t>* targetVector = nullptr;
    if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::PRE)) {
        targetVector = &visitedAutomatonStates_PRE;
    } else if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::TRA)) {
        targetVector = &visitedAutomatonStates_TRA;
    } else if (progress == static_cast<uint8_t>(Tree_Node::TASK_PROGRESS::SUF)) {
        targetVector = &visitedAutomatonStates_SUF;
    }
    
    if (targetVector) {
        return std::find(targetVector->begin(), targetVector->end(), state) != targetVector->end();
    }
    return false;
}

/**
 * clearVisitedAutomatonStates - Clear all visited states for all progress levels
 */
void TaskAllocationAlgorithms::clearVisitedAutomatonStates() {
    visitedAutomatonStates_PRE.clear();
    visitedAutomatonStates_TRA.clear();
    visitedAutomatonStates_SUF.clear();
}

/**
 * addBatchValue - Add a batch value to the collection
 */
void TaskAllocationAlgorithms::addBatchValue(int8_t batchValue) {
    if (std::find(treebatchvals.begin(), treebatchvals.end(), batchValue) == treebatchvals.end()) {
        treebatchvals.push_back(batchValue);
    }
}

/**
 * GetCorrectSearchAlgorithm - Determine the appropriate search algorithm based on the batch value
 * Returns US if batch value is 0 or not found in the tree
 * Returns CS if batch value is found in the tree
 * Returns ES if the negation of the batch value is found in the tree
 */
TaskAllocationAlgorithms::SearchAlgorithmSelector TaskAllocationAlgorithms::GetCorrectSearchAlgorithm(int8_t batchValue) {
    bool found = std::find(treebatchvals.begin(), treebatchvals.end(), batchValue) != treebatchvals.end();
    bool foundNeg = std::find(treebatchvals.begin(), treebatchvals.end(), -batchValue) != treebatchvals.end();
    if (batchValue == 0 || (!found && !foundNeg)) {
        addBatchValue(batchValue);  // Add the new batch value to the collection
        return SearchAlgorithmSelector::US;
    } else if (found) {
        return SearchAlgorithmSelector::CS;
    } else if (foundNeg) {
        return SearchAlgorithmSelector::ES;
    } else {
        return SearchAlgorithmSelector::US;
    }
}
    

/**
 * getBatchValues - Get the collection of batch values in the tree
 */
std::vector<int8_t>& TaskAllocationAlgorithms::getBatchValues() {
    return treebatchvals;
}

/**
 * clearBatchValues - Clear all batch values
 */
void TaskAllocationAlgorithms::clearBatchValues() {
    treebatchvals.clear();
}

/**
 * addUntraversedPlanningNode - Add a node to the untraversed queue
 */
void TaskAllocationAlgorithms::addUntraversedPlanningNode(Tree_Node* node) {
    if (node) {
        untraversedPlanningQueue.push(node);
    }
}

/**
 * getNextUntraversedNode - Get the next untraversed node from the queue
 */
Tree_Node* TaskAllocationAlgorithms::getNextUntraversedNode() {
    if (!untraversedPlanningQueue.empty()) {
        Tree_Node* node = untraversedPlanningQueue.front();
        untraversedPlanningQueue.pop();
        return node;
    }
    return nullptr;
}

/**
 * clearUntraversedQueue - Clear all untraversed nodes from the queue
 */
void TaskAllocationAlgorithms::clearUntraversedQueue() {
    while (!untraversedPlanningQueue.empty()) {
        untraversedPlanningQueue.pop();
    }
}

/**
 * getTaskAllocation
 * Greedily selects minimum set of robots from sorted times that satisfy all required capabilities
 * Iterates through robots in ascending time order and accumulates only required capabilities
 * Returns (task allocation vector, max time of selected robots)
 */
std::pair<std::vector<bool>, uint16_t> TaskAllocationAlgorithms::getTaskAllocation(
    const std::vector<Robot*>& robots,
    const std::vector<bool>& requiredCapabilities, 
    const std::vector<std::pair<uint16_t, uint16_t>>& sortedTimes) {
    
    std::vector<bool> taskAllocation(robots.size(), false);
    std::vector<bool> accumulatedCapabilities(requiredCapabilities.size(), false);
    uint16_t maxTime = 0;
    
    if (robots.empty() || requiredCapabilities.empty()) {
        return {taskAllocation, maxTime};
    }
    
    // Iterate through robots in ascending time order (best times first)
    // Exit as soon as all required capabilities are satisfied
    auto it = sortedTimes.begin();
    while (it != sortedTimes.end() && accumulatedCapabilities != requiredCapabilities) {
        const auto& [robotIdx, time] = *it;
        
        if (robotIdx < robots.size()) {
            Robot* robot = robots[robotIdx];
            if (robot) {
                std::vector<bool> robotCapabilities = robot->getCapabilities();
                bool robotContributesCapability = false;
                
                // Check if this robot contributes any NEW required capabilities (not already accumulated)
                for (size_t j = 0; j < requiredCapabilities.size() && j < robotCapabilities.size(); ++j) {
                    if (requiredCapabilities[j] && robotCapabilities[j] && !accumulatedCapabilities[j]) {
                        accumulatedCapabilities[j] = true;
                        robotContributesCapability = true;
                    }
                }
                
                // Only mark robot as selected if it contributes at least one required capability
                if (robotContributesCapability) {
                    taskAllocation[robotIdx] = true;
                    maxTime = std::max(maxTime, time);  // Update max time of selected robots
                }
            }
        }
        
        ++it;
    }
    
    if (accumulatedCapabilities != requiredCapabilities) {
        std::string missing;
        for (size_t i = 0; i < requiredCapabilities.size(); ++i) {
            if (requiredCapabilities[i] && !accumulatedCapabilities[i]) {
                if (!missing.empty()) missing += ", ";
                missing += std::to_string(i);
            }
        }
        throw std::runtime_error("Failed to satisfy all required capabilities. Missing: " + missing);
    }

    return {taskAllocation, maxTime};
}
//helper method to prune subtree
// Implements three boundary rules:
// Rule 1: Remove nodes with OTH progress (terminal nodes)
// Rule 2: Remove nodes with (automaton state, progress) pairs already in traversedTree
// Rule 3: For nodes with same automaton state and progress, keep only minimum cost
std::vector<Tree_Node*> TaskAllocationAlgorithms::pruneSubtree(PlanningDecisionTree* subtree) {
    std::vector<Tree_Node*> nodesToRemove;
    std::vector<Tree_Node*> subtreeNodes = subtree->getAllNodes();
    
    // Rule 1: If the NBA is Ininite Mark OTH progress nodes (terminal nodes) for removal
    //If the the NBA is finite mark the TRA progress nodes for removal
   //Also add the node to the planning tree rather than just the traversed tree
    for (Tree_Node* node : subtreeNodes) {
    if (nba && nba->isInfinite()) {
        if (node->getProgress() == Tree_Node::TASK_PROGRESS::OTH) {
            nodesToRemove.push_back(node);  // Mark OTH nodes for removal so they don't re-enter queue
            traversedTree->insertNode(node);  // Add to traversed tree
            planningTree->insertNode(node);  // Add to planning tree
            // Increment the algorithm metric for total nodes satisfying the ltl
            metrics->subtree_efficiency_.nodes_satisfying_ltl++;
        }  
    } else{ 
        if (node->getProgress() == Tree_Node::TASK_PROGRESS::TRA) {
            nodesToRemove.push_back(node);  // Mark TRA nodes for removal so they don't re-enter queue
            traversedTree->insertNode(node);  // Add to traversed tree
            planningTree->insertNode(node);  // Add to planning tree
            // Increment the algorithm metric for total nodes satisfying the ltl
            metrics->subtree_efficiency_.nodes_satisfying_ltl++;
        }
    }
    }

    
    // Rule 2: Remove nodes with traversed (automaton state, progress) pairs
    // If we've already visited (state S, progress P), don't sample it again
    for (Tree_Node* node : subtreeNodes) {
        // Rule 2: Track (nbaState, progress) pair as visited
                // Always add - never clear. Each progress level maintains its own visited set
                uint8_t nodeProgress = static_cast<uint8_t>(node->getProgress());
                uint32_t nbaId = node->getAutomatonState()->getId();
                if (!isAutomatonStateVisited(nbaId, nodeProgress)) {
                    addVisitedAutomatonState(nbaId, nodeProgress);
                } else {
                    nodesToRemove.push_back(node);  // Mark for removal so it doesn't re-enter queue
                    continue;
                }
    }
    
    
    // Rule 3: For nodes with same automaton state and progress, keep only minimum cost
    // (Only apply for finite automata; infinite automata need to explore multiple paths for cycles)
    
    // Group remaining nodes by (automaton state ID, progress)
    std::map<std::pair<uint32_t, int>, std::vector<Tree_Node*>> nodeGroups;
    for (Tree_Node* node : subtreeNodes) {
        if (std::find(nodesToRemove.begin(), nodesToRemove.end(), node) != nodesToRemove.end()) {
            continue;  // Skip nodes already marked for removal
        }
        
        if (node->getAutomatonState()) {
            auto key = std::make_pair(node->getAutomatonState()->getId(), 
                                     static_cast<int>(node->getProgress()));
            nodeGroups[key].push_back(node);
        }
    }
    
    // Within each group with duplicates, keep only the minimum cost node
    for (auto& [key, nodes] : nodeGroups) {
        if (nodes.size() > 1) {
            
            // Calculate cost for each node (sum of all robot times)
            Tree_Node* minCostNode = nodes[0];
            uint32_t minCost = 0;
            for (uint16_t time : minCostNode->getTimes()) {
                minCost += time;
            }
            
            // Find minimum cost node and mark all others for removal
            for (size_t i = 1; i < nodes.size(); ++i) {
                uint32_t nodeCost = 0;
                for (uint16_t time : nodes[i]->getTimes()) {
                    nodeCost += time;
                }
                
                if (nodeCost < minCost) {
                    // Current node is better, mark previous min for removal
                    nodesToRemove.push_back(minCostNode);
                    minCostNode = nodes[i];
                    minCost = nodeCost;
                } else {
                    // Current node is worse or equal, mark for removal
                    nodesToRemove.push_back(nodes[i]);
                }
            }
        }
    }
    
    // Remove all marked nodes from subtree and add to traversedTree
    // Transfer nodes from subtree to traversedTree (don't delete, just move)
    for (Tree_Node* node : nodesToRemove) {
        // Add to traversedTree first (while node still exists in memory)
        traversedTree->insertNode(node);
        subtree->removeFrontierNode(node);  // Remove from subtree
        // Increment the algorithm metric for total nodes pruned
        metrics->subtree_efficiency_.total_nodes_pruned++;
    }
    
    // Return remaining frontier nodes after pruning
    return subtree->getFrontierNodes();
}

/**
 * visualizeTree - Creates a Graphviz visualization of the entire planning tree
 * Shows all nodes colored by progress level and batch value
 * Displays robot allocations for each node
 */
void TaskAllocationAlgorithms::visualizeTree(const std::string& filename) const {
    if (!planningTree || !planningTree->getRoot()) {
        std::cerr << "Error: Planning tree is empty or not initialized" << std::endl;
        return;
    }
    
    // Helper lambda to escape special characters for DOT format
    auto escapeDotLabel = [](const std::string& str) -> std::string {
        std::string result;
        for (size_t i = 0; i < str.length(); ++i) {
            char c = str[i];
            // Only escape double quotes - leave \\n and other sequences alone
            if (c == '"') {
                result += "\\\"";
            } else if (c == '\\' && i + 1 < str.length() && str[i + 1] == 'n') {
                // Keep \\n as-is for DOT newlines
                result += "\\n";
                ++i;  // Skip the 'n'
            } else if (c != '\r') {  // Skip carriage returns
                result += c;
            }
        }
        return result;
    };
    
    std::string dotFile = filename + ".dot";
    std::string pngFile = filename + ".png";
    
    std::ofstream file(dotFile);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file: " << dotFile << std::endl;
        return;
    }
    
    // Write DOT header
    file << "digraph PlanningTree {\n";
    file << "  rankdir=TB;\n";
    file << "  node [shape=box, style=filled, fontname=Arial, fontsize=10];\n";
    file << "  edge [fontname=Arial, fontsize=9];\n\n";
    
    // Color mapping for progress levels
    auto getProgressColor = [](Tree_Node::TASK_PROGRESS prog) -> std::string {
        switch(prog) {
            case Tree_Node::TASK_PROGRESS::PRE: return "#FFE6E6";  // Light red
            case Tree_Node::TASK_PROGRESS::TRA: return "#FFFFCC";  // Light yellow
            case Tree_Node::TASK_PROGRESS::SUF: return "#E6F3FF";  // Light blue
            case Tree_Node::TASK_PROGRESS::OTH: return "#E6FFE6";  // Light green
            default: return "#FFFFFF";                              // White
        }
    };
    
    auto getProgressLabel = [](Tree_Node::TASK_PROGRESS prog) -> std::string {
        switch(prog) {
            case Tree_Node::TASK_PROGRESS::PRE: return "PRE";
            case Tree_Node::TASK_PROGRESS::TRA: return "TRA";
            case Tree_Node::TASK_PROGRESS::SUF: return "SUF";
            case Tree_Node::TASK_PROGRESS::OTH: return "OTH";
            default: return "?";
        }
    };
    
    // Get all nodes from the tree
    std::vector<Tree_Node*> allNodes = planningTree->getAllNodes();
    
    // Write nodes
    for (Tree_Node* node : allNodes) {
        if (!node || !node->getAutomatonState() || !node->getTSState()) continue;
        
        std::string nodeId = "node_" + std::to_string(node->getId());
        uint16_t nbaId = node->getAutomatonState()->getId();
        uint16_t tsId = node->getTSState()->getId();
        std::string progLabel = getProgressLabel(node->getProgress());
        uint16_t maxTime = node->getMaxTime();
        
        // Create label with node information
        std::string label = "Node " + std::to_string(node->getId()) + "\\n";
        label += "NBA: " + std::to_string(nbaId) + ", TS: " + std::to_string(tsId) + "\\n";
        label += "Progress: " + progLabel + "\\n";
        label += "Max Time: " + std::to_string(maxTime) + "\\n";
        label += "Batch: " + std::to_string((int)node->getBatch()) + "\\n";
        
        // Add robot allocation info
        label += "Robots: [";
        const auto& allocation = node->getRoboTaskAllocation();
        for (size_t i = 0; i < allocation.size(); ++i) {
            label += (allocation[i] ? "1" : "0");
            if (i < allocation.size() - 1) label += ",";
        }
        label += "]";
        
        // Escape label for DOT format
        label = escapeDotLabel(label);
        
        // Determine node color based on progress
        std::string color = getProgressColor(node->getProgress());
        
        // Check if this is the optimal frontier node
        Tree_Node* optimalNode = planningTree->getOptimalFrontierNode(nba->isFinite());
        if (node == optimalNode) {
            file << "  " << nodeId << " [label=\"" << label << "\", fillcolor=\"#FF6B6B\", "
                 << "penwidth=3, color=red];\n";
        } else {
            file << "  " << nodeId << " [label=\"" << label << "\", fillcolor=\"" << color 
                 << "\"];\n";
        }
    }
    
    // Write edges
    file << "\n";
    for (Tree_Node* node : allNodes) {
        if (!node) continue;
        
        // Find all children by checking which nodes have this node as parent
        for (Tree_Node* otherNode : allNodes) {
            if (otherNode && otherNode->getParent() == node) {
                std::string fromId = "node_" + std::to_string(node->getId());
                std::string toId = "node_" + std::to_string(otherNode->getId());
                uint16_t edgeCost = otherNode->getMaxTime();
                
                file << "  " << fromId << " -> " << toId 
                     << " [label=\"cost=" << edgeCost << "\"];\n";
            }
        }
    }
    
    file << "}\n";
    file.close();
    
    // Convert DOT to PNG using Graphviz
    std::string command = "dot -Tpng \"" + dotFile + "\" -o \"" + pngFile + "\"";
    system(command.c_str());
}

/**
 * visualizeOptimalPath - Creates a comprehensive visualization of the optimal path
 * Integrates gridworld, tasks, constraints, robots, capabilities, LTL formula, and optimal path
 */
void TaskAllocationAlgorithms::visualizeOptimalPath(const std::string& filename) const {
    if (!planningTree || !planningTree->getRoot() || !environment || !multiRobotSystem || !nba) {
        std::cerr << "Error: Required components not initialized" << std::endl;
        return;
    }
    
    // Helper lambda to escape special characters for DOT format
    auto escapeDotLabel = [](const std::string& str) -> std::string {
        std::string result;
        for (size_t i = 0; i < str.length(); ++i) {
            char c = str[i];
            // Only escape double quotes - leave \\n and other sequences alone
            if (c == '"') {
                result += "\\\"";
            } else if (c == '\\' && i + 1 < str.length() && str[i + 1] == 'n') {
                // Keep \\n as-is for DOT newlines
                result += "\\n";
                ++i;  // Skip the 'n'
            } else if (c != '\r') {  // Skip carriage returns
                result += c;
            }
        }
        return result;
    };
    
    // Helper lambda to get capability name
    auto getRobotCapabilityName = [](RobotCapability cap) -> std::string {
        switch (cap) {
            case RobotCapability::MOVEMENT_GROUND:      return "Movement_Ground";
            case RobotCapability::MOVEMENT_AERIAL:      return "Movement_Aerial";
            case RobotCapability::MOVEMENT_AQUATIC:     return "Movement_Aquatic";
            case RobotCapability::SENSOR_CAMERA:        return "Sensor_Camera";
            case RobotCapability::SENSOR_LIDAR:         return "Sensor_LIDAR";
            case RobotCapability::SENSOR_GPS:           return "Sensor_GPS";
            case RobotCapability::SENSOR_IMU:           return "Sensor_IMU";
            case RobotCapability::SENSOR_PROXIMITY:     return "Sensor_Proximity";
            case RobotCapability::MANIPULATION_GRIPPER: return "Manipulation_Gripper";
            case RobotCapability::MANIPULATION_TOOL:    return "Manipulation_Tool";
            case RobotCapability::COMMUNICATION_WIFI:   return "Communication_WiFi";
            case RobotCapability::COMMUNICATION_4G:     return "Communication_4G";
            case RobotCapability::CAPABILITY_PAYLOAD:   return "Capability_Payload";
            default:                                     return "Unknown";
        }
    };
    
    Tree_Node* optimalNode = planningTree->getOptimalFrontierNode(nba->isFinite());
    if (!optimalNode) {
        std::cerr << "Error: No optimal frontier node found" << std::endl;
        return;
    }
    
    std::string dotFile = filename + ".dot";
    std::string pngFile = filename + ".png";
    
    std::ofstream file(dotFile);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file: " << dotFile << std::endl;
        return;
    }
    
    // Trace path from optimal node back to root
    std::vector<Tree_Node*> optimalPath;
    Tree_Node* current = optimalNode;
    size_t maxDepth = 1000;
    size_t depth = 0;
    
    while (current != nullptr && depth < maxDepth) {
        optimalPath.push_back(current);
        Tree_Node* parent = current->getParent();
        if (parent == current) {
            std::cerr << "Warning: Circular parent reference detected" << std::endl;
            break;
        }
        current = parent;
        depth++;
    }
    std::reverse(optimalPath.begin(), optimalPath.end());
    
    // Write DOT header
    file << "digraph ComprehensiveOptimalPath {\n";
    file << "  rankdir=TB;\n";
    file << "  concentrate=true;\n";
    file << "  node [fontname=Arial];\n";
    file << "  edge [fontname=Arial, fontsize=9];\n\n";
    
    // ===== ENVIRONMENT & GRIDWORLD CLUSTER =====
    file << "  subgraph cluster_environment {\n";
    file << "    label=\"Environment & Gridworld\";\n";
    file << "    style=filled;\n";
    file << "    fillcolor=\"#F0F0F0\";\n";
    file << "    color=black;\n\n";
    
    if (environment->getGridWorld()) {
        GridWorld* grid = environment->getGridWorld();
        uint16_t width = grid->getWidth();
        uint16_t height = grid->getHeight();
        
        std::string gridInfo = "GridWorld: " + std::to_string(width) + "x" + std::to_string(height) + "\\n";
        gridInfo += "Total States: " + std::to_string(environment->getNumStates()) + "\\n";
        gridInfo += "Initial State: " + std::to_string(environment->getInitialState()) + "\\n";
        
        file << "    env_info [shape=box, label=\"" << escapeDotLabel(gridInfo) << "\", fillcolor=\"#E8F4F8\"];\n\n";
        
        // Show task locations
        file << "    env_tasks [shape=plaintext, label=<\n";
        file << "      <TABLE BORDER=\"1\" CELLBORDER=\"1\" CELLSPACING=\"0\">\n";
        file << "        <TR><TD COLSPAN=\"3\"><B>Task Locations</B></TD></TR>\n";
        file << "        <TR><TD><B>State ID</B></TD><TD><B>Position</B></TD><TD><B>Type</B></TD></TR>\n";
        
        TS* ts = environment->getTransitionSystem();
        if (ts) {
            for (uint16_t i = 0; i < environment->getNumStates() && i < 10; ++i) {
                Point taskPos = environment->TSStateIdToGridCenter(i);
                file << "        <TR><TD>" << i << "</TD><TD>(" << taskPos.getX() << "," << taskPos.getY() 
                     << ")</TD><TD>Task</TD></TR>\n";
            }
        }
        file << "      </TABLE>\n";
        file << "    >];\n";
    }
    
    file << "  }\n\n";
    
    // ===== ROBOT TEAM CLUSTER =====
    file << "  subgraph cluster_robots {\n";
    file << "    label=\"Robot Team & Capabilities\";\n";
    file << "    style=filled;\n";
    file << "    fillcolor=\"#F0F8F0\";\n";
    file << "    color=black;\n\n";
    
    uint16_t teamSize = (multiRobotSystem) ? multiRobotSystem->getNumRobots() : 0;
    std::string teamInfo = "Team Size: " + std::to_string(teamSize) + " robots\\n";
    file << "    team_info [shape=box, label=\"" << escapeDotLabel(teamInfo) << "\", fillcolor=\"#E8F8E8\"];\n\n";
    
    // Robot capabilities table
    file << "    robot_caps [shape=plaintext, label=<\n";
    file << "      <TABLE BORDER=\"1\" CELLBORDER=\"1\" CELLSPACING=\"0\">\n";
    file << "        <TR><TD COLSPAN=\"3\"><B>Robot Capabilities</B></TD></TR>\n";
    file << "        <TR><TD><B>Robot ID</B></TD><TD><B>Name</B></TD><TD><B>Capabilities</B></TD></TR>\n";
    
    const auto& robots = multiRobotSystem->getRobots();
    for (const auto* robot : robots) {
        if (!robot) continue;
        
        std::string capsList;
        const auto& robotCaps = robot->getCapabilities();
        bool hasAny = false;
        for (size_t c = 0; c < robotCaps.size(); ++c) {
            if (robotCaps[c]) {
                if (hasAny) capsList += ", ";
                capsList += getRobotCapabilityName(static_cast<RobotCapability>(c));
                hasAny = true;
            }
        }
        if (capsList.empty()) capsList = "None";
        
        file << "        <TR><TD>R" << robot->getRobotId() << "</TD>";
        file << "<TD>" << robot->getName() << "</TD>";
        file << "<TD>" << capsList << "</TD></TR>\n";
    }
    
    file << "      </TABLE>\n";
    file << "    >];\n";
    file << "  }\n\n";
    
    // ===== LTL FORMULA CLUSTER =====
    file << "  subgraph cluster_ltl {\n";
    file << "    label=\"LTL Formula & Task Constraints\";\n";
    file << "    style=filled;\n";
    file << "    fillcolor=\"#F8F0F0\";\n";
    file << "    color=black;\n\n";
    
    if (nba && nba->getLTLFormula()) {
        auto formula = nba->getLTLFormula();
        std::string formulaInfo = "LTL Formula Details:\\n";
        formulaInfo += "Automaton States: " + std::to_string(nba->getNumStates()) + "\\n";
        formulaInfo += "Accepting States: ";
        int acceptingCount = 0;
        for (uint16_t i = 0; i < nba->getNumStates(); ++i) {
            if (nba->isAccepting(i)) acceptingCount++;
        }
        formulaInfo += std::to_string(acceptingCount) + "\\n";
        formulaInfo += "\\n--- LTL FORMULA ---\\n";
        formulaInfo += formula->toString();
        
        file << "    ltl_info [shape=box, label=\"" << escapeDotLabel(formulaInfo) << "\", fillcolor=\"#F8E8E8\"];\n\n";
        
        file << "    ltl_tasks [shape=plaintext, label=<\n";
        file << "      <TABLE BORDER=\"1\" CELLBORDER=\"1\" CELLSPACING=\"0\">\n";
        file << "        <TR><TD COLSPAN=\"5\"><B>Task Requirements (LTL APs)</B></TD></TR>\n";
        file << "        <TR><TD><B>AP ID</B></TD><TD><B>TS Region</B></TD><TD><B>Batch Value</B></TD><TD><B>Type</B></TD><TD><B>Required Capabilities</B></TD></TR>\n";
        
        try {
            const auto& batchAPs = formula->getBatchAtomicPropositions();
            uint16_t count = 0;
            for (const auto& batchAP : batchAPs) {
                if (count >= 12) break;  // Limit to 12 rows
                uint16_t apId = batchAP.getAPId();
                uint16_t tsRegionId = batchAP.getAP();
                int8_t batchVal = batchAP.getBatch();
                std::string batchType = (batchVal > 0) ? "Compatible" : (batchVal < 0) ? "Exclusive" : "Unrelated";
                
                // Get required capabilities
                std::string capsList;
                const auto& caps = batchAP.getCapabilities();
                bool hasAny = false;
                for (size_t c = 0; c < caps.size(); ++c) {
                    if (caps[c]) {
                        if (hasAny) capsList += ", ";
                        capsList += getRobotCapabilityName(static_cast<RobotCapability>(c));
                        hasAny = true;
                    }
                }
                if (capsList.empty()) capsList = "None";
                
                file << "        <TR><TD>AP" << apId << "</TD>";
                file << "<TD>TS" << tsRegionId << "</TD>";
                file << "<TD>" << static_cast<int>(batchVal) << "</TD>";
                file << "<TD>" << batchType << "</TD>";
                file << "<TD>" << capsList << "</TD></TR>\n";
                count++;
            }
        } catch (const std::exception& e) {
            file << "        <TR><TD COLSPAN=\"5\">Error loading task information</TD></TR>\n";
        }
        
        file << "      </TABLE>\n";
        file << "    >];\n";
    }
    
    file << "  }\n\n";
    
    // ===== OPTIMAL PATH CLUSTER =====
    file << "  subgraph cluster_path {\n";
    file << "    label=\"Optimal Execution Path\";\n";
    file << "    style=filled;\n";
    file << "    fillcolor=\"#FFF8F0\";\n";
    file << "    color=darkred;\n";
    file << "    penwidth=2;\n\n";
    
    // Path summary
    std::string pathSummary = "Optimal Path Summary\\n";
    pathSummary += "Total Steps: " + std::to_string(optimalPath.size()) + "\\n";
    pathSummary += "Final Cost: " + std::to_string(optimalNode->getMaxTime()) + "\\n";
    uint32_t totalCostSum = 0;
    for (uint16_t t : optimalNode->getTimes()) totalCostSum += t;
    pathSummary += "Total Execution Time: " + std::to_string(totalCostSum) + "s\\n";
    
    file << "    path_summary [shape=note, label=\"" << escapeDotLabel(pathSummary) << "\", fillcolor=\"#FFFFCC\", penwidth=2];\n\n";
    
    // Path nodes with detailed information
    for (size_t i = 0; i < optimalPath.size(); ++i) {
        Tree_Node* node = optimalPath[i];
        if (!node) continue;
        
        std::string nodeId = "opt_" + std::to_string(i);
        uint16_t nbaId = node->getAutomatonState() ? node->getAutomatonState()->getId() : 0;
        uint16_t tsId = node->getTSState() ? node->getTSState()->getId() : 0;
        uint16_t maxTime = node->getMaxTime();
        uint16_t totalCost = 0;
        for (uint16_t t : node->getTimes()) totalCost += t;
        
        // Create detailed label
        std::string label = "Step " + std::to_string(i) + "\\n";
        label += "NBA: " + std::to_string(nbaId) + " | TS: " + std::to_string(tsId) + "\\n";
        label += "Cost: " + std::to_string(maxTime) + "s | Total: " + std::to_string(totalCost) + "s\\n";
        label += "-----\\n";
        label += "Allocated Robots:\\n";
        
        const auto& allocation = node->getRoboTaskAllocation();
        const auto& times = node->getTimes();
        const auto& robots = multiRobotSystem->getRobots();
        
        for (size_t r = 0; r < allocation.size(); ++r) {
            if (allocation[r]) {
                uint32_t robotId = (r < robots.size() && robots[r]) ? robots[r]->getRobotId() : r + 1;
                label += "  R" + std::to_string(robotId) + ": " + std::to_string(times[r]) + "s\\n";
            }
        }
        
        // Escape and write node
        label = escapeDotLabel(label);
        
        // Highlight final optimal node
        if (i == optimalPath.size() - 1) {
            file << "    " << nodeId << " [shape=box, label=\"" << label << "\", fillcolor=\"#FF6B6B\", "
                 << "penwidth=3, color=darkred, fontweight=bold, fontsize=11];\n";
        } else {
            file << "    " << nodeId << " [shape=box, label=\"" << label << "\", fillcolor=\"#B3E5FC\", penwidth=1.5];\n";
        }
    }
    
    // Path edges
    file << "\n";
    for (size_t i = 0; i < optimalPath.size() - 1; ++i) {
        std::string fromId = "opt_" + std::to_string(i);
        std::string toId = "opt_" + std::to_string(i + 1);
        
        Tree_Node* fromNode = optimalPath[i];
        Tree_Node* toNode = optimalPath[i + 1];
        
        uint16_t costDelta = toNode->getMaxTime() - fromNode->getMaxTime();
        
        file << "    " << fromId << " -> " << toId 
             << " [label=\"Δ=" << costDelta << "s\", penwidth=2, color=darkred];\n";
    }
    
    file << "  }\n\n";
    
    // ===== INTERCONNECTIONS =====
    file << "  // Connections between clusters\n";
    file << "  env_info -> path_summary [style=dashed, color=gray, label=\"tasks\"];\n";
    file << "  team_info -> path_summary [style=dashed, color=gray, label=\"robots\"];\n";
    file << "  ltl_info -> path_summary [style=dashed, color=gray, label=\"constraints\"];\n\n";
    
    file << "}\n";
    file.close();
    
    // Convert DOT to PNG using Graphviz
    std::string command = "dot -Tpng \"" + dotFile + "\" -o \"" + pngFile + "\"";
    system(command.c_str());
}