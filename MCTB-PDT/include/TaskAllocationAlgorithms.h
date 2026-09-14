#ifndef TASK_ALLOCATION_ALGORITHMS_H
#define TASK_ALLOCATION_ALGORITHMS_H

#include "AlgorithmMetrics.h"
#include "Tree/PlanningDecisionTree.h"
#include "Environment/Environment.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Transition_Systems/GeneralTransitionSystem.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <string>
#include <spot/tl/formula.hh>

class TaskAllocationAlgorithms {
    private:
        BuchiAutomaton* nba;
        Environment* environment;
        PlanningDecisionTree* planningTree;
        PlanningDecisionTree* traversedTree; // To keep track of the tree being traversed during search
        MultiRobotSystem* multiRobotSystem;
        std::vector<uint16_t> visitedAutomatonStates_PRE;   // Visited states at PRE progress
        std::vector<uint16_t> visitedAutomatonStates_TRA;   // Visited states at TRA progress
        std::vector<uint16_t> visitedAutomatonStates_SUF;   // Visited states at SUF progress
        std::vector<int8_t> treebatchvals; // To keep track of batch values in the tree (if needed for cost calculations)
        std::queue<Tree_Node*> untraversedPlanningQueue; // Queue of nodes in planning tree not yet traversed
        AlgorithmMetrics* metrics;
        
    public:
        // Constructor
        TaskAllocationAlgorithms(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr);
        
        // Enum for selecting the search algorithm based on inter-task constraints
        // US: Unrelated-Task Search, CS: Compatible-Task Search, ES: Exclusive-Task Search
        enum class SearchAlgorithmSelector {
            US = 0,
            CS = 1,
            ES = 2
        };
        
        // Destructor
        ~TaskAllocationAlgorithms();
        
        // System component setters
        void setNBA(BuchiAutomaton* nbaPtr);
        void setEnvironment(Environment* envPtr);
        void setMultiRobotSystem(MultiRobotSystem* robotSysPtr);
        
        // System component getters
        BuchiAutomaton* getNBA() const;
        Environment* getEnvironment() const;
        MultiRobotSystem* getMultiRobotSystem() const;

        // Planning tree methods
        void setPlanningTree(PlanningDecisionTree* tree);
        PlanningDecisionTree* getPlanningTree() const;
        
        // Traversed tree methods
        void setTraversedTree(PlanningDecisionTree* tree);
        PlanningDecisionTree* getTraversedTree() const;

        // Visited state methods - separate tracking per progress level
        void addVisitedAutomatonState(uint16_t state, uint8_t progress);
        bool isAutomatonStateVisited(uint16_t state, uint8_t progress) const;
        void clearVisitedAutomatonStates();

        // Batch values in tree methods
        void addBatchValue(int8_t batchValue);
        SearchAlgorithmSelector GetCorrectSearchAlgorithm(int8_t batchValue);
        std::vector<int8_t>& getBatchValues();
        void clearBatchValues();

        // Untraversed planning queue methods
        void addUntraversedPlanningNode(Tree_Node* node);
        Tree_Node* getNextUntraversedNode();
        void clearUntraversedQueue();
        
        //algorithm metrics methods
        AlgorithmMetrics& getMetrics() { return *metrics; }
        const AlgorithmMetrics& getMetrics() const { return *metrics; }
        void resetMetrics() { *metrics = AlgorithmMetrics(); }
        
        /**
         * Algorithm 1: Intensive Inter-Task Relationship Tree Search
         * Builds a planning tree considering inter-task relationships
         */
        PlanningDecisionTree* intensiveInterTaskRelationshipTreeSearch(
            BuchiAutomaton* nba,
            Environment* env,
            MultiRobotSystem* multiRobotSystem);
        
        /**
         * Algorithm 2: Unrelated-Task Search (US)
         * Searches for unrelated tasks that can be executed
         * Mutates newNode with search results
         */
        void unrelatedTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode,
            uint16_t apId);
        
        /**
         * Algorithm 3: Compatible-Task Search (CS)
         * Searches for compatible tasks considering constraints
         * Mutates newNode with search results
         */
        void compatibleTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode,
            uint16_t apId);
        
        /**
         * Algorithm 4: Exclusive-task Search (ES)
         * Searches for exclusive tasks that have conflicting batches
         * Mutates newNode with search results
         */
        void exclusiveTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode,
            uint16_t apId);
        std::vector<Tree_Node*> pruneSubtree(PlanningDecisionTree* subtree);
        
        /**
         * getTaskAllocation
         * Greedily selects minimum set of robots that satisfy all required capabilities
         * Returns (task allocation vector, max time of selected robots)
         */
        std::pair<std::vector<bool>, uint16_t> getTaskAllocation(
            const std::vector<Robot*>& robots,
            const std::vector<bool>& requiredCapabilities,
            const std::vector<std::pair<uint16_t, uint16_t>>& sortedTimes);

        /**
         * visualizeTree
         * Visualizes the planning decision tree using Graphviz 
         */
        void visualizeTree(const std::string& filename) const;
        /** 
         * visualize Optimal Path
         */
        void visualizeOptimalPath(const std::string& filename) const;

private:
        /**
         * updateAllocatedRobotsToMaxTime - Helper function
         * Finds max time among allocated robots and sets all allocated robots to that time
         * Uses taskAllocation vector as a bitmask to filter which robots to consider
         */
        void updateAllocatedRobotsToMaxTime(
            const std::vector<bool>& taskAllocation,
            const std::vector<uint16_t>& timesToGoal,
            std::vector<uint16_t>& updatedTimes);

        /**
         * placeAllocatedRobotsAtAdjacentCells - Helper function
         * Places allocated robots at adjacent cells around task location (N, E, S, W, NE, NW, SE, SW)
         * Uses taskAllocation as a bitmask to determine which robots to place
         */
        void placeAllocatedRobotsAtAdjacentCells(
            const std::vector<bool>& taskAllocation,
            const Point& taskLocation,
            std::vector<Point>& updatedPositions);

        /**
         * updateNodeProgress - Helper function
         * Updates node progress based on whether it enters an accepting automaton state
         * Increments progress when entering accepting state, or handles SUF->OTH transition
         */
        void updateNodeProgress(
            Tree_Node* newNode,
            Tree_Node* currentNode);
};
#endif // TASK_ALLOCATION_ALGORITHMS_H
