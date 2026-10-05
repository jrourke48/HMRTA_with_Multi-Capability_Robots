#include "Tree/PlanningDecisionTree.h"
#include "Environment/Environment.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Transition_Systems/GeneralTransitionSystem.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include "RandomNode.h"
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <memory>
#include <string>
#include <spot/tl/formula.hh>

class RandomSamplingTaskAllocation {
    public:
        struct TaskAllocPath{
            uint16_t pathLength;
            std::vector<Random_Node*> path;
            uint16_t makespan;
        };

    private:
        BuchiAutomaton* nba;
        Environment* environment;
        MultiRobotSystem* multiRobotSystem;
        std::map<uint16_t, std::pair<TaskAllocPath, TaskAllocPath>> paths; //accepting state -> pair of task allocation paths (prefix,suffix)
        std::vector<Random_Node*> optimalPath; //optimal path
        uint16_t optimalMakespan; //optimal makespan for the optimal path
        uint16_t numNBAStates; // Number of states in the NBA
        BuchiAutomaton* prunedNBA; // Copy of NBA with infeasible edges removed
        uint16_t maxIterations; // Maximum number of iterations for random sampling
        uint16_t Iterations; // Number of iterations for random sampling
        double timeLimit; // Time limit for random sampling
        double computationTime; // Time taken for the computation
        std::vector<Point> initialPositions; // Robot placements when planning began, snapshotted
                                             // once so sampling never reads the team's live state

    public:
        // Constructor with iteration parameter
        RandomSamplingTaskAllocation(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr, uint16_t maxIterations);
        //Constructor with timelimit parameter
        RandomSamplingTaskAllocation(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr, double timeLimit);
        
        // Destructor
        ~RandomSamplingTaskAllocation();

        //trim infeasible edges from the buchi to make sure that all edges are feasible during search This needs work
        void pruneInfeasibleNBAPaths();

        //run the random sampling task allocation algorithm
        void run();
        //helper for run for finite and infinite automata BOTH need work 
        void runFinitePathPlanner();
        void runInfinitePathPlanner();

        //Uses DFS to get minimum length path to Goal This needs work
        TaskAllocPath getMinLengthPath(Node* srcNode, Node* goalNode);

        //Incrementally build a random path (or cycle if srcNode == goalNode) of at most targetLength nodes
        //returns an empty path if the goal is not reached or the makespan exceeds makespanBound
        //startTimes and startPositions carry the robot clocks and placements the path continues from,
        //so a suffix picks up where its prefix left off; empty means start fresh
        TaskAllocPath buildRandomPath(BuchiAutomaton* searchNBA, Node* srcNode, Node* goalNode, uint16_t targetLength,
                                      bool hasBound, uint16_t makespanBound,
                                      const std::vector<uint16_t>& startTimes = {},
                                      const std::vector<Point>& startPositions = {});

        //Advance one step: every robot serving an AP travels to that AP's region, and the robots on
        //the same AP are synchronised to the last arrival, since the task is not done until all arrive
        void applyStep(const std::vector<uint16_t>& apSet,
                       const std::vector<std::vector<uint8_t>>& teams,
                       std::vector<uint16_t>& times,
                       std::vector<Point>& positions) const;


        //========================
        // SETTERS AND GETTERS
        //========================
        // System component setters
        void setNBA(BuchiAutomaton* nbaPtr);
        void setEnvironment(Environment* envPtr);
        void setMultiRobotSystem(MultiRobotSystem* robotSysPtr);
        // System component getters
        BuchiAutomaton* getNBA() const;
        BuchiAutomaton* getPrunedNBA() const;
        Environment* getEnvironment() const;
        MultiRobotSystem* getMultiRobotSystem() const;
        // Paths getters and adders
        // get the (prefix, suffix) paths for a specific accepting state
        const std::pair<TaskAllocPath, TaskAllocPath>& getPath(uint16_t acceptingState) const;
        // get the entire map of paths for all accepting states
        const std::map<uint16_t, std::pair<TaskAllocPath, TaskAllocPath>>& getAllPaths() const;
        // Add a new path for a specific accepting state 
        void addPath(uint16_t acceptingState, TaskAllocPath prePath, TaskAllocPath sufPath);
        //add a new path for a finite NBA meaning no suffix is needed
        void addNoSufPath(uint16_t acceptingState, TaskAllocPath prePath);

        // Optimal path getters and setters
        const std::vector<Random_Node*>& getOptimalPath() const;
        void setOptimalPath(const std::vector<Random_Node*>& path);
        // Optimal makespan getters and setters
        uint16_t getOptimalMakespan() const;
        void setOptimalMakespan(uint16_t makespan);
        // Number of NBA states getters and setters
        uint16_t getNumNBAStates() const;
        void setNumNBAStates(uint16_t num);
        // Max iterations getters and setters
        uint16_t getMaxIterations() const;
        void setMaxIterations(uint16_t maxIter);
        // Iterations getters and setters
        uint16_t getIterations() const;
        void setIterations(uint16_t iters);
        // Time limit getters and setters
        double getTimeLimit() const;
        void setTimeLimit(double limit);
        // Computation time getters and setters
        double getComputationTime() const;
        void setComputationTime(double time);

        //get random feasible task allocation - returns (robotsByAP, satisfiedTrueAPs)
        //robotsByAP[i] = vector of robot indices assigned to satisfy apSet[i]
        std::pair<std::vector<std::vector<uint8_t>>, std::vector<uint16_t>> getRandomFeasibleTaskAllocation(Node* curNode, Node* newNode);

        //Assign a disjoint team of robots to every AP of a conjunction, so that each team covers
        //its AP's required capabilities. Robots are considered in random order, so repeated calls
        //give different assignments, but the search is exhaustive: it returns false only when no
        //assignment exists. Both pruning and allocation ask this same question.
        bool assignTeamsToConjunction(const std::vector<uint16_t>& apSet,
                                      std::vector<std::vector<uint8_t>>& robotsByAP) const;
};
