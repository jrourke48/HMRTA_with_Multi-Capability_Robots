#include "Tree/PlanningDecisionTree.h"
#include "Environment/Environment.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/TarjansAlgorithm.cpp"
#include "../../Transition_Systems/GeneralTransitionSystem.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <string>
#include <spot/tl/formula.hh>

class RandomSamplingTaskAllocation {
    private:
        BuchiAutomaton* nba;
        Environment* environment;
        MultiRobotSystem* multiRobotSystem;
        std::vector<std::vector<uint16_t>> AcceptingSCCs; // Strongly connected components of the accepting states in the NBA
        std::vector<std::vector<Random_Node*>> paths; //optimal allocation paths for each SCC in the NBA
        std::vector<uint16_t> optimalMakespans; //optimal makespans for each sccs
        std::vector<uint16_t> optimalSCC; //optimal paths 
        uint16_t optimalMakespan; //optimal makespan for the optimal SCC
        uint16_t numSCCs; // Number of strongly connected components in the NBA
        uint16_t maxIterations; // Maximum number of iterations for random sampling
        uint16_t Iterations; // Number of iterations for random sampling
        double timeLimit; // Time limit for random sampling
        double computationTime; // Time taken for the computation

        //set accepting sccs
        void setAcceptingSCCs();
        //get random feasible task allocation - returns (robotsByAP, satisfiedTrueAPs)
        //robotsByAP[i] = vector of robot indices assigned to satisfy apSet[i]
        std::pair<std::vector<std::vector<uint8_t>>, std::vector<uint16_t>> getRandomFeasibleTaskAllocation(Node* curNode, Node* newNode);
    public:
        // Constructor with iteration parameter
        RandomSamplingTaskAllocation(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr, uint16_t maxIterations);
        //Constructor with timelimit parameter
        RandomSamplingTaskAllocation(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr, double timeLimit);
        
        // Destructor
        ~RandomSamplingTaskAllocation();

        //run the random sampling task allocation algorithm
        void run();

        //========================
        // SETTERS AND GETTERS
        //========================
        // System component setters
        void setNBA(BuchiAutomaton* nbaPtr);
        void setEnvironment(Environment* envPtr);
        void setMultiRobotSystem(MultiRobotSystem* robotSysPtr);
        // System component getters
        BuchiAutomaton* getNBA() const;
        Environment* getEnvironment() const;
        MultiRobotSystem* getMultiRobotSystem() const;
        // Accepting SCCs getters
        std::vector<std::vector<uint16_t>> getAcceptingSCCs() const;
        // Paths getters and setters
        const std::vector<std::vector<std::vector<uint16_t>>>& getPaths() const;
        void setPaths(const std::vector<std::vector<std::vector<uint16_t>>>& paths);
        // Optimal makespans getters and setters
        const std::vector<uint16_t>& getOptimalMakespans() const;
        void setOptimalMakespans(const std::vector<uint16_t>& makespans);
        // Number of SCCs getters and setters
        uint16_t getNumSCCs() const;
        void setNumSCCs(uint16_t num);
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
};
