
#ifndef TREE_NODE_H
#define TREE_NODE_H

#include "../../Automatons/Edge_Node.h"
#include "Environment/Point.h"
#include <vector>

class Tree_Node {
    public:
        enum class TASK_PROGRESS {
            PRE,
            TRA,
            SUF,
            OTH
        };
    
    private:
        uint32_t id; // Unique identifier for the node (encodes NBA, TS, and robot allocation info)
        Tree_Node* ParentNode; // Pointer to the parent node
        std::vector<Tree_Node*> children; // Child nodes in the tree
        Node* automaton_state; // Associated NBA state
        Node* ts_state; // Associated transition system state
        std::vector<bool> robo_task_allocation; // Vector indicating which robot is allocated to which task
        std::vector<uint16_t> times; // Vector of approximate times for each robot to complete its allocated tasks
        std::vector<Point> robotPositions; // Positions of robots at this node
        int8_t batch; // Batch number for the node
        TASK_PROGRESS prog; // Type of the node (pre, tra, suf, oth)
    public:
        
        
        // Constructor
        Tree_Node(uint32_t id, Tree_Node* parent, Node* automatonState, Node* tsState, 
                  std::vector<bool> taskAllocation, std::vector<uint16_t> times, std::vector<Point> robotpositions, int8_t batch, TASK_PROGRESS prog);
        
        Tree_Node(uint32_t id, Tree_Node* parent, Node* automatonState, Node* tsState, int8_t batch);
        // Destructor
        ~Tree_Node();
        
        // Tree structure methods
        Tree_Node* getParent() const;
        void setParent(Tree_Node* newParent);
        const std::vector<Tree_Node*>& getChildren() const;
        void addChild(Tree_Node* child);
        void removeChild(Tree_Node* child);
        
        // Getter methods
        uint32_t getId() const;
        Node* getAutomatonState() const;
        Node* getTSState() const;
        const std::vector<bool>& getRoboTaskAllocation() const;
        bool isRobotAllocated(uint16_t robotIndex) const;
        const std::vector<uint16_t>& getTimes() const;
        const std::vector<Point>& getRobotPositions() const;
        uint16_t getSumOfTimes() const;
        uint16_t getNumUtilizedRobots() const;
        uint16_t getMaxTime() const;
        uint16_t getTimeForRobot(uint16_t robotIndex) const;
        int8_t getBatch() const;
        TASK_PROGRESS getProgress() const;

        
        // Setter methods
        void setAutomatonState(Node* state);
        void setTSState(Node* state);
        void setRoboTaskAllocation(const std::vector<bool>& allocation);
        void setTimes(const std::vector<uint16_t>& newTimes);
        void setBatch(int8_t newBatch);
        void setProgress(TASK_PROGRESS curProg);
        void setId(uint32_t newId);
        void setRobotPositions(const std::vector<Point>& positions);
        bool hasRobotPositions() const;
        
        // Sorting methods
        // Returns vector of (robotIndex, time) pairs sorted by time in ascending order
        // Preserves original times vector; maps each sorted time to its robot index
        std::vector<std::pair<uint16_t, uint16_t>> getSortedTimes() const;
        static std::vector<std::pair<uint16_t, uint16_t>> getSortedTimes(const std::vector<uint16_t>& times);

private:
        // Helper method for quicksort - sorts indices based on their corresponding times
        void quickSortIndices(std::vector<uint16_t>& indices, int low, int high) const;
};

#endif // TREE_NODE_H