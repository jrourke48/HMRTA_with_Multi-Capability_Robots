#ifndef RANDOM_NODE_H
#define RANDOM_NODE_H

#include <vector>
#include "../../Automatons/Edge_Node.h"

class Random_Node {
    // Define the members and methods for the Random_Node class here
    private:
        uint16_t nodeId; // ID of the random node in the linked list
        Node* automatonState; // Pointer to the corresponding node in the planning tree
        std::vector<uint16_t> trueAPs; // Task IDs associated with this node
        std::vector<std::vector<uint8_t>> taskAllocations; //the Task allocation for each region for this node
        std::vector<uint16_t> times; // Task requirements for this node
        uint16_t curmakespan; // Time associated with this node
        Random_Node* next; // Pointer to next node in the path
        //set curmakespan should only be called internally
        void setCurmakespan();

    public:
        Random_Node(uint16_t id, Node* nodePtr, const std::vector<uint16_t> trueAPS, const std::vector<std::vector<uint8_t>>& allocations, std::vector<uint16_t> times);
        Random_Node(uint16_t id, Node* nodePtr, const std::vector<uint16_t> trueAPS, const std::vector<std::vector<uint8_t>>& allocations, std::vector<uint16_t> times, Random_Node* next);
        
        // Node ID getters and setters
        uint16_t getNodeId() const;
        void setNodeId(uint16_t id);
        
        // Automaton state getters and setters
        Node* getautomatonState() const;
        void setautomatonState(Node* nodePtr);
        
        // Task allocation getters and setters
        const std::vector<std::vector<uint8_t>>& getTaskAllocations() const;
        void setTaskAllocations(const std::vector<std::vector<uint8_t>>& allocations);

        const std::vector<uint8_t> getTaskAllocation(uint16_t trueAP) const;
        
        
        // True APs getters and setters
        const std::vector<uint16_t>& getTrueAPs() const;
        void setTrueAPs(const std::vector<uint16_t>& aps);
        
        // Times getters and setters
        const std::vector<uint16_t>& getTimes() const;
        void setTimes(const std::vector<uint16_t>& times);
        
        // Current makespan getters and setters
        uint16_t getCurmakespan() const;
        
        // Linked list navigation
        Random_Node* getNext() const;
        void setNext(Random_Node* node);
};

#endif