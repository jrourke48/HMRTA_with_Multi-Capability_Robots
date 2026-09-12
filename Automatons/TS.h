#ifndef TS_H
#define TS_H

#include "Automaton.h"
#include "Edge_Node.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_map>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>

class TS : public Automaton {
private:
    std::vector<uint16_t> initialStates;  // Track which states are initial states
    std::unordered_map<uint16_t, uint32_t> stateToNodeId;  // Map from state index to node ID

public:
    TS();
    
    // Copy constructor for deep copying
    TS(const TS& other);
    
    ~TS() override;

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    void removeNode(uint16_t nodeId);
    bool isAdjacent(uint16_t srcId, uint16_t dstId) const override;

    // Get all adjacent nodes for a given node
    std::vector<uint16_t> getAdjacent(uint16_t nodeId) const;

    // TS-specific methods
    void setInitial(uint16_t stateId);
    bool isInitial(uint16_t stateId) const;
    const std::vector<uint16_t>& getInitialStates() const;

    // Convert TS to Spot automaton
    // If dict is provided, use it; otherwise create a new BDD dictionary
    spot::twa_graph_ptr toSpotAutomaton(spot::bdd_dict_ptr dict = nullptr) const;
};

#endif
