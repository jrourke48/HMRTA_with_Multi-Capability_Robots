#ifndef PRODUCT_AUTOMATON_H
#define PRODUCT_AUTOMATON_H

#include "Automaton.h"
#include "TS.h"
#include "Environment/Environment.h"
#include "BuchiAutomaton.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>

class ProductAutomaton : public Automaton {
private:
    std::map<uint16_t, std::string> stateMapping;  // Maps product state ID to its label
    std::vector<uint16_t> acceptingStates;  // Set of accepting states
    spot::twa_graph_ptr spotAutomaton;  // Pointer to the underlying Spot automaton
    const Environment* envPtr = nullptr;  // Store pointer to Environment for later use
    const MultiRobotSystem* mrsPtr = nullptr;  // Store pointer to MultiRobotSystem for later use
    
    // DOT parsing helper methods
    void parseProductFromDot(const std::string& dotContent);
    std::string extractLabelFromDotBrackets(const std::string& content) const;
public:
    ProductAutomaton();
    
    // Constructor from Spot twa_graph
    ProductAutomaton(spot::twa_graph_ptr spotAutomaton);

    //Constructor from individual components (TS, Mult-Robot System, and automaton states)
    ProductAutomaton(const Environment& env, const MultiRobotSystem& mrs, const BuchiAutomaton& buchiAutomaton);
    
    // Compute the optimal accepting path starting from the given state
    std::tuple<std::vector<uint16_t>, uint32_t> OptimalAcceptingPath();
    
    ~ProductAutomaton() override;

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint16_t srcId, uint16_t dstId) const override;
    
    void setAccepting(uint16_t stateId);
    bool isAccepting(uint16_t stateId) const;
    const std::vector<uint16_t>& getAcceptingStates() const;


    // Product-specific methods
    void initCartesianStateMapping(std::vector<std::vector<uint16_t>> cartesianStates);
    void addStateMapping(uint16_t productState, const std::string& label);
    void initStateMapping(const std::string& dotContent);
    void updateStateMapping(const std::string& dotContent);
    std::string getStateMapping(uint16_t productState) const;
    // getter for the id in the label
    uint16_t getIdFromLabel(const std::string& label) const;
    //replacing the old label for the new label
    std::string replaceLabel(const std::string& oldLabel, const std::string& additionalLabel);
    // Getter for Spot automaton
    spot::twa_graph_ptr getSpotAutomaton() const { return spotAutomaton; }
    uint32_t getEdgeWeight(Node* srcNode, Node* dstNode) const;
};

#endif
