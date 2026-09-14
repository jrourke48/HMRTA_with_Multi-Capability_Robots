# Automaton Classes & Spot Integration

Comprehensive guide to the abstract automaton hierarchy and Spot library integration for LTL-based multi-robot planning.

## Overview

This module provides a unified interface for working with different types of automata needed for formal task planning:

- **Automaton** - Abstract base class defining the automaton interface
- **BuchiAutomaton** - Generalized Büchi automata from LTL formulas
- **TransitionSystem (TS)** - Discrete region transition graphs
- **ProductAutomaton** - Product of TS and Büchi automata
- **Node/Edge** - Graph data structures for states and transitions

## Architecture

```
Automaton (abstract base)
    ├── BuchiAutomaton
    │   └── Represents acceptance conditions and transitions
    ├── TransitionSystem (TS)  
    │   └── Discrete workspace regions with motion model
    └── ProductAutomaton
        └── States = (TS_state, Buchi_state) pairs
```

## Core Classes

### Automaton (Abstract Base)

Base class providing the interface for all automaton types.

**Header**: `Automaton.h`

```cpp
class Automaton {
protected:
    uint32_t numNodes = 0;
    uint32_t numEdges = 0;
    std::map<uint32_t, Node*> nodeMap;

public:
    // Virtual interface
    virtual void add_Node(Node* node) = 0;
    virtual bool isAdjacent(uint32_t srcId, uint32_t dstId) const = 0;
    
    // Common methods
    uint32_t getnumStates() const { return numNodes; }
    uint32_t getnumEdges() const { return numEdges; }
    Node* getNode(uint32_t nodeId) const;
    const std::map<uint32_t, Node*>& getNodes() const;
};
```

**Key Methods**:
- `add_Node()` - Add state to automaton
- `isAdjacent()` - Check if direct transition exists
- `getNode()` - Retrieve state by ID
- `getNodes()` - Get all states

### Node & Edge Structures

**Header**: `Edge_Node.h`

Lightweight data structures for graph representation.

#### Node

```cpp
class Node {
private:
    std::string label;           // State name/label
    uint32_t id;                 // Unique ID
    uint32_t numEdges;
    std::vector<Edge> edges;     // Outgoing edges

public:
    // Constructors
    Node(uint32_t id);
    Node(uint32_t id, const std::string& label);
    
    // Access methods
    uint32_t getId() const;
    std::string getLabel() const;
    std::vector<Edge> getEdges() const;
    
    // Modification
    void addEdge(const Edge& edge);
    bool isAdjacent(uint32_t dstId) const;
    void setLabel(const std::string& label);
};
```

**Usage**:
```cpp
// Create a state
Node* stateA = new Node(0, "RegionA");

// Add transition to another state
Edge transitionToB(1, "action_move");  // destination ID, label
stateA->addEdge(transitionToB);

// Query adjacency
if (stateA->isAdjacent(1)) {
    std::cout << "Can reach state 1\n";
}
```

#### Edge

```cpp
class Edge {
private:
    uint32_t dstId;              // Destination node ID
    std::string label;           // Transition label/condition
    uint32_t weight;             // For weighted edges

public:
    // Constructors
    Edge(uint32_t dstId);
    Edge(uint32_t dstId, const std::string& label);
    Edge(uint32_t dstId, const std::string& label, uint32_t weight);
    
    // Access
    uint32_t getDstId() const;
    std::string getLabel() const;
    uint32_t getWeight() const;
    
    // Modification
    void setLabel(const std::string& label);
    void setWeight(uint32_t w);
};
```

**Usage**:
```cpp
// Create labeled edge
Edge e1(5, "p0 & p1");           // Condition: p0 AND p1
Edge e2(3, "move_north", 1);     // With weight

// Query edge properties
std::string condition = e1.getLabel();
uint32_t dest = e1.getDstId();
```

## Specific Automaton Types

### BuchiAutomaton

Represents generalized Büchi automata from LTL specifications.

**Header/Source**: `BuchiAutomaton.h/cpp`

```cpp
class BuchiAutomaton : public Automaton {
private:
    std::vector<uint32_t> initialStates;
    std::vector<uint32_t> acceptingStates;
    std::map<uint32_t, std::set<uint32_t>> acceptanceSets;

public:
    BuchiAutomaton();
    
    // Add states/transitions via base class
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;
    
    // Accepting state management
    void setAccepting(uint32_t stateId);
    bool isAccepting(uint32_t stateId) const;
    const std::vector<uint32_t>& getAcceptingStates() const;
    
    // Initial states
    void setInitial(uint32_t stateId);
    bool isInitial(uint32_t stateId) const;
};
```

**Usage**:
```cpp
// Create Büchi automaton
BuchiAutomaton* buchi = new BuchiAutomaton();

// Add states
Node* q0 = new Node(0, "q0");
Node* q1 = new Node(1, "q1");
buchi->add_Node(q0);
buchi->add_Node(q1);

// Add transitions (with labels from LTL)
q0->addEdge(Edge(1, "p0"));      // If p0 true
buchi->getNode(0)->addEdge(Edge(1, "p0"));

// Mark accepting state
buchi->setAccepting(1);
buchi->setInitial(0);

// Query
if (buchi->isAccepting(1)) {
    std::cout << "State 1 is accepting\n";
}
```

### TransitionSystem (TS)

Represents discrete regions and allowed motion.

**Header/Source**: `TS.h/cpp`

```cpp
class TS : public Automaton {
private:
    std::vector<uint32_t> initialStates;
    std::unordered_map<uint32_t, uint32_t> stateToNodeId;

public:
    TS();
    
    // Base class implementation
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;
    
    // Initial state management
    void setInitial(uint32_t stateId);
    bool isInitial(uint32_t stateId) const;
    const std::vector<uint32_t>& getInitialStates() const;
};
```

**Usage**:
```cpp
// Create transition system for 3 regions
TS* ts = new TS();

// Add region states
Node* regionA = new Node(0, "RegionA");
Node* regionB = new Node(1, "RegionB");
Node* regionC = new Node(2, "RegionC");

ts->add_Node(regionA);
ts->add_Node(regionB);
ts->add_Node(regionC);

// Add transitions (motion between regions)
regionA->addEdge(Edge(1));       // Can move A → B
regionA->addEdge(Edge(2));       // Can move A → C
regionB->addEdge(Edge(0));       // Can move B → A
// etc.

// Set initial region
ts->setInitial(0);

// Check connectivity
if (ts->isAdjacent(0, 1)) {
    std::cout << "Can move from A to B\n";
}
```

### ProductAutomaton

Represents the product T ⊗ B of a transition system and Büchi automaton.

**Header/Source**: `ProductAutomaton.h/cpp`

```cpp
class ProductAutomaton : public Automaton {
private:
    std::vector<std::pair<uint32_t, uint32_t>> stateMapping;
    std::vector<uint32_t> acceptingStates;

public:
    ProductAutomaton();
    ProductAutomaton(spot::twa_graph_ptr spotAutomaton);
    
    // Base class
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;
    
    // Accepting states
    void setAccepting(uint32_t stateId);
    bool isAccepting(uint32_t stateId) const;
    
    // State mapping: product state → (TS state, Buchi state)
    void addStateMapping(uint32_t productState, uint32_t tsState, uint32_t automataState);
    std::pair<uint32_t, uint32_t> getStateMapping(uint32_t productState) const;
};
```

**Usage**:
```cpp
// Create product automaton from Spot automaton
spot::twa_graph_ptr buchi = spot::translate("F p0", false, false, false);
ProductAutomaton* product = new ProductAutomaton(buchi);

// In manual construction:
ProductAutomaton* manual = new ProductAutomaton();

// Add product states (pairs of TS and Buchi states)
for (uint32_t s = 0; s < ts_states; ++s) {
    for (uint32_t q = 0; q < buchi_states; ++q) {
        uint32_t productState = s * buchi_states + q;
        Node* pnode = new Node(productState);
        manual->add_Node(pnode);
        
        // Map to original states
        manual->addStateMapping(productState, s, q);
        
        // Mark accepting if Buchi state is accepting
        if (buchi->state_is_accepting(q)) {
            manual->setAccepting(productState);
        }
    }
}

// Query state mapping
auto mapping = product->getStateMapping(stateId);
uint32_t ts_state = mapping.first;
uint32_t buchi_state = mapping.second;
```

## Spot Library Integration

The Spot library handles LTL-to-Büchi conversion and manipulation.

### Setup

**Installation** (Ubuntu/Debian):
```bash
sudo apt-get install libspot-dev libbdd-dev
```

**Include**:
```cpp
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/bdd.hh>
#include <bddx.h>
```

### LTL to Büchi Conversion

**Translate LTL formula to automaton**:
```cpp
#include <spot/twaalgos/translate.hh>

// Convert LTL to Büchi automaton
std::string ltlFormula = "F (p0 & F p1)";  // Eventually p0, then p1
spot::twa_graph_ptr buchi = spot::translate(ltlFormula, false, false, false);

if (!buchi) {
    std::cerr << "Failed to translate LTL formula\n";
    return -1;
}

std::cout << "Büchi states: " << buchi->num_states() << "\n";
std::cout << "Accepting sets: " << buchi->acc().num_sets() << "\n";
```

**Formula syntax**:
```
F phi        - Eventually phi
G phi        - Always phi  
X phi        - Next phi
phi U psi    - Until
phi & psi    - AND
phi | psi    - OR
!phi         - NOT
p0, p1, ...  - Propositions
```

**Examples**:
```cpp
"F p0"                    // Eventually visit region 0
"G F p0"                  // Infinitely often visit region 0
"F (p0 & F p1)"          // Eventually 0, then eventually 1
"!(p0 & p1)"             // Never (0 and 1 simultaneously)
"F p0 | F p1"            // Eventually 0 or eventually 1
"G (p0 -> X p1)"         // If in 0, next must be in 1
```

### Querying Spot Automaton

```cpp
// Get state information
unsigned numStates = buchi->num_states();
unsigned initialState = buchi->get_init_state_number();

// Check if state is accepting
for (unsigned i = 0; i < numStates; ++i) {
    if (buchi->state_is_accepting(i)) {
        std::cout << "State " << i << " is accepting\n";
    }
}

// Iterate over edges (transitions)
for (const auto& edge : buchi->edges()) {
    unsigned src = edge.src;
    unsigned dst = edge.dst;
    bdd cond = edge.cond;  // Boolean condition
    
    // Get condition as formula string
    std::string label = bdd_format_formula(buchi->get_dict(), cond);
    std::cout << "Transition: " << src << " --[" << label << "]-> " << dst << "\n";
}

// Get state names if available
auto names = buchi->get_named_prop<std::map<unsigned, std::string>>("state-names");
if (names) {
    for (const auto& pair : *names) {
        std::cout << "State " << pair.first << ": " << pair.second << "\n";
    }
}
```

### Export to GraphViz

```cpp
#include <spot/twaalgos/dot.hh>

// Output as DOT format (for Graphviz)
std::cout << spot::dot_reachable(buchi) << std::endl;

// Or save to file
std::ofstream out("buchi.dot");
out << spot::dot_reachable(buchi);
out.close();

// Convert to image
// dot -Tpng buchi.dot -o buchi.png
```

## Complete Example

Building and using all automaton types together:

```cpp
#include "Automaton.h"
#include "BuchiAutomaton.h"
#include "TS.h"
#include "ProductAutomaton.h"
#include "Edge_Node.h"
#include <spot/twaalgos/translate.hh>
#include <iostream>

int main() {
    // ========== 1. Create Transition System ==========
    TS* ts = new TS();
    
    // Three regions
    ts->add_Node(new Node(0, "RegionA"));
    ts->add_Node(new Node(1, "RegionB"));
    ts->add_Node(new Node(2, "RegionC"));
    
    // Add transitions
    ts->getNode(0)->addEdge(Edge(1, "move_AB"));
    ts->getNode(1)->addEdge(Edge(2, "move_BC"));
    ts->getNode(1)->addEdge(Edge(0, "move_BA"));
    ts->getNode(2)->addEdge(Edge(1, "move_CB"));
    
    ts->setInitial(0);
    
    std::cout << "TS: " << ts->getnumStates() << " states, " 
              << ts->getnumEdges() << " transitions\n";
    
    // ========== 2. Create Büchi Automaton from LTL ==========
    std::string ltl = "F (p0 & F p1)";  // Visit A then B
    spot::twa_graph_ptr spotBuchi = spot::translate(ltl, false, false, false);
    
    if (!spotBuchi) {
        std::cerr << "LTL translation failed\n";
        return -1;
    }
    
    std::cout << "\nLTL: " << ltl << "\n";
    std::cout << "Büchi: " << spotBuchi->num_states() << " states\n";
    
    // ========== 3. Build Product Automaton ==========
    ProductAutomaton* product = new ProductAutomaton(spotBuchi);
    
    std::cout << "\nProduct: " << product->getnumStates() << " states, " 
              << product->getnumEdges() << " transitions\n";
    
    // ========== 4. Query States ==========
    std::cout << "\nAccepting states in product:\n";
    for (const auto& nodePair : product->getNodes()) {
        if (product->isAccepting(nodePair.first)) {
            auto mapping = product->getStateMapping(nodePair.first);
            std::cout << "  Product state " << nodePair.first 
                      << " = (TS:" << mapping.first 
                      << ", Buchi:" << mapping.second << ")\n";
        }
    }
    
    // ========== Cleanup ==========
    delete ts;
    delete product;
    
    return 0;
}
```

## Building

### With G++

```bash
g++ -std=c++17 \
    -I/usr/include/spot \
    -I/usr/include/eigen3 \
    MyProgram.cpp \
    ProductAutomaton.cpp \
    BuchiAutomaton.cpp \
    TS.cpp \
    -o my_program \
    -lspot -lbddx
```

### With CMake

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyAutomatonProject)

set(CMAKE_CXX_STANDARD 17)

find_package(Spot REQUIRED)

add_executable(my_program 
    MyProgram.cpp
    ProductAutomaton.cpp
    BuchiAutomaton.cpp
    TS.cpp
)

target_link_libraries(my_program Spot::Spot)
```

## Common Patterns

### Pattern 1: Create TS manually

```cpp
TS* workspace = new TS();

// Add regions
for (int i = 0; i < 5; ++i) {
    workspace->add_Node(new Node(i, "Region_" + std::to_string(i)));
}

// Add grid transitions (4-connected)
for (int i = 0; i < 5; ++i) {
    for (int j = i+1; j < 5; ++j) {
        if (areAdjacent(i, j)) {  // Define adjacency
            workspace->getNode(i)->addEdge(Edge(j));
            workspace->getNode(j)->addEdge(Edge(i));  // Bidirectional
        }
    }
}

workspace->setInitial(0);
```

### Pattern 2: Translate LTL and build product

```cpp
// Translate LTL
spot::twa_graph_ptr buchi = spot::translate(ltlFormula, false, false, false);

// Create product from Spot automaton
ProductAutomaton* product = new ProductAutomaton(buchi);

// Access state information
for (auto& node : product->getNodes()) {
    auto [ts_id, buchi_id] = product->getStateMapping(node.first);
    
    if (product->isAccepting(node.first)) {
        // Process accepting state
    }
}
```

### Pattern 3: Search for accepting path

```cpp
// BFS to find accepting state
std::queue<uint32_t> q;
std::set<uint32_t> visited;

q.push(initialState);
visited.insert(initialState);

while (!q.empty()) {
    uint32_t current = q.front();
    q.pop();
    
    if (product->isAccepting(current)) {
        std::cout << "Found accepting state: " << current << "\n";
        break;
    }
    
    Node* node = product->getNode(current);
    if (node) {
        for (const auto& edge : node->getEdges()) {
            uint32_t next = edge.getDstId();
            if (visited.find(next) == visited.end()) {
                visited.insert(next);
                q.push(next);
            }
        }
    }
}
```

## Debugging Tips

### Print automaton info

```cpp
std::cout << "States: " << automaton->getnumStates() << "\n";
std::cout << "Edges: " << automaton->getnumEdges() << "\n";

for (const auto& nodePair : automaton->getNodes()) {
    Node* node = nodePair.second;
    std::cout << "State " << node->getId() << " (" << node->getLabel() << "): ";
    for (const auto& edge : node->getEdges()) {
        std::cout << edge.getDstId() << " ";
    }
    std::cout << "\n";
}
```

### Export for visualization

```cpp
// Save Spot automaton as DOT
std::ofstream out("automaton.dot");
out << spot::dot_reachable(spotBuchi);
out.close();

// View with: dot -Tpng automaton.dot -o automaton.png
```

### Check Spot installation

```bash
pkg-config --cflags --libs spot
spot-x.cfg --version  # Check Spot version
```

## References

- **Spot Library**: https://spot.lrde.epita.fr/
- **LTL Documentation**: https://spot.lrde.epita.fr/concepts.html
- **Büchi Automata**: https://en.wikipedia.org/wiki/Büchi_automaton
- **LTL Specification**: https://en.wikipedia.org/wiki/Linear_temporal_logic

## Troubleshooting

### LTL formula not translating

**Problem**: `spot::translate()` returns null
- **Check**: Formula syntax (use F, G, X, U, &, |, !)
- **Check**: Proposition names are p0, p1, p2, etc.
- **Check**: Spot is installed: `pkg-config --list-all | grep spot`

### Compilation errors

**Problem**: "undefined reference to `spot::...`"
- **Fix**: Add `-lspot` to linker flags
- **Fix**: Check Spot include path: `-I/usr/include/spot`

**Problem**: "error: no matching function for call to `translate`"
- **Fix**: Verify Spot headers: `#include <spot/twaalgos/translate.hh>`
- **Fix**: Include `<bddx.h>` for BDD support

### Automaton has no edges

**Problem**: Added nodes but no transitions appear
- **Check**: Did you call `add_Node()` for all nodes first?
- **Check**: Are nodes added to the automaton before adding edges?
- **Check**: Is `addEdge()` called on the correct node?

## Next Steps

1. **Basic Usage**: See VoronoiPruningExample.cpp
2. **Advanced**: See CompleteWorkflowExample.cpp
3. **Integration**: Link with PlanningSystem module
4. **Deployment**: Integrate into your main project

## Files Reference

| File | Purpose |
|------|---------|
| `Automaton.h` | Abstract base class |
| `BuchiAutomaton.h/cpp` | Büchi automaton implementation |
| `TS.h/cpp` | Transition system implementation |
| `ProductAutomaton.h/cpp` | Product automaton implementation |
| `Edge_Node.h` | Node and Edge data structures |

---

**Last Updated**: March 2026
**Spot Version**: ≥ 2.14.4
**C++ Standard**: 17 or later
