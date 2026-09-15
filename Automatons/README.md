# Automatons Module

Comprehensive guide to the abstract automaton hierarchy and Spot library integration for LTL-based multi-robot planning.

## Overview

The Automatons module provides a unified hierarchy of automaton classes for formal task planning with LTL specifications. It serves as the bridge between high-level temporal logic formulas and low-level planning algorithms.

This module is responsible for:

1. **Abstract Automaton Interface** - Common methods and properties across all automata types
2. **Büchi Automaton Generation** - LTL-to-automaton conversion via Spot library integration
3. **Transition System Representation** - Discrete workspace modeling with state and edge abstractions
4. **Product Automaton Construction** - Synthesis of combined (TS × Büchi) state spaces
5. **Graph Data Structures** - Node and edge abstractions with labeling and properties
6. **GBA Semantics** - Edge-indexed acceptance set tracking and parsing

## Class Hierarchy

```
Automaton (abstract base)
├── BuchiAutomaton
│   └── Generalized Büchi Automata with edge-indexed acceptance
├── TS (TransitionSystem)
│   └── Discrete workspace regions with motion model
└── ProductAutomaton
    └── Product of TS and Büchi with combined state space
```

## Classes and Components

### 1. Automaton (Abstract Base Class)

**File**: `Automaton.h`

Defines the abstract interface all automaton types must implement.

#### Key Members

```cpp
protected:
    uint32_t numNodes;                           // Total number of states
    uint32_t numEdges;                           // Total number of transitions
    std::map<uint32_t, Node*> nodeMap;          // Maps node IDs to node pointers
```

#### Key Methods

```cpp
// Pure virtual - must be implemented by subclasses
virtual void add_Node(Node* node) = 0;
virtual bool isAdjacent(uint32_t srcId, uint32_t dstId) const = 0;

// Common interface
uint32_t getnumStates() const;
uint32_t getnumEdges() const;
Node* getNode(uint32_t nodeId) const;
void add_Edge(Edge* edge);
```

#### Properties

- **Extensible**: Designed for adding new automaton types
- **Type-safe**: Uses `Node*` pointers with virtual methods for polymorphism
- **Memory Managed**: Subclasses responsible for node/edge cleanup

---

### 2. BuchiAutomaton

**Files**: `BuchiAutomaton.h`, `BuchiAutomaton.cpp`

Implements Generalized Büchi Automata (GBA) with full LTL support via Spot library.

#### Features

- **LTL Parsing**: Converts LTL formulas to automata using Spot
- **GBA Semantics**: Edge-indexed acceptance set tracking
- **DOT Format**: Imports and exports automata in DOT graph format
- **Finite/Infinite Detection**: Classifies automata based on acceptance conditions
- **Edge Label Parsing**: Extracts APs and acceptance marks from DOT edge labels

#### Key Methods

**Automaton Generation**
```cpp
// Generate from LTL formula string
void generateFromLTLString(const string& ltlFormula);

// Parse DOT format content
void parseDotFormat(const string& dotContent);
```

**Acceptance Set Handling**
```cpp
// Parse GBA acceptance set numbers from "Inf(0)&Inf(1)" format
vector<uint16_t> parseglobalAcceptanceSets(const string& label);

// Determine if automaton is finite (no cyclic acceptance) or infinite
bool isInfinite() const { return _isInfinite; }

// Check if automaton contains GBA elements
bool checkIsInfinite(const string& dotContent);
```

**Edge Label Processing**
```cpp
// Extract label from DOT edge with acceptance marks
// Format: "p0 & p1:{0,1}" where ':' separates APs from acceptance sets
string extractLabelFromDotBrackets(const string& content);
```

#### Key Members

```cpp
vector<uint16_t> acceptingStates;    // GBA acceptance set IDs (parsed from labels)
bool _isFinite;                      // Whether automaton is finite
std::vector<Edge_Node*> edges;       // All edges with acceptance information
```

#### Label Format

**Edge labels in DOT format**:
- Simple: `"p0"` - single atomic proposition
- Complex: `"p0 & p1 | p2"` - boolean expressions
- With Acceptance: `"p0 & p1:{0,1,2}"` - APs and GBA acceptance marks

**Parsing**:
1. Split by `:` to separate AP section from acceptance marks
2. Parse AP section for atomic propositions and boolean structure
3. Parse acceptance marks from `{x,y,z}` format using comma delimiter

#### GBA Edge Filtering Rules

During edge extraction, edges are filtered based on AP structure:

1. **Single AP or OR-connected APs**: Included (can be satisfied individually)
   - `"p0"` ✓
   - `"p0 | p1"` ✓

2. **Multiple AND-connected APs**: **Completely Excluded** (require simultaneous execution)
   - `"p0 & p1"` ✗ (both true simultaneously)
   - `"p0 & p1 & p2"` ✗

3. **Negated APs**: Always skipped
   - `"!p0"` ✗

**Rationale**: Robot capabilities are checked per state; edges requiring multiple APs simultaneously cannot be decomposed.

#### Spot Integration

```cpp
spot::formula spotFormula;           // Internal Spot representation
spot::twa_graph_ptr spotTWA;         // Spot automaton graph
```

Uses Spot's conversion from LTL to Büchi automata:
- Handles complex temporal operators (F, G, X, U, R, W, etc.)
- Generates minimized automata
- Provides DOT export functionality

---

### 3. TS (Transition System)

**Files**: `TS.h`, `TS.cpp`

Represents the discrete workspace as a state transition graph.

#### Purpose

Abstracts continuous or complex environments into discrete states and transitions suitable for formal planning.

#### Class Definition

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

#### Key Members

- `std::map<uint32_t, Node*> nodeMap` - State ID to state object mapping
- `std::vector<Edge_Node*> edgeList` - All transitions in the system
- `uint32_t initialState` - Entry point for planning

#### Key Methods

```cpp
// State management
void addState(uint32_t id, const string& label = "");
Node* getState(uint32_t id) const;

// Transition management
void addTransition(uint32_t fromState, uint32_t toState, 
                   const string& label, double cost = 1.0);
vector<uint32_t> getSuccessors(uint32_t state) const;
```

#### Usage

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

// Set initial region
ts->setInitial(0);

// Check connectivity
if (ts->isAdjacent(0, 1)) {
    std::cout << "Can move from A to B\n";
}
```

#### Atomic Propositions

States are labeled with atomic propositions that become true when the transition system reaches those states:

```cpp
// Example TS with propositions
TS ts;
ts.addState(0, "");              // Initial state
ts.addState(1, "at_location_A");
ts.addState(2, "at_location_B");
ts.addTransition(0, 1, "move_to_A", 5.0);
ts.addTransition(1, 2, "move_to_B", 8.0);
```

TS can be paired with grid-based or continuous workspace representations for detailed environment modeling.

---

### 4. ProductAutomaton

**Files**: `ProductAutomaton.h`, `ProductAutomaton.cpp`

Combines a TS and Büchi automaton into a single unified state space.

#### Concept

The product automaton merges two automaton spaces:
- **TS Component**: Represents feasible robot motions
- **Büchi Component**: Represents task progress tracking

**Product State**: `(buchi_state, robot_state1, robot_)` pair

#### Class Definition

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

#### Key Members

- `TS* transitionSystem` - Reference to workspace model
- `BuchiAutomaton* buchiAutomaton` - Reference to task automaton
- `std::map<pair<uint32_t, uint32_t>, uint32_t> productStates` - Product state mapping

#### Product Space Growth

- **Finite TS** with n states
- **Büchi automaton** with m states  
- **Product space**: Up to n × m states

The product automaton is the actual planning domain:
- **States** = valid (workspace_region, task_phase) combinations
- **Transitions** = motions that advance both workspace and task
- **Accepting paths** = complete task execution satisfying both TS and Büchi conditions

#### Usage

```cpp
// Create product automaton from Spot automaton
spot::twa_graph_ptr buchi = spot::translate("F p0", false, false, false);
ProductAutomaton* product = new ProductAutomaton(buchi);

// Manual construction:
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

---

### 5. Node and Edge Classes

**File**: `Edge_Node.h`

Lightweight data structures for graph representation.

#### Node (State Representation)

```cpp
class Node {
private:
    std::string label;           // State name/label or atomic propositions
    uint32_t id;                 // Unique ID
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

#### Edge (Transition Representation)

```cpp
class Edge {
private:
    uint32_t dstId;              // Destination node ID
    std::string label;           // Transition label/condition or APs
    uint32_t weight;             // For weighted edges
    std::set<uint16_t> acceptanceMarks; // GBA acceptance sets

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

**Label Format in GBA**:
- Simple: `"p0"` - single atomic proposition
- Complex: `"p0 & p1 | p2"` - boolean expressions
- With Acceptance: `"p0 & p1:{0,1}"` - APs and GBA acceptance marks

Parsing: Split by `:` to separate AP section from acceptance marks `{x,y,z}`

---

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

---

## Edge-Indexed AP Collection with GBA Support

### Algorithm: `collectAPsFromEdgesByIndexWithAcceptance()`

Extracts atomic propositions from automaton edges while respecting GBA acceptance semantics.

**Location**: `TaskAllocationAlgorithms.cpp` (lines 800-870)

**Input**:
- `edges`: Vector of edge labels (strings)

**Output**:
- `outEdgeAPIds`: Vector of sets; `outEdgeAPIds[i]` = AP IDs for edge i
- `outEdgeAcceptanceSets`: Vector of sets; `outEdgeAcceptanceSets[i]` = acceptance set IDs for edge i

**Process**:

1. **Split by Colon**: Separate APs from acceptance marks
   ```
   "p0 & p1:{0,1}" → AP part: "p0 & p1", Acceptance part: "{0,1}"
   ```

2. **Parse AP Part**: Extract individual APs while checking for 2+ AND-ed APs
   - OR clauses are split by `|`
   - AND clauses within each OR are checked
   - If any AND clause has 2+ true APs: **skip entire edge** (use `continue`)

3. **Parse Acceptance Part**: Extract set IDs from `{x,y,z}` format
   - Split by `,`
   - Convert each to uint16_t
   - Store in set for deduplication

4. **Populate Outputs**: Add extracted data to output vectors at correct edge index

**Example**:

```
Edge 0: "p0 & p1:{0,1}"     → SKIPPED (2 AND-ed APs)
Edge 1: "p0 | p1:{0}"       → AP IDs: {0, 1}, Acceptance: {0}
Edge 2: "p2:{1,2}"          → AP IDs: {2}, Acceptance: {1, 2}
Edge 3: "!p3:{0}"           → SKIPPED (negated AP)
```

**Output Vectors**:
```cpp
outEdgeAPIds[1] = {0, 1}     // OR clause, both included
outEdgeAPIds[2] = {2}
outEdgeAcceptanceSets[1] = {0}
outEdgeAcceptanceSets[2] = {1, 2}
```

---

## GBA Semantics - Acceptance Set Tracking

### Concept

In GBA, acceptance conditions track which "acceptance sets" each edge belongs to. A path is accepting if it visits all required acceptance sets infinitely often (for infinite automata) or at least once (for finite automata).

### Implementation

**Per-Edge Tracking**:
```cpp
edgeAcceptanceSets[edgeIdx] = {0, 1}  // Edge belongs to acceptance sets 0 and 1
```

**Global Acceptance State**:
```cpp
vector<uint16_t> acceptingSets = gba->getAcceptingStates(); // e.g., {0, 1, 2}
```

### Progress Increment Logic

**When Edge is Traversed**:
1. Retrieve acceptance marks from edge: `{0, 1}`
2. Remove from global acceptingSets: `{0, 1, 2}` → `{2}`
3. When acceptingSets becomes empty: increment progress phase

```cpp
if (nba->isInfinite()) {
    std::set<uint16_t> curracceptingset = edgeAcceptanceSets[edgeIdx];
    for (uint16_t setNum : curracceptingset) {
        auto it = std::find(acceptingSets.begin(), acceptingSets.end(), setNum);
        if (it != acceptingSets.end()) {
            acceptingSets.erase(it);
        }
    }
}

if (acceptingSets.empty()) {
    int newProgressInt = static_cast<int>(currentNode->getProgress()) + 1;
    Tree_Node::TASK_PROGRESS newProgress = static_cast<Tree_Node::TASK_PROGRESS>(newProgressInt);
    newNode->setProgress(newProgress);
}
```

---

## Finite vs. Infinite Automata

### Detection

```cpp
bool isInfinite = ba->isInfinite();  // Automatically detected based on DOT content
```

**Finite**: No "Inf(" patterns in edge labels (all acceptance conditions are one-time events)

**Infinite**: Contains "Inf(" in edge labels (acceptance conditions must be satisfied repeatedly)

### Impact on Planning

**Finite Automata**:
- All acceptance sets must be visited exactly once
- Progress terminates at OTH state
- Aggressive pruning: removes duplicate (state, progress) pairs

**Infinite Automata**:
- Acceptance sets must be revisited infinitely often
- Progress may cycle: PRE→TRA→SUF→OTH→PRE
- Relaxed pruning: allows multiple traversals of same (state, progress) pairs

---

## Building and Compiling

### Compilation Flags

```bash
-std=c++20          # C++20 standard (for modern features)
-Wall -Wextra       # Warnings
-g3                 # Debug symbols
-I"spot/include"    # Spot library headers
-L"spot/lib"        # Spot library linking
-lspot -lbddx       # Link Spot and BDD libraries
```

### Example Build

```bash
g++ -std=c++20 -Wall -Wextra -g3 \
    -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev" \
    -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev/buddy/src" \
    -L"C:/Users/johnn/Spot Library/spot-2.14.4.dev/lib" \
    BuchiAutomaton.cpp -o BuchiAutomaton.o -lspot -lbddx
```

---

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

---

## Usage Examples

### Creating a Büchi Automaton from LTL

```cpp
#include "BuchiAutomaton.h"

BuchiAutomaton* gba = new BuchiAutomaton();

// Generate from LTL formula
gba->generateFromLTLString("F(\"p0\") & F(\"p1\")");

// Check if infinite
if (gba->isInfinite()) {
    cout << "Infinite automaton (requires cyclic execution)" << endl;
}

// Get acceptance sets
vector<uint16_t> acceptingSets = gba->getAcceptingStates();
cout << "Acceptance sets to visit: ";
for (auto s : acceptingSets) cout << s << " ";
cout << endl;
```

### Working with Transition Systems

```cpp
#include "TS.h"

TS ts;

// Add states
ts.addState(0, "");               // Initial
ts.addState(1, "at_location_A");
ts.addState(2, "at_location_B");

// Add transitions
ts.addTransition(0, 1, "move", 5.0);
ts.addTransition(1, 2, "move", 8.0);
ts.addTransition(2, 1, "backtrack", 3.0);

// Query
vector<uint32_t> successors = ts.getSuccessors(0);  // Get states reachable from state 0
```

### Creating a Product Automaton

```cpp
#include "ProductAutomaton.h"
#include <spot/twaalgos/translate.hh>

// Create product from LTL formula and transition system
std::string ltl = "F p0 & G F p1";
spot::twa_graph_ptr buchi = spot::translate(ltl, false, false, false);
ProductAutomaton* product = new ProductAutomaton(buchi);

// Query accepting states
for (uint32_t state = 0; state < product->getnumStates(); ++state) {
    if (product->isAccepting(state)) {
        auto [ts_state, buchi_state] = product->getStateMapping(state);
        cout << "State (" << ts_state << ", " << buchi_state << ") is accepting\n";
    }
}
```

TS* ts = new TS();
// ... populate TS ...

BuchiAutomaton* gba = new BuchiAutomaton();
gba->generateFromLTLString("F(\"p0\") & F(\"p1\")");

ProductAutomaton* product = new ProductAutomaton(ts, gba);
product->constructProduct();

// Query product space
vector<uint32_t> productSuccessors = product->getProductSuccessors(0);
```

---

## File Reference

| File | Purpose |
|------|---------|
| `Automaton.h` | Abstract base class for all automata |
| `BuchiAutomaton.h` / `.cpp` | GBA implementation with Spot integration |
| `TS.h` / `.cpp` | Transition system for workspace modeling |
| `ProductAutomaton.h` / `.cpp` | Product space of TS and Büchi |
| `Edge_Node.h` | Node and edge data structures |

---

## Debugging Tips

### Checking Automaton Structure

```cpp
// Print number of states/edges
cout << "States: " << gba->getnumStates() << endl;
cout << "Edges: " << gba->getnumEdges() << endl;

// Export DOT for visualization
gba->exportDot("automaton.dot");
```

### Verifying Edge Labels

Enable debug output in `collectAPsFromEdgesByIndexWithAcceptance()`:
```cpp
std::cerr << "Edge " << edgeIdx << ": " << edges[edgeIdx] << std::endl;
std::cerr << "  APs: "; 
for (auto ap : edgeAPIds[edgeIdx]) std::cerr << ap << " ";
std::cerr << "  Acceptance: ";
for (auto acc : edgeAcceptanceSets[edgeIdx]) std::cerr << acc << " ";
std::cerr << std::endl;
```

### Spot Library Integration

If Spot compilation fails:
1. Verify Spot library path is correct
2. Check include directories: `spot/twa/twagraph.hh`, `spot/tl/parse.hh`
3. Link libraries: `-lspot -lbddx`
4. Use C++20 standard: `-std=c++20`

---

## See Also

- [Task Batch Planning Decision Tree/README.md](../Task%20Batch%20Planning%20Decision%20Tree/README.md) - How automata are used in planning
- [AUTOMATONS_README.md](../AUTOMATONS_README.md) - Detailed automaton format specifications
- [SPOT_AUTOMATA_FORMATS.md](../SPOT_AUTOMATA_FORMATS.md) - Spot library format reference

---

**Last Updated**: 2026-07-21  
**Module Status**: GBA Semantics Implementation Complete
