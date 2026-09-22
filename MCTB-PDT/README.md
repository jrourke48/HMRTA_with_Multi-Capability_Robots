# Task Batch Planning Decision Tree Module

## TO-DO
### Before Tuesday:
1. Work on a more comprehensive, detailed README
### Before School:
3. Build the product automaton: done
4. Finish introduction and literature review
5. Add Tests for the Multi-Capability robots: done
6. Add Tests for the exclusive and complementary task algorithms: done
### Fall: 
7. Test against the product automaton
8. Test test test
9. Rest of writing
### Reach Goals:
implement a random sampling task allocation
10. Test Against a MILP Formulation
11. Weighing other costs than time
### Algorithm Innefficiencies
1. Could parse the edge labels during buchi construction
instead of over and over in the algorithm loop
2. 


## Overview

This module implements the core task allocation algorithms for the thesis project for multi-robot systems using a hierarchical decision tree search. The main algorithm named the IITRTS (Intensive Inter-Task Relationship Tree Search) takes 3 inputs to produce the Planning Decision Tree ([Tree/](../Tree/README.md)) that is then searched the optimal path from root to frontier tree node is the completed robot allocation for the specified task. The three inputs to the main algorithm are the buchi automatons (NBA)[Automatons](../../Automatons/README.md), environment (Env)[Environment](../Environment/README.md), and Multi-robot system (MRS)[MultiRobotSystem](../MultiRobotSystem/README.md). The environment is comprised of a gridworld and a Transition System (TS). The transition system is an automaton that represents a partition of the entire workspace into meaningful sections such as rooms or aisles. While the gridworld divides the workspace into cells with an arbitrary resolutuion to allow robots to navigate the workspace. Next the Multi-Robot System details the number of robots, their resepctive capabilities, and their positions in the environment. Lastly, the NBA is an automaton that describes the specified task for the robots. The task is specefied as an LTL formula [LTLFormula](../LTLFormula/README.md) comprised of Batch Atomic Propostions and Temportal and logic operators that is then converted to a buchi automaton using the SPOT library.    These three inputs are then used by the main algorithm, IITRTS, to build the Tree, which is a subset of the entire product automaton. Then once the tree is built we do a tree search to find the optimal frontier node.

### Key Responsibilities

1. **Task Allocation Algorithms** - Search strategies for assigning tasks to robots
2. **Decision Tree Structure** - Hierarchical representation of task allocation possibilities ([Tree/](../Tree/README.md))
3. **Planning Space Management** - State, progress, and acceptance tracking during search
4. **Pruning Strategies** - Optimization to prevent state space explosion
5. **Integration** - Coordination between [Environment](../Environment/README.md), [Automatons](../../Automatons/README.md), and [MultiRobotSystem](../MultiRobotSystem/README.md)

## Architecture
### System Architecture

```
┌─────────────────────────────────────────────────────────┐
│          LTL Task Specifications (Formula)              │
└─────────────────┬───────────────────────────────────────┘
                  │ (Parse & Convert)
                  ▼
┌─────────────────────────────────────────────────────────┐
│     Generalized Büchi Automata (GBA) Generation         |
│              (via Spot Library)                         │
└─────────────────┬───────────────────────────────────────┘
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
┌───────────────┐  ┌──────────────────┐
│ Transition    │  │ Büchi Automaton  │
│ System (TS)   │  │ States/Edges     │
└───────┬───────┘  └────────┬─────────┘
        │                   │
        └─────────┬─────────┘
                  ▼
┌─────────────────────────────────────────────────────────┐
│          Product Automaton (TS × GBA)                   │
│       Combined State Space with Acceptance Marks        │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│      Planning Decision Tree (Task Allocation)           │
│     Hierarchical Search with Pruning Strategies         │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│    Allocated Task Sequences for Robot Team             │
│         (Paths satisfying LTL specifications)           │
└─────────────────────────────────────────────────────────┘
```

### Core Components

#### 1. **PlanningDecisionTree** (`PlanningDecisionTree.h/cpp`)
The main tree structure class that manages the hierarchical decision tree.

**Key Responsibilities:**
- Maintain tree root and node count
- Construct tree by inserting nodes with associated automaton and transition system states
- Manage tree lifecycle (creation, modification, cleanup)

**Key Methods:**
- `insertNode()` - Add a new node to the tree
- `deleteSubtree()` - Remove a subtree rooted at a given node
- `getRoot()` - Access the tree root
- `getNodeCount()` - Query total nodes in tree

#### 2. **Tree_Node** (`Tree_Node.h`)
Represents individual nodes within the planning tree.

**Node State Information:**
- **Automaton State**: Current state in the Büchi automaton (NBA)
- **Transition System State**: Current state in the transition system (TS)
- **Task Allocation**: Boolean vector indicating which robot is allocated to which task
- **Time Estimates**: Estimated completion times for each robot
- **Batch Number**: Groups related tasks
- **Progress Type**: Tracks position in task execution cycle (PRE, TRA, SUF, OTH)

**Progress Enum:**
```cpp
enum class TASK_PROGRESS {
    PRE,  // Pre-condition phase
    TRA,  // Transition phase
    SUF,  // Suffix phase
    OTH   // Other/unspecified
};
```

#### 3. **TaskAllocationAlgorithms** (`TaskAllocationAlgorithms.h/cpp`)
Core algorithms for building planning trees and determining optimal paths.

**System Components:**
- Büchi automaton for LTL specification validation
- Environment representation (transition system + grid world)
- Multi-robot system for coordination

**Key Methods:**
- `buildPlanningTree()` - Construct the complete planning decision tree
- `determineBestPath()` - Select optimal execution path through tree
- Component getters/setters for NBA, Environment, and MultiRobotSystem

#### 4. **Environment** (`Environment.h/cpp`)
Encapsulates the operational environment combining a transition system with a grid world.

**Responsibilities:**
- Bridge between discrete transition systems and continuous grid environments
- State-grid coordinate mapping
- Successor state queries
- Obstacle/free space queries

**Key Methods:**
- `getTransitionSystem()` / `setTransitionSystem()` - Access to discrete TS
- `getGridWorld()` / `setGridWorld()` - Access to 2D grid representation
- `gridToStateId()` - Convert 2D grid point to state ID
- `stateIdToGrid()` - Convert state ID to 2D grid point
- `isObstacle()` / `isFree()` - Query environment properties

#### 5. **GridWorld** (`GridWorld.h/cpp`)
2D grid representation of the physical environment with costmap.

**Grid Properties:**
- **Costmap**: 2D array where:
  - 0 = free space
  - 1-254 = movement cost
  - 255 = obstacle

**Key Methods:**
- `getCost()` / `setCost()` - Query/set cell costs
- `isObstacle()` / `isFree()` - Check cell properties
- `isInBounds()` - Validate coordinates
- `setObstacle()` / `clearCell()` - Modify environment

#### 6. **Point** (`Point.h/cpp`)
Simple 2D coordinate representation for grid-based navigation.

**Operations:**
- Coordinate access (x, y)
- Vector arithmetic (addition, subtraction)
- Distance calculations
- Comparison operators

## File Structure

```
Task Batch Planning Decision Tree/
├── PlanningDecisionTree.h          # Main tree structure
├── PlanningDecisionTree.cpp        # Tree implementation
├── Tree_Node.h                      # Individual node definition
├── TaskAllocationAlgorithms.h       # Core algorithms
├── TaskAllocationAlgorithms.cpp     # Algorithm implementation
├── Environment.h                    # Environment abstraction
├── Environment.cpp                  # Environment implementation
├── GridWorld.h                      # 2D grid representation
├── GridWorld.cpp                    # Grid implementation
├── Point.h                          # 2D point utility
├── Point.cpp                        # Point implementation
├── [Environment/](../Environment/README.md)  # Grid and environment management
├── [LTLFormula/](../LTLFormula/README.md)    # LTL parsing and atomic propositions
├── [MultiRobotSystem/](../MultiRobotSystem/README.md)  # Multi-robot coordination
├── [Tree/](../Tree/README.md)                # Planning tree node structures
├── [Automatons/](../../Automatons/README.md) # Büchi automata (parent directory)
└── README.md                        # This file
```

## Key Features

### 1. Hierarchical Planning
- Organize complex task allocations in tree structures
- Support branching strategies for different robot configurations
- Track progress through multi-phase task execution (PRE, TRA, SUF)

### 2. Multi-Robot Coordination
- Allocate tasks to multiple robots simultaneously
- Track individual robot completion times
- Batch related tasks for coherent execution

### 3. Formal Methods Integration
- Integrate with Büchi automata for LTL specifications
- Connect to transition systems for state-based planning
- Ensure formal correctness of task sequences

### 4. Environment Awareness
- Navigate 2D grid environments with obstacles and costs
- Map between discrete transition system states and continuous grid coordinates
- Support cost-aware planning decisions

## LTL Formula Syntax & Operator Precedence

### Overview
Linear Temporal Logic (LTL) formulas describe the behavior and requirements of the robotic system. The SPOT library converts these high-level specifications into Büchi automata for formal verification and planning.

### Basic Operators (in order of precedence, highest to lowest)

| Operator | Name | Example |
|----------|------|---------|
| `!` | NOT (negation) | `!p` - NOT p |
| `&` | AND | `p & q` - p AND q |
| `\|` | OR | `p \| q` - p OR q |
| `^` | XOR | `p ^ q` - p XOR q |
| `i` | IMPLIES | `p i q` - p IMPLIES q |
| `e` | EQUIVALENT | `p e q` - p EQUIVALENT q |

### Temporal Operators (always lower precedence than logical operators)

| Operator | Name | Meaning |
|----------|------|---------|
| `F` | Eventually | `F p` - p becomes true eventually |
| `G` | Globally/Always | `G p` - p is always true |
| `X` | Next | `X p` - p is true in the next state |
| `U` | Until (strong) | `p U q` - p holds until q becomes true |
| `V` | Release (weak) | `p V q` - p holds unless q becomes true |
| `W` | Weak Until | `p W q` - p holds until q (weak version) |
| `M` | Strong Release | `p M q` - strong version of release |

### Operator Precedence Rules

1. **Logical operators bind tighter than temporal operators**
   - `F(a & b)` ≠ `(F a) & (F b)`
   - `F(a & b)` means "eventually both a AND b together"
   - `(F a) & (F b)` means "eventually a" AND "eventually b" (can be at different times)

2. **Use parentheses for clarity**
   - Always group temporal operators with operands: `G(F p)` not `G F p`
   - Helps override default precedence and improves readability

3. **Key patterns**
   - `G(F p)` - Infinitely often p (infinite repetition, creates cycles)
   - `F p & F q` - Both p and q eventually (finite, one-time satisfaction)
   - `G(p i F q)` - Whenever p, eventually q (reactive specification)

### Examples from Test Suite

**FINITE AUTOMATA** (No G operator - eventuality, can terminate):

```ltl
(F"p0" & F"p1")
  → Eventually visit p0, and eventually visit p1
  → Once both satisfied, mission completes
  
(F"p0" & F"p1" & F"p2" & F"p3")
  → Visit four locations in any order
  → All must eventually occur, then execution ends
```

**INFINITE AUTOMATA** (Has G operator - liveness, cannot terminate):

```ltl
G(F"p0")
  → Infinitely often visit p0
  → Must repeatedly return to location p0
  → Creates cycles, cannot terminate
  
G((F"p0") & (F"p1") & (F"p2"))
  → Always eventually p0, p1, and p2
  → All three locations visited infinitely often
  → Patrol pattern that repeats forever
```

### Finite vs. Infinite Automata

| Pattern | Type | Behavior | Use Case |
|---------|------|----------|----------|
| `F p & F q & ...` | Finite | Tasks complete when all conditions met | One-time missions |
| `G(F p)` | Infinite | Infinite repetition of condition | Surveillance, patrol |
| `!(F X p)` | Finite | Negated eventuality | Avoidance tasks |
| `G !(obstacle)` | Infinite | Always avoid obstacle | Safety constraints |

### Quick Reference: Identifying Automaton Type

- **No G operator** → FINITE automaton (unless negated F)
- **Has G operator** → INFINITE automaton (liveness requirement)
- **Uncertain?** → Use `isFinite()` method for precise DFS cycle detection

### Syntax Rules

- Use double quotes for atomic propositions: `"p0"`, `"p1"`, etc.
- Use `&` not `&&` for conjunction
- Use `|` not `||` for disjunction
- Use `!` prefix for negation: `!"p"` not `p!`
- Use `i` for implication, not `→` or `->`
- Use `e` for equivalence, not `<->`

## Usage Example

```cpp
// 1. Create environment
GeneralTransitionSystem* ts = new GeneralTransitionSystem(...);
GridWorld* grid = new GridWorld(100, 100);
Environment* env = new Environment(ts, grid);

// 2. Initialize multi-robot system
MultiRobotSystem* robots = new MultiRobotSystem(3); // 3 robots

// 3. Create NBA from LTL formula
BuchiAutomaton* nba = new BuchiAutomaton(...);

// 4. Setup task allocation algorithms
TaskAllocationAlgorithms* allocator = new TaskAllocationAlgorithms(nba, env, robots);

// 5. Build planning tree
std::vector<bool> initialAllocation = {true, false, true};
std::vector<uint16_t> times = {10, 15, 12};
PlanningDecisionTree* planTree = allocator->buildPlanningTree(
    0,                                  // root ID
    nba->getInitialState(),            // automaton state
    ts->getInitialState(),             // TS state
    initialAllocation,                 // task allocation
    times,                             // estimated times
    1,                                 // batch number
    Tree_Node::TASK_PROGRESS::PRE      // progress type
);

// 6. Determine best execution path
Tree_Node* bestPath = allocator->determineBestPath(planTree);

// 7. Cleanup
delete planTree;
delete allocator;
delete robots;
delete nba;
delete env;
```

## Integration Points

### With Automatons Module ([Automatons/](../../Automatons/README.md))
- Uses `Node` from `Automatons/` for automaton state representation
- Works with `BuchiAutomaton` for LTL specification validation
- Tracks progress through automaton states during planning

### With LTL Formula Module ([LTLFormula/](../LTLFormula/README.md))
- Parses LTL specifications into atomic propositions
- Manages batch atomic propositions for task representation
- Converts formulas to Büchi automata via SPOT library

### With Environment Module ([Environment/](../Environment/README.md))
- Integrates `GeneralTransitionSystem` for discrete state space
- Maps between TS state IDs and grid coordinates
- Queries successor states for planning exploration

### With Multi-Robot System ([MultiRobotSystem/](../MultiRobotSystem/README.md))
- Coordinates task allocation across multiple robots
- Tracks individual robot times and capabilities
- Manages robot-task relationships in allocation vectors

### With Tree Module ([Tree/](../Tree/README.md))
- Manages planning tree node structures and relationships
- Stores task allocation decisions at each tree node
- Tracks progress types (PRE, TRA, SUF, OTH) through task phases

## Data Flow

```
LTL Formula
    ↓
[Büchi Automaton] ←── Task Allocation Algorithms ←── [Environment]
    ↓                          ↓                          ↓
Automaton States      Planning Decision Tree      [Transition System]
                              ↓                          ↓
                          Tree_Node                  [GridWorld]
                          (decision points)              ↓
                              ↓                      [Point coordinates]
                        Task Allocation
                        (robot assignments)
```

## Compilation

### With CMake
```bash
cmake -B build
cmake --build build
```

### With g++
```bash
g++ -std=c++20 -I. -L. PlanningDecisionTree.cpp Tree_Node.cpp \
    TaskAllocationAlgorithms.cpp Environment.cpp GridWorld.cpp Point.cpp \
    -o planning_tree -lspot -lbddx
```

## Future Extensions

- **Heuristic Pruning**: Implement A* style heuristics to reduce tree size
- **Dynamic Replanning**: Support re-planning when environment changes
- **Parallelization**: Parallel tree construction for large state spaces
- **Visualization**: Export tree structure for analysis and debugging
- **Optimization**: Minimize makespan and resource usage metrics

## Notes for Developers

1. **Memory Management**: The tree owns Tree_Node pointers. Call `deleteSubtree()` or `clearTree()` to free memory properly.
2. **State Consistency**: Ensure automaton and TS states remain synchronized throughout tree construction.
3. **Batch Semantics**: Use batch numbers to group related task phases (PRE, TRA, SUF) for coherent execution.
4. **Thread Safety**: Current implementation is single-threaded. Add synchronization if using in multi-threaded contexts.

## References

- Related to hierarchical task allocation for multi-robot systems
- Integrates with LTL formal specifications via Büchi automata
- Part of the HMRTA test suite generation framework

---

# AlgorithmMetrics Class Documentation

## Overview

The `AlgorithmMetrics` class provides comprehensive tracking and reporting of all thesis evaluation metrics across three evaluation dimensions:

1. **Correctness & Feasibility** - LTL satisfaction, capability satisfaction, constraint violations
2. **Computational Efficiency** - Runtime analysis, scalability, node generation & pruning
3. **Solution Quality** - Mission completion time, task distribution, resource utilization

## Independent Variables

Variables that characterize the problem instance and environment:

| Variable | Type | Meaning |
|----------|------|---------|
| `num_automaton_states` | int | Number of states in Büchi automaton $\|S_{\mathcal{B}}\|$ |
| `num_automaton_edges` | int | Number of transitions in automaton |
| `num_atomic_propositions` | int | Number of atomic propositions in LTL formula |
| `num_robots` | int | Team size $N$ |
| `total_robot_capabilities` | int | Sum of all capabilities across fleet |
| `num_ts_regions` | int | Number of regions in transition system |
| `avg_capabilities_per_robot` | double | Average capabilities per robot |
| `capability_homogeneity` | double | Measure of fleet homogeneity |
| `num_inter_task_constraints` | int | Number of task inter-dependencies |

## Metrics Hierarchy

### 1. Runtime Metrics

Tracks computation time and scalability analysis:

```cpp
struct RuntimeMetrics {
    double total_computation_time_ms;
    double high_level_runtime_ms;
    double low_level_runtime_ms;
    
    // Parametric analysis (collected across runs)
    std::vector<std::pair<int, double>> runtime_vs_num_robots;
    std::vector<std::pair<int, double>> runtime_vs_automaton_states;
    std::vector<std::pair<int, double>> runtime_vs_automaton_edges;
    std::vector<std::pair<double, double>> runtime_vs_capability_density;
    // ... more parametric measurements
};
```

**Key measurements:**
- `runtime_vs_num_robots`: Scale test with N=5, 10, 20, 50, 100 robots
- `runtime_vs_automaton_states`: Scale test with |S_B|=4, 8, 16, 32, 64, 128
- `runtime_vs_automaton_edges`: Transition-density scaling
- `runtime_vs_capability_density`: Capability coverage in fleet
- `runtime_vs_capability_homogeneity`: Fleet homogeneity impact

### 2. Subtree Efficiency Metrics

Compares tree-based search against full product automaton:

```cpp
struct SubtreeEfficiencyMetrics {
    // Node tracking
    long long total_nodes_generated;
    long long total_nodes_expanded;
    long long total_nodes_pruned;
    long long nodes_satisfying_ltl;            // OTH or TRA nodes
    
    // Memory comparison
    long long tree_memory_bytes;
    long long product_automaton_memory_bytes;
    long long full_product_automaton_nodes;
    
    // Derived efficiency ratios
    double pruning_ratio;                      // pruned / total
    double explored_product_ratio;             // generated / full_product
    double tree_product_ratio;                 // tree_nodes / full_product_nodes
    double memory_reduction_ratio;             // tree_memory / product_memory
    double optimality_gap_percent;
    double runtime_speedup_percent;
    long long state_space_reduction;
};
```

**Key insights:**
- **Pruning Ratio**: Higher ratio = more effective pruning strategy
- **State-Space Reduction**: Demonstrates scalability beyond full product
- **Memory Reduction Ratio**: Shows practical memory savings (< 1.0 is good)
- **Runtime Speedup %**: Quantifies tree-search advantage

### 3. Solution Quality Metrics

Evaluates the quality of the computed solution:

```cpp
struct SolutionQualityMetrics {
    // Time metrics
    double makespan_seconds;                   // Mission completion time
    double sum_of_travel_times_seconds;        // Total team effort
    double total_travel_distance;
    double max_individual_travel_distance;
    
    // Load balancing
    double load_balance_variance;
    int robots_utilized;
    double robot_utilization_ratio;
    
    // Per-robot details
    std::vector<double> individual_travel_times;
    std::vector<double> individual_travel_distances;
    std::vector<int> tasks_per_robot;
};
```

**Interpretation:**
- **Makespan**: Lower is better (parallelizable completion)
- **Sum of travel times**: Lower is better (team efficiency)
- **Load balance**: Lower variance indicates even distribution
- **Robot utilization**: % of robots actually used in solution

## Usage Examples

### Basic Usage

```cpp
#include "AlgorithmMetrics.h"

int main() {
    AlgorithmMetrics metrics;
    
    // Set independent variables
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = 10;
    vars.num_automaton_states = 16;
    vars.num_automaton_edges = 32;
    vars.num_atomic_propositions = 5;
    vars.total_robot_capabilities = 25;
    vars.num_ts_regions = 20;
    vars.num_inter_task_constraints = 3;
    
    metrics.setIndependentVariables(vars);
    
    // Record measurements
    metrics.startTimer();
    
    // ... run algorithm ...
    
    metrics.stopTimer();
    
    // Record correctness
    AlgorithmMetrics::CorrectnessMetrics correctness;
    correctness.ltl_satisfaction_rate = 1.0;
    correctness.capability_satisfaction_rate = 1.0;
    correctness.acceptance_condition_reached = true;
    correctness.feasibility_status = "FEASIBLE";
    metrics.recordCorrectness(correctness);
    
    // Compute and display results
    metrics.computeDerivedMetrics();
    metrics.printSummary();
    metrics.exportToCSV("metrics.csv");
    metrics.exportToJSON("metrics.json");
    
    return 0;
}
```

### Scalability Testing (Runtime vs Robots)

```cpp
AlgorithmMetrics metrics;

for (int num_robots : {5, 10, 20, 50, 100}) {
    // Configure problem with num_robots
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = num_robots;
    // ... set other variables ...
    metrics.setIndependentVariables(vars);
    
    // Run algorithm
    metrics.startTimer();
    runAlgorithm(/* ... */);
    metrics.stopTimer();
    
    // Record result
    metrics.addRuntimeVsRobots(num_robots, metrics.getRuntime().total_computation_time_ms);
}

metrics.printDetailedReport();
```

### Scalability Testing (Runtime vs Automaton States)

```cpp
for (int states : {4, 8, 16, 32, 64, 128}) {
    // Create Büchi automaton with 'states' states
    
    metrics.startTimer();
    runAlgorithm(/* ... */);
    metrics.stopTimer();
    
    metrics.addRuntimeVsAutomatonStates(states, 
                                       metrics.getRuntime().total_computation_time_ms);
}
```

### Subtree Efficiency Analysis

```cpp
AlgorithmMetrics::SubtreeEfficiencyMetrics efficiency;

// After tree-search execution
efficiency.total_nodes_generated = tree_nodes_count;
efficiency.total_nodes_pruned = pruned_nodes_count;
efficiency.tree_memory_bytes = calculateTreeMemory();

// Compute full product automaton (for small instances)
efficiency.full_product_automaton_nodes = buildFullProduct().nodeCount();
efficiency.product_automaton_memory_bytes = calculateProductMemory();

metrics.recordSubtreeEfficiency(efficiency);
metrics.computeDerivedMetrics();

// Now efficiency.pruning_ratio, .explored_product_ratio, etc. are computed
```

### Solution Quality Tracking

```cpp
AlgorithmMetrics::SolutionQualityMetrics quality;

quality.makespan_seconds = computeMakespan(solution);
quality.sum_of_travel_times_seconds = computeTotalTravelTime(solution);
quality.total_travel_distance = computeTotalDistance(solution);
quality.max_individual_travel_distance = computeMaxIndividualDistance(solution);

// Track per-robot metrics
for (const auto& robot : solution.robots) {
    quality.individual_travel_times.push_back(robot.total_time);
    quality.individual_travel_distances.push_back(robot.total_distance);
    quality.tasks_per_robot.push_back(robot.task_count);
    
    if (robot.task_count > 0) {
        quality.robots_utilized++;
    }
}

metrics.recordSolutionQuality(quality);
metrics.computeDerivedMetrics();
```

## Export Formats

### CSV Export
```csv
Metric,Value,Unit
Automaton States,16,count
Number of Robots,10,count
Total Computation Time,125.45,ms
LTL Satisfaction Rate,1.0000,ratio
Makespan,45.32,seconds
...
```

### JSON Export
```json
{
  "independent_variables": {
    "automaton_states": 16,
    "num_robots": 10,
    ...
  },
  "runtime_metrics": {
    "total_computation_time_ms": 125.45,
    ...
  },
  ...
}
```

## Integration with Algorithm

1. **Initialization**: Create metrics instance and set independent variables
2. **During execution**: Use startTimer/stopTimer, recordHighLevel/LowLevel for sub-components
3. **Node tracking**: Increment counters for generated/pruned/expanded nodes
4. **After execution**: Record all metric structures and call computeDerivedMetrics()
5. **Analysis**: Use printSummary(), printDetailedReport(), or export functions

## Recommended Experiment Design

### Phase 1: Correctness Verification
- Small instances (N≤5, |S_B|≤16)
- Verify against full product automaton
- Record all correctness metrics

### Phase 2: Scalability Analysis
- Fix all but one independent variable
- Vary: N, |S_B|, capability density, homogeneity
- Collect runtime and node count data
- Generate scaling plots

### Phase 3: Efficiency Comparison
- For instances where full product succeeds: compare directly
- Track pruning effectiveness
- Measure memory savings

### Phase 4: Solution Quality
- Analyze makespan, load balance, utilization
- Compare greedy vs optimal allocations (when available)

## Performance Targets

- **Pruning Ratio**: > 0.7 (70% of generated nodes pruned)
- **State-Space Reduction**: Product nodes - Tree nodes should be large
- **Runtime Speedup**: > 50% faster than full product (when comparable)
- **Robot Utilization**: Should approach 100% for well-designed instances
- **Makespan**: Within 10-20% of theoretical lower bound
