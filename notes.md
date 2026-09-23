# HMRTA Development Notes

## Session 1 (2026-09-16 to 2026-09-17)

### ProductAutomaton Improvements
- **Edge Label Storage**: Modified `parseProductFromDot()` to extract and store edge labels from DOT representation in Edge objects
- **Buchi Automaton Pointer**: Added `const BuchiAutomaton* buchiPtr` member variable to ProductAutomaton for runtime access
- **Capability Validation Logic**: 
  - Implemented team-based capability checking for edge feasibility
  - Changed from individual robot validation to collective team validation using element-wise OR
  - Checks if team of robots at destination collectively satisfies required capabilities
  - Handles disjunctions (OR) and conjunctions (AND) of APs correctly
  - All APs in a conjunction must be satisfied OR at least one complete conjunction satisfied

### TS Pruning Removed
- Removed pre-pruning of transition systems by robot capability
- Infeasible edges now filtered during product automaton construction instead
- Keeps full transition system space intact

### Edge_Node.h Updates
- Fixed `getEdgestoNode()` to return ALL edges to a destination ID (not just first one)
- Changed return type from reference to value to support multiple edges
- Added const correctness to method

### RandomSamplingTaskAllocation Implementation
- Created `getRandomFeasibleTaskAllocation()` method
- Generates random 50/50 robot allocation per iteration
- Validates allocation against required APs and capabilities
- Retries until feasible allocation found

### RandomNode Linked List Structure
- Converted Random_Node to singly-linked list node
- Added `next` pointer for path linking
- Created complete getter/setter interface:
  - `nodeId`: getNodeId(), setNodeId()
  - `automatonState`: getautomatonState(), setautomatonState()
  - `taskAllocation`: getTaskAllocation(), setTaskAllocation()
  - `trueAPs`: getTrueAPs(), setTrueAPs()
  - `times`: getTimes(), setTimes()
  - `curmakespan`: getCurmakespan(), setCurmakespan()
  - `next`: getNext(), setNext()
- Changed taskAllocation from `std::vector<bool>` to `std::vector<std::pair<uint16_t, bool>>`
- Added new members: `times`, `curmakespan`

### Key Technical Decisions
- Nondeterministic Büchi automata can have multiple transitions between same two states with different labels
- Random allocation strategy uses 50/50 probability per robot for task assignment
- Linked list structure enables efficient path traversal and manipulation

## Session 2 (2026-09-22) - Robot Homogeneity Test Suite

### Test Suite Architecture
- **6 Büchi Automatons**: Progressive complexity scaling from 2-3 to 12 independent capabilities
- **3 Robot Counts**: 6, 10, 16 robots per test
- **Dynamic Homogeneity Ranges**:
  - 6 robots: {0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0} (max 2.0)
  - 10 robots: {0.2, 0.4, 0.6, 0.8, 1.0, 1.2} (max 1.2)
  - 16 robots: {0.125, 0.25, 0.375, 0.5, 0.625, 0.75} (max 0.75)
- **Total Tests**: ~127 configurations (6 automatons × 3 robot counts × varying homogeneity per count)
- **Output**: 18 CSV files (6 automatons × 3 robot counts) + 28 plots (14 per Python script)

### Capability System
- **12-Element Capability Pool**: 
  - MOVEMENT_GROUND(0), MOVEMENT_AERIAL(1), MOVEMENT_AQUATIC(2)
  - SENSOR_CAMERA(3), SENSOR_LIDAR(4), SENSOR_GPS(5), SENSOR_IMU(6), SENSOR_PROXIMITY(7)
  - MANIPULATION_GRIPPER(8), MANIPULATION_TOOL(9)
  - COMMUNICATION_WIFI(10), COMMUNICATION_4G(11)
- **Homogeneity Definition**: (Independent Capabilities Distributed) / (Number of Robots), no double-counting
- **Maximum Homogeneity**: 12 / robotCount

### Buchi Automaton Capability Requirements (Sparse Distribution)
1. **Automaton 1**: 2-3 independent capabilities (p0: 2 caps, p1: 3 caps)
2. **Automaton 2**: 3-4 independent capabilities (4 APs, distributed caps 0-5)
3. **Automaton 3**: 4-6 independent capabilities (5 APs, distributed caps 0-5)
4. **Automaton 4**: 6-8 independent capabilities (7 APs, distributed caps 0-7)
5. **Automaton 5**: 8-10 independent capabilities (10 APs, sparse distribution across caps 0-9)
6. **Automaton 6**: 12 independent capabilities (10 APs, sparse distribution across all 12 caps)

**Key Design**: Each proposition uses 2-7 capabilities with overlap; no single proposition requires all capabilities needed by the formula.

### Robot Capability Distribution (Overlapping Strategy)
- **Previous Approach** (Exclusive): Each robot received sequential unique capabilities
  - Robot 1: caps {0,1}, Robot 2: caps {2,3}, Robot 3: caps {4,5}
  - Problem: Robots were not capable individually
- **New Approach** (Overlapping): Each robot gets ~75% of total independent capabilities
  - Robot 1: caps {0,1,2,3,4}, Robot 2: caps {1,2,3,4,5}, Robot 3: caps {2,3,4,5,0}
  - Benefit: Robots are individually capable while maintaining fleet homogeneity metric
  - Same total independent capabilities, improved individual robot capability

### Test Environment
- **Transition System**: 6 states (R0-R5) with star topology (all connect to R2)
- **GridWorld**: 210×210 pixels
- **Mapping**: 6 TS states mapped to grid regions:
  - R0: (180,140), R1: (180,35), R2: (120,105)
  - R3: (45,35), R4: (45,105), R5: (45,175)

### Issues Resolved

**Issue 1: Capability Pool Capping at 6 Types**
- **Problem**: Only 6 unique types available; max homogeneity capped at 6/robotCount
- **Solution**: Expanded to 12 distinct RobotCapability enum values
- **Impact**: Feasible testing ranges for all robot counts

**Issue 2: Capless Robot Problem**
- **Problem**: Distribution algorithm sometimes assigned 0 capabilities to robots
- **Root Cause**: No minimum guarantee per robot
- **Solution**: Added check `if (capCount == 0) capCount = 1`
- **Impact**: All robots now have ≥1 capability; homogeneity metric remains valid

**Issue 3: Homogeneity Calculation Bug**
- **Problem**: When homogeneity=0.625 and robots=16, showed 0.75 (12 caps instead of 10)
- **Root Cause**: Used `capIdx % capabilityPool.size()` (12) instead of `% totalCapabilities` (10)
- **Solution**: Changed modulo to cycle only within available capabilities
- **Impact**: Homogeneity values now accurately reflect independent capability count

**Issue 4: Static Homogeneity Range Infeasibility**
- **Problem**: Single range {0.2, 0.6, 1.0, 1.4, 1.8, 2.2, 2.6, 3.0} applied to all robot counts; some values impossible
- **Solution**: Dynamic map<robotCount, vector<homogeneity>> with mathematically valid ranges
- **Impact**: All test values now achievable

**Issue 5: Excessive Capability Requirements**
- **Problem**: Early designs required 4-8+ capabilities from small pool of 6 types
- **Solution**: Designed progressive requirements using 12-capability pool with sparse distribution
- **Impact**: Feasible for all homogeneity values

### Plotting & Visualization
- **plot_robot_homogeneity.py**: 14 plots combining metrics
  - 2 combined plots (all configs/automatons)
  - 6 automaton-specific computation time plots
  - 6 automaton-specific makespan plots
- **plot_average_capabilities.py**: Identical structure for average capability metrics
- **Bug Fix**: Added None-check for filtered makespans to handle astronomical values gracefully

### Data Output Structure
- **CSV Organization**: One file per (automaton_id, robot_count) pair
- **Columns**: robot_homogeneity, computation_time, makespan, memory_usage, etc.
- **Statistics**: Summary report and unified statistics export

### Key Metrics Being Collected
- Task allocation computation time
- Product automaton makespan
- Product automaton states/edges
- Memory usage (task allocation + product)
- Average capabilities per robot
- Capability homogeneity (independent caps / total robots)

### Testing Insights
- Homogeneity scaling analysis across 6 automatons with increasing formula complexity
- Product automaton growth correlation with robot count and capability requirements
- Impact of overlapping capability distribution on algorithm performance
- Feasibility thresholds per automaton as homogeneity varies
