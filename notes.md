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

## Session 3 (2026-09-29) - Sampling Test Failures & NBA Pruning Bugs

Starting point: `TestSamplingTaskAllocation` at 49 passed / 12 failed, and the product
automaton reporting a worse makespan (345) than the random sampling allocator (308) on
the same 4-robot instance.

### ProductAutomaton: Suffix-Only Accepting Path
- Removed step 3 of `OptimalAcceptingPath()` (the extra "+1 node" appended after the cycle)
- Path is now just prefix + suffix: initial -> accepting state -> cycle back to it
- `finalLabel` references replaced with `secondAcceptingLabel` throughout

### ProductAutomaton: Global Optimality Experiment (REVERTED)
- Tried replacing the two-step search with one that enumerates every accepting-on-cycle
  state, finds the best cycle for each, and keeps the global minimum
- **Result**: identical makespan, dramatically slower. Reverted via `git checkout`
- **Takeaway**: the two-step search was not the source of the 345 vs 308 gap. The gap is
  more likely in which edges exist in the product at all (see Structural Item 1)

### Issues Resolved

**Issue 1: Inverted Feasibility Logic in pruneInfeasibleNBAPaths**
- **Problem**: Edges were kept/dropped on the wrong condition
- **Root Cause**: `edgeIsFeasible = true` was set when a required capability was *missing*
- **Solution**: Track `apSetFeasible` per conjunction; only mark the edge feasible when a
  full conjunction is satisfied by the team (OR of ANDs, evaluated correctly)
- **Impact**: Test 2 capability checks now pass

**Issue 2: Edges With No APs Were Being Pruned**
- **Problem**: Edges labelled `!p1`, `!p0 & !p1`, or `true` were removed from the pruned NBA
- **Root Cause**: Empty `trueAPs` meant the feasibility loop never ran, leaving the edge
  marked infeasible. Semantically these edges require *no* robots, so they are always feasible
- **Solution**: Treat empty `trueAPs` as feasible before entering the conjunction loop
- **Impact**: Negation-only self-loops (e.g. `3 -> 3[!p1]`) survive pruning

**Issue 3: Infinite Loop in getRandomFeasibleTaskAllocation (THE HANG)**
- **Problem**: Test 5 hung forever after Issue 2 was fixed
- **Root Cause**: `while (!isFeasible)` wrapped a `for` over `trueAPs`. With `trueAPs` empty
  the body never executed and `isFeasible` never flipped. Fixing Issue 2 meant the path
  builder could finally step onto a negation-only edge and trigger it
- **Solution**: Return an empty allocation immediately for an edge with no positive APs;
  replaced the unbounded `while` with a capped retry (`MAX_ALLOCATION_ATTEMPTS = 100`);
  clear `robotsByAP` on failure so a stale draw is never paired with an empty AP set
- **Impact**: Hang eliminated

**Issue 4: Allocation Read From the Unpruned NBA**
- **Problem**: "satisfied AP set is not one of the edge's AP sets" across Tests 5 and 7
- **Root Cause**: `getRandomFeasibleTaskAllocation` called `nba->getTrueAPs(...)` while
  `buildRandomPath` and `validateStep` both work on `prunedNBA`. An AP set that pruning had
  removed could be allocated for a step the pruned edge no longer permits
- **Solution**: Select `prunedNBA` when present, matching the pattern already in `getMinLengthPath`
- **Impact**: Only shows up when pruning actually removes edges, which is why the
  all-capabilities fixture passed and the no-camera / infinite fixtures did not

**Issue 5: BuchiAutomaton Had No Copy Constructor (Shallow Copy)**
- **Problem**: "original NBA edges are unchanged after pruning" failed whenever edges were
  actually removed
- **Root Cause**: `prunedNBA = new BuchiAutomaton(*nba)` used the implicit copy constructor.
  `nodeMap` is `std::map<uint16_t, Node*>`, so both automata pointed at the *same* Node
  objects and `node->setEdges(...)` mutated the original
- **Solution**: Added a copy constructor that allocates a new `Node` per entry. `Node` owns
  its `edges` vector by value, so copying the node is enough to decouple them. `ltlFormula`
  and `spotAutomaton` stay shared by design. `operator=` deleted
- **Impact**: Test 2 fully green

**Issue 6: Unset Iteration/Time Limits Read as Zero**
- **Problem**: Each constructor alone ran 0 iterations; only setting *both* limits worked
- **Root Cause**: `while (Iterations < maxIterations && computationTime < timeLimit)` in both
  planners. The iteration constructor leaves `timeLimit = 0.0`, so `computationTime < 0.0`
  was false on entry, and symmetrically for the time constructor
- **Solution**: A limit left at 0 is treated as unset, so only the limit the caller supplied bounds the loop
- **Impact**: 4 "ran 0" failures in Tests 6, 7, 9

**Issue 7: Modulo by Zero in getRandomAllocation**
- **Problem**: `std::rand() % apSet.size()` is undefined behaviour on an empty AP set
- **Root Cause**: No guard. Currently unreachable, but directly in the path of Structural Item 1
- **Solution**: Return the empty allocation before the loop when `apSet` is empty

### Structural Items (NOT FIXED - need a design decision)

**4. Rejection sampling with a 50% coin flip**
- `getRandomAllocation` assigns each robot with probability 1/2, so an AP needing all four
  robots is drawn 1-in-16
- The 100-attempt cap stops the hang but does not fix the sampler: if a conjunction is only
  satisfiable by a specific near-full assignment, the result is silently "no allocation"
  rather than the allocation that exists
- Worth replacing with a constructive assignment that covers required capabilities directly

### Open Question
- Product automaton reports makespan 345; the random sampling allocator reports 308 on the
  same instance. Structural Item 1 is the leading suspect — verify before assuming the
  path search itself is at fault (the global-optimality rewrite already ruled that out)

working
Every remaining failure is a multi-AP conjunction the sampler can't allocate. Here's why, using your fixtures:

createTestMultiRobotSystem2 gives 6 robots with exactly one capability each — 2 ground, 2 camera, 2 GPS. Fixture 1's propositions p0, p1, p2 each require capabilities {0, 3, 5} = ground + camera + GPS. So one proposition consumes three robots, one of each type, and robots can't be shared between propositions in the same conjunction.

Conjunction	Robots needed	Status
p1 (1 AP)	3	feasible
p0 & p2 (2 APs)	6 — the entire team, perfectly split	feasible, but only 1 partition in 512 draws
p0 & p1 & p2 (3 APs)	9	impossible — only 6 robots exist
Now match that against the failures: 0->0 and 1->0 are p0 & p1 & p2 (impossible). 0->5, 1->5, 1->2 are 2-AP (possible but ~18% chance of being found within the 100 retries). That accounts for all 8.

Two distinct bugs behind it
1. The feasibility test ignores that robots can't be shared. pruneInfeasibleNBAPaths checks each proposition against the pooled capabilities of the whole team, independently per proposition. So p0 & p1 & p2 looks feasible — the team does collectively have ground, camera and GPS — even though satisfying all three at once needs 9 robots. The edge is kept, then nothing downstream can allocate it.

The test's reference helper edgeIsDeterministicallyFeasible makes the same relaxation, which is why "no infeasible edge is kept" passes. Both share the blind spot, so the test suite can't catch it. Deciding conjunction feasibility properly is a bipartite matching problem, not an independent per-AP check.

2. Nothing checks whether allocation actually succeeded. When getRandomFeasibleTaskAllocation exhausts its retries it returns an empty AP set — which I added — but buildRandomPath and runFinitePathPlanner both feed that straight into a new Random_Node without looking:


auto [robotsByAP, satisfiedTrueAPs] = getRandomFeasibleTaskAllocation(curNode, nextNode);
Random_Node* newRandomNode = new Random_Node(..., satisfiedTrueAPs, robotsByAP, curtimes);
So a failed allocation becomes a path step carrying an empty AP set, and validateStep correctly reports that empty set isn't one of the edge's options. That's the literal error you're seeing. The fix is to treat a failed allocation as "this edge can't be taken" and pick a different candidate.

Bug 2 is small and unambiguous. Bug 1 is the modeling decision — and note it also affects the product automaton, which uses the same independent-per-AP capability pooling at ProductAutomaton.cpp:572. Say which you want and I'll do it.