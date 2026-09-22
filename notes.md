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
