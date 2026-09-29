# Makefile for Planning Tree Integration Tests
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3
INCLUDES = -I. -I./include -I./src -I..
LIBS = -lspot -lbddx
RAYLIB_FLAGS = $(shell pkg-config --cflags --libs raylib 2>/dev/null || echo "-lraylib -lGL -lm -lpthread -ldl -lrt -lX11")

# Output directory
OUT_DIR = Testing
OUT_BIN = $(OUT_DIR)/TestIntegration
TREE_BIN = $(OUT_DIR)/TreeBuildingTests
TASK_ALLOC_BIN = $(OUT_DIR)/TestTaskAllocation
PLANNING_TREE_BIN = $(OUT_DIR)/TestPlanningTreeComprehensive
OPTIMAL_PATH_BIN = $(OUT_DIR)/OptimalPathTest
DSTAR_BIN = $(OUT_DIR)/Test_DStarGridWorld
VIS_BIN = $(OUT_DIR)/Visualization
INTERTASK_BIN = $(OUT_DIR)/InterTaskConstraintTests
MULTICAP_BIN = $(OUT_DIR)/MultiCapabilityRobotTest
AUTOMATON_STATES_BIN = $(OUT_DIR)/AutomatonStatesTest
AUTOMATON_STATES_BATCH_BIN = $(OUT_DIR)/AutomatonStatesBatchTest
NUMBER_ROBOTS_BIN = $(OUT_DIR)/NumberRobotsTest
TS_REGIONS_BIN = $(OUT_DIR)/TransitionSystemRegionsTest
AVG_CAPS_BIN = $(OUT_DIR)/AverageCapabilitiesTest
ROBOT_HOMOGENEITY_BIN = $(OUT_DIR)/RobotHomogeneityTest
SAMPLING_TASK_ALLOC_BIN = $(OUT_DIR)/TestSamplingTaskAllocation

# Source files
TEST_SRC = apps/TestIntegration.cpp
TREE_TEST_SRC = apps/TreeBuildingTests.cpp
TASK_ALLOC_SRC = apps/TestTaskAllocationAlgorithms.cpp
PLANNING_TREE_TEST_SRC = apps/TestPlanningTreeComprehensive.cpp
OPTIMAL_PATH_SRC = apps/Optimal_Path_Test.cpp
DSTAR_SRC = apps/Test_DStarGridWorld.cpp
VIS_SRC = apps/VisualizationApp.cpp
INTERTASK_SRC = apps/Inter_task_constraint_tests.cpp
MULTICAP_SRC = apps/Multi_Capability_Robot_Test.cpp
AUTOMATON_STATES_SRC = organizedtests/automaton_states/test_automaton_states.cpp
AUTOMATON_STATES_BATCH_SRC = organizedtests/automaton_states_batch/test_automaton_states_batch.cpp
NUMBER_ROBOTS_SRC = organizedtests/number_robots/test_number_robots.cpp
TS_REGIONS_SRC = organizedtests/transition_system_regions/test_transition_system_regions.cpp
AVG_CAPS_SRC = organizedtests/average_capabilities/test_average_capabilities.cpp
ROBOT_HOMOGENEITY_SRC = organizedtests/robot_homogeneity/test_robot_homogeneity.cpp
SAMPLING_TASK_ALLOC_SRC = apps/TestingSamplingTaskAllocation.cpp
# GridVis - only included in visualization targets
GRIDVIS_SRC = src/Environment/gridvis.cpp
# TaskAllocationAlgorithms - algorithms for task allocation
TASK_ALLOC_SRCS = src/TaskAllocationAlgorithms.cpp
# RandomSamplingTaskAllocation - random sampling algorithm implementation
SAMPLING_TASK_ALLOC_IMPL_SRCS = src/RandomSamplingTaskAllocation.cpp src/RandomSamplingAlgo/RandomNode.cpp

COMMON_SRCS = \
	src/Environment/Point.cpp \
	src/Environment/GridWorld.cpp \
	src/Environment/Environment.cpp \
	src/Environment/DStarGridWorld.cpp \
	src/Tree/Tree_Node.cpp \
	src/Tree/PlanningDecisionTree.cpp \
	src/MultiRobotSystem/Robot.cpp \
	src/MultiRobotSystem/MultiRobotSystem.cpp \
	src/LTLFormula/BatchAtomicProposition.cpp \
	src/LTLFormula/LTLFormula.cpp \
	../Automatons/BuchiAutomaton.cpp \
	../Automatons/ProductAutomaton.cpp \
	../Automatons/TS.cpp \
	../Transition_Systems/GeneralTransitionSystem.cpp \
	src/AlgorithmMetrics.cpp \
	src/TestRunManager.cpp

# Object files
TEST_OBJ = $(TEST_SRC:.cpp=.o)
TREE_TEST_OBJ = $(TREE_TEST_SRC:.cpp=.o)
TASK_ALLOC_OBJ = $(TASK_ALLOC_SRC:.cpp=.o)
PLANNING_TREE_TEST_OBJ = $(PLANNING_TREE_TEST_SRC:.cpp=.o)
OPTIMAL_PATH_OBJ = $(OPTIMAL_PATH_SRC:.cpp=.o)
DSTAR_OBJ = $(DSTAR_SRC:.cpp=.o)
VIS_OBJ = $(VIS_SRC:.cpp=.o)
INTERTASK_OBJ = $(INTERTASK_SRC:.cpp=.o)
MULTICAP_OBJ = $(MULTICAP_SRC:.cpp=.o)
AUTOMATON_STATES_OBJ = $(AUTOMATON_STATES_SRC:.cpp=.o)
AUTOMATON_STATES_BATCH_OBJ = $(AUTOMATON_STATES_BATCH_SRC:.cpp=.o)
NUMBER_ROBOTS_OBJ = $(NUMBER_ROBOTS_SRC:.cpp=.o)
TS_REGIONS_OBJ = $(TS_REGIONS_SRC:.cpp=.o)
AVG_CAPS_OBJ = $(AVG_CAPS_SRC:.cpp=.o)
ROBOT_HOMOGENEITY_OBJ = $(ROBOT_HOMOGENEITY_SRC:.cpp=.o)
SAMPLING_TASK_ALLOC_OBJ = $(SAMPLING_TASK_ALLOC_SRC:.cpp=.o)
SAMPLING_TASK_ALLOC_IMPL_OBJS = $(SAMPLING_TASK_ALLOC_IMPL_SRCS:.cpp=.o)
GRIDVIS_OBJ = $(GRIDVIS_SRC:.cpp=.o)
COMMON_OBJS = $(COMMON_SRCS:.cpp=.o)
TASK_ALLOC_OBJS = $(TASK_ALLOC_SRCS:.cpp=.o)
ALL_OBJS = $(TEST_OBJ) $(TREE_TEST_OBJ) $(TASK_ALLOC_OBJ) $(PLANNING_TREE_TEST_OBJ) $(OPTIMAL_PATH_OBJ) $(DSTAR_OBJ) $(VIS_OBJ) $(INTERTASK_OBJ) $(MULTICAP_OBJ) $(AUTOMATON_STATES_OBJ) $(AUTOMATON_STATES_BATCH_OBJ) $(NUMBER_ROBOTS_OBJ) $(TS_REGIONS_OBJ) $(AVG_CAPS_OBJ) $(ROBOT_HOMOGENEITY_OBJ) $(SAMPLING_TASK_ALLOC_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(GRIDVIS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)

# Targets
.PHONY: all clean run rebuild help test test-tree test-task test-planning-tree test-optimal-path test-dstar test-intertask test-multicap test-automaton-states test-automaton-states-batch test-number-robots test-ts-regions test-average-capabilities test-robot-homogeneity test-sampling-task-alloc test-all tree-only task-only planning-tree-only optimal-path-only dstar-only intertask-only multicap-only automaton-states-only automaton-states-batch-only number-robots-only ts-regions-only average-capabilities-only robot-homogeneity-only sampling-task-alloc-only vis vis-only

all: $(OUT_BIN) $(TREE_BIN) $(TASK_ALLOC_BIN) $(PLANNING_TREE_BIN) $(OPTIMAL_PATH_BIN) $(DSTAR_BIN) $(INTERTASK_BIN) $(MULTICAP_BIN) $(AUTOMATON_STATES_BIN) $(AUTOMATON_STATES_BATCH_BIN) $(NUMBER_ROBOTS_BIN) $(TS_REGIONS_BIN) $(AVG_CAPS_BIN) $(ROBOT_HOMOGENEITY_BIN) $(SAMPLING_TASK_ALLOC_BIN) $(VIS_BIN)

# Build the test executable
$(OUT_BIN): $(TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(OUT_BIN)"

# Build the tree building tests executable
$(TREE_BIN): $(TREE_TEST_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TREE_BIN)"

# Build the task allocation tests executable
$(TASK_ALLOC_BIN): $(TASK_ALLOC_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TASK_ALLOC_BIN)"

# Build the planning tree comprehensive tests executable
$(PLANNING_TREE_BIN): $(PLANNING_TREE_TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(PLANNING_TREE_BIN)"

# Build the optimal path tests executable
$(OPTIMAL_PATH_BIN): $(OPTIMAL_PATH_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS) $(GRIDVIS_OBJ)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS) $(RAYLIB_FLAGS)
	@echo "✓ Build successful: $(OPTIMAL_PATH_BIN)"

# Build the D* GridWorld tests executable
$(DSTAR_BIN): $(DSTAR_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(DSTAR_BIN)"

# Build the inter-task constraint tests executable
$(INTERTASK_BIN): $(INTERTASK_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS) $(GRIDVIS_OBJ)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS) $(RAYLIB_FLAGS)
	@echo "✓ Build successful: $(INTERTASK_BIN)"

# Build the multi-capability robot tests executable
$(MULTICAP_BIN): $(MULTICAP_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS) $(GRIDVIS_OBJ)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS) $(RAYLIB_FLAGS)
	@echo "✓ Build successful: $(MULTICAP_BIN)"

# Build the automaton states scaling tests executable
$(AUTOMATON_STATES_BIN): $(AUTOMATON_STATES_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AUTOMATON_STATES_BIN)"

# Build the automaton states batch scaling tests executable
$(AUTOMATON_STATES_BATCH_BIN): $(AUTOMATON_STATES_BATCH_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AUTOMATON_STATES_BATCH_BIN)"

# Build the number robots scaling tests executable
$(NUMBER_ROBOTS_BIN): $(NUMBER_ROBOTS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(NUMBER_ROBOTS_BIN)"

# Build the transition system regions scaling tests executable
$(TS_REGIONS_BIN): $(TS_REGIONS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TS_REGIONS_BIN)"

# Build the average capabilities scaling tests executable
$(AVG_CAPS_BIN): $(AVG_CAPS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AVG_CAPS_BIN)"

# Build the robot homogeneity scaling tests executable
$(ROBOT_HOMOGENEITY_BIN): $(ROBOT_HOMOGENEITY_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(ROBOT_HOMOGENEITY_BIN)"

# Build the sampling task allocation tests executable
$(SAMPLING_TASK_ALLOC_BIN): $(SAMPLING_TASK_ALLOC_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(SAMPLING_TASK_ALLOC_BIN)"

# Build the visualization executable
$(VIS_BIN): $(VIS_OBJ) $(GRIDVIS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS) $(RAYLIB_FLAGS)
	@echo "✓ Build successful: $(VIS_BIN)"

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Run the test
run: $(OUT_BIN)
	@echo "Running integration tests..."
	@$(OUT_BIN)

# Run tree building tests
test-tree: $(TASK_ALLOC_BIN)
	@echo "Running task allocation tests..."
	@$(TASK_ALLOC_BIN)

# Run task allocation tests
test-task: $(TREE_BIN)
	@echo "Running tree building tests..."
	@$(TREE_BIN)

# Run planning tree comprehensive tests
test-planning-tree: $(PLANNING_TREE_BIN)
	@echo "Running planning tree comprehensive tests..."
	@$(PLANNING_TREE_BIN)

# Run optimal path tests
test-optimal-path: $(OPTIMAL_PATH_BIN)
	@echo "Running optimal path tests..."
	@$(OPTIMAL_PATH_BIN)

# Run D* GridWorld tests
test-dstar: $(DSTAR_BIN)
	@echo "Running D* GridWorld tests..."
	@$(DSTAR_BIN)

# Run inter-task constraint tests
test-intertask: $(INTERTASK_BIN)
	@echo "Running inter-task constraint tests..."
	@$(INTERTASK_BIN)

# Run multi-capability robot tests
test-multicap: $(MULTICAP_BIN)
	@echo "Running multi-capability robot tests..."
	@$(MULTICAP_BIN)

# Run automaton states scaling tests
test-automaton-states: $(AUTOMATON_STATES_BIN)
	@echo "Running automaton states scaling tests..."
	@cd organizedtests/automaton_states && ../../$(AUTOMATON_STATES_BIN)

# Run automaton states batch scaling tests
test-automaton-states-batch: $(AUTOMATON_STATES_BATCH_BIN)
	@echo "Running automaton states batch scaling tests..."
	@cd organizedtests/automaton_states_batch && ../../$(AUTOMATON_STATES_BATCH_BIN)

# Run number robots scaling tests
test-number-robots: $(NUMBER_ROBOTS_BIN)
	@echo "Running number robots scaling tests..."
	@cd organizedtests/number_robots && ../../$(NUMBER_ROBOTS_BIN)

# Run transition system regions scaling tests
test-ts-regions: $(TS_REGIONS_BIN)
	@echo "Running transition system regions scaling tests..."
	@cd organizedtests/transition_system_regions && ../../$(TS_REGIONS_BIN)

# Run average capabilities scaling tests
test-average-capabilities: $(AVG_CAPS_BIN)
	@echo "Running average capabilities scaling tests..."
	@cd organizedtests/average_capabilities && ../../$(AVG_CAPS_BIN)

# Run robot homogeneity scaling tests
test-robot-homogeneity: $(ROBOT_HOMOGENEITY_BIN)
	@echo "Running robot homogeneity scaling tests..."
	@cd organizedtests/robot_homogeneity && ../../$(ROBOT_HOMOGENEITY_BIN)

# Run sampling task allocation tests
test-sampling-task-alloc: $(SAMPLING_TASK_ALLOC_BIN)
	@echo "Running sampling task allocation tests..."
	@$(SAMPLING_TASK_ALLOC_BIN)

# Shortcut to run only tree tests
tree-only: test-tree

# Shortcut to run only task allocation tests
task-only: test-task

# Shortcut to run only planning tree tests
planning-tree-only: test-planning-tree

# Shortcut to run only optimal path tests
optimal-path-only: test-optimal-path

# Shortcut to run only D* tests
dstar-only: test-dstar

# Shortcut to run only inter-task constraint tests
intertask-only: test-intertask

# Shortcut to run only multi-capability robot tests
multicap-only: test-multicap

# Shortcut to run only automaton states tests
automaton-states-only: test-automaton-states

# Shortcut to run only automaton states batch tests
automaton-states-batch-only: test-automaton-states-batch

# Shortcut to run only number robots tests
number-robots-only: test-number-robots

# Shortcut to run only transition system regions tests
ts-regions-only: test-ts-regions

# Shortcut to run only average capabilities tests
average-capabilities-only: test-average-capabilities

# Shortcut to run only robot homogeneity tests
robot-homogeneity-only: test-robot-homogeneity

# Shortcut to run only sampling task allocation tests
sampling-task-alloc-only: test-sampling-task-alloc

# Run visualization
vis: $(VIS_BIN)
	@echo "Running visualization..."
	@$(VIS_BIN)

# Shortcut to build only visualization
vis-only: $(VIS_BIN)
	@echo "✓ Visualization built. Run 'make vis' to execute."

# Run all tests
test-all: test-tree test-task test-planning-tree test-optimal-path test-dstar test-intertask test-multicap test-automaton-states test-automaton-states-batch test-number-robots test-ts-regions test-average-capabilities test-robot-homogeneity

# Run tests and show output
test: test-all

# Rebuild from scratch
rebuild: clean all
	@echo "✓ Rebuild complete"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -f $(ALL_OBJS)
	@rm -f $(OUT_BIN)
	@echo "✓ Cleaned"

# Help target
help:
	@echo "Planning Tree Integration Test Makefile"
	@echo "========================================"
	@echo "Targets:"
	@echo "  make all                 - Build all test executables (default)"
	@echo "  make run                 - Build and run the integration test"
	@echo "  make test-tree           - Build and run the tree building tests"
	@echo "  make test-task           - Build and run the task allocation tests"
	@echo "  make test-planning-tree  - Build and run planning tree tests"
	@echo "  make test-optimal-path   - Build and run optimal path visualization tests"
	@echo "  make test-dstar          - Build and run D* GridWorld algorithm tests"
	@echo "  make test-intertask      - Build and run inter-task constraint tests"
	@echo "  make test-automaton-states - Build and run automaton states scaling tests"
	@echo "  make test-number-robots  - Build and run number robots scaling tests"
	@echo "  make vis                 - Build and run visualization app (requires raylib)"
	@echo "  make vis-only            - Build only visualization executable"
	@echo "  make tree-only           - Shortcut for 'make test-tree'"
	@echo "  make task-only           - Shortcut for 'make test-task'"
	@echo "  make planning-tree-only  - Shortcut for 'make test-planning-tree'"
	@echo "  make optimal-path-only   - Shortcut for 'make test-optimal-path'"
	@echo "  make intertask-only      - Shortcut for 'make test-intertask'"
	@echo "  make automaton-states-only - Shortcut for 'make test-automaton-states'"
	@echo "  make number-robots-only  - Shortcut for 'make test-number-robots'"
	@echo "  make dstar-only          - Shortcut for 'make test-dstar'"
	@echo "  make test                - Run all tests"
	@echo "  make test-all            - Run all tests (explicit)"
	@echo "  make rebuild             - Clean and rebuild all"
	@echo "  make clean               - Remove build artifacts"
	@echo "  make help                - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make                        # Build all tests"
	@echo "  make run                    # Build and run integration test"
	@echo "  make test-tree              # Build and run tree tests"
	@echo "  make vis                    # Build and run visualization (D* pathfinding)"
	@echo "  make test-optimal-path      # Build and run optimal path tests"
	@echo "  make test                   # Build and run all tests"
	@echo "  make clean && make          # Rebuild from scratch"
