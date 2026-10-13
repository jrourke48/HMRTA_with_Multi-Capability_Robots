# Makefile for Planning Tree Integration Tests
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3 -MMD -MP
INCLUDES = -I. -I./include -I./src -I..
LIBS = -lspot -lbddx
RAYLIB_FLAGS = $(shell pkg-config --cflags --libs raylib 2>/dev/null || echo "-lraylib -lGL -lm -lpthread -ldl -lrt -lX11")

# Output directory
OUT_DIR = Testing
# Trials to repeat each sampler budget in the budget study.
# BUDGET_APPEND=--append adds to the existing CSVs instead of replacing them.
BUDGET_TRIALS ?= 1
BUDGET_APPEND ?=
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
# Sampler budget study: one binary per sweep (their config factories collide)
BUDGET_COMMON_SRCS = budget_study/budget_common.cpp
BS_AUTOMATON_STATES_BIN = $(OUT_DIR)/BudgetAutomatonStates
BS_AUTOMATON_STATES_SRCS = budget_study/budget_automaton_states.cpp budget_study/configs_automaton_states.cpp
BS_NUMBER_ROBOTS_BIN = $(OUT_DIR)/BudgetNumberRobots
BS_NUMBER_ROBOTS_SRCS = budget_study/budget_number_robots.cpp budget_study/configs_number_robots.cpp
BS_TS_REGIONS_BIN = $(OUT_DIR)/BudgetTransitionSystemRegions
BS_TS_REGIONS_SRCS = budget_study/budget_transition_system_regions.cpp budget_study/configs_transition_system_regions.cpp
BS_AVG_CAPS_BIN = $(OUT_DIR)/BudgetAverageCapabilities
BS_AVG_CAPS_SRCS = budget_study/budget_average_capabilities.cpp budget_study/configs_average_capabilities.cpp
BS_ROBOT_HOMOGENEITY_BIN = $(OUT_DIR)/BudgetRobotHomogeneity
BS_ROBOT_HOMOGENEITY_SRCS = budget_study/budget_robot_homogeneity.cpp budget_study/configs_robot_homogeneity.cpp
# Showcase replay: re-plans the configurations one planner clearly won and draws both plans.
# Same one-binary-per-sweep split, and the same config factories, as the budget study above.
REPLAY_COMMON_SRCS = budget_study/replay_common.cpp
RP_AUTOMATON_STATES_BIN = $(OUT_DIR)/ReplayAutomatonStates
RP_AUTOMATON_STATES_SRCS = budget_study/replay_automaton_states.cpp budget_study/configs_automaton_states.cpp
RP_NUMBER_ROBOTS_BIN = $(OUT_DIR)/ReplayNumberRobots
RP_NUMBER_ROBOTS_SRCS = budget_study/replay_number_robots.cpp budget_study/configs_number_robots.cpp
RP_TS_REGIONS_BIN = $(OUT_DIR)/ReplayTransitionSystemRegions
RP_TS_REGIONS_SRCS = budget_study/replay_transition_system_regions.cpp budget_study/configs_transition_system_regions.cpp
RP_AVG_CAPS_BIN = $(OUT_DIR)/ReplayAverageCapabilities
RP_AVG_CAPS_SRCS = budget_study/replay_average_capabilities.cpp budget_study/configs_average_capabilities.cpp
RP_ROBOT_HOMOGENEITY_BIN = $(OUT_DIR)/ReplayRobotHomogeneity
RP_ROBOT_HOMOGENEITY_SRCS = budget_study/replay_robot_homogeneity.cpp budget_study/configs_robot_homogeneity.cpp

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
BUDGET_COMMON_OBJS = $(BUDGET_COMMON_SRCS:.cpp=.o)
BS_AUTOMATON_STATES_OBJS = $(BS_AUTOMATON_STATES_SRCS:.cpp=.o)
BS_NUMBER_ROBOTS_OBJS = $(BS_NUMBER_ROBOTS_SRCS:.cpp=.o)
BS_TS_REGIONS_OBJS = $(BS_TS_REGIONS_SRCS:.cpp=.o)
BS_AVG_CAPS_OBJS = $(BS_AVG_CAPS_SRCS:.cpp=.o)
BS_ROBOT_HOMOGENEITY_OBJS = $(BS_ROBOT_HOMOGENEITY_SRCS:.cpp=.o)
REPLAY_COMMON_OBJS = $(REPLAY_COMMON_SRCS:.cpp=.o)
RP_AUTOMATON_STATES_OBJS = $(RP_AUTOMATON_STATES_SRCS:.cpp=.o)
RP_NUMBER_ROBOTS_OBJS = $(RP_NUMBER_ROBOTS_SRCS:.cpp=.o)
RP_TS_REGIONS_OBJS = $(RP_TS_REGIONS_SRCS:.cpp=.o)
RP_AVG_CAPS_OBJS = $(RP_AVG_CAPS_SRCS:.cpp=.o)
RP_ROBOT_HOMOGENEITY_OBJS = $(RP_ROBOT_HOMOGENEITY_SRCS:.cpp=.o)
GRIDVIS_OBJ = $(GRIDVIS_SRC:.cpp=.o)
COMMON_OBJS = $(COMMON_SRCS:.cpp=.o)
TASK_ALLOC_OBJS = $(TASK_ALLOC_SRCS:.cpp=.o)
ALL_OBJS = $(TEST_OBJ) $(TREE_TEST_OBJ) $(TASK_ALLOC_OBJ) $(PLANNING_TREE_TEST_OBJ) $(OPTIMAL_PATH_OBJ) $(DSTAR_OBJ) $(VIS_OBJ) $(INTERTASK_OBJ) $(MULTICAP_OBJ) $(AUTOMATON_STATES_OBJ) $(AUTOMATON_STATES_BATCH_OBJ) $(NUMBER_ROBOTS_OBJ) $(TS_REGIONS_OBJ) $(AVG_CAPS_OBJ) $(ROBOT_HOMOGENEITY_OBJ) $(SAMPLING_TASK_ALLOC_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(BUDGET_COMMON_OBJS) $(BS_AUTOMATON_STATES_OBJS) $(BS_NUMBER_ROBOTS_OBJS) $(BS_TS_REGIONS_OBJS) $(BS_AVG_CAPS_OBJS) $(BS_ROBOT_HOMOGENEITY_OBJS) $(REPLAY_COMMON_OBJS) $(RP_AUTOMATON_STATES_OBJS) $(RP_NUMBER_ROBOTS_OBJS) $(RP_TS_REGIONS_OBJS) $(RP_AVG_CAPS_OBJS) $(RP_ROBOT_HOMOGENEITY_OBJS) $(GRIDVIS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)

# Targets
.PHONY: all clean run rebuild help test compare test-tree test-task test-planning-tree test-optimal-path test-dstar test-intertask test-multicap test-automaton-states test-automaton-states-batch test-number-robots test-ts-regions test-average-capabilities test-robot-homogeneity test-sampling-task-alloc test-all tree-only task-only planning-tree-only optimal-path-only dstar-only intertask-only multicap-only automaton-states-only automaton-states-batch-only number-robots-only ts-regions-only average-capabilities-only robot-homogeneity-only sampling-task-alloc-only vis vis-only budget-automaton-states budget-number-robots budget-ts-regions budget-average-capabilities budget-robot-homogeneity budget-study replay-automaton-states replay-number-robots replay-ts-regions replay-average-capabilities replay-robot-homogeneity showcase

all: $(OUT_BIN) $(TREE_BIN) $(TASK_ALLOC_BIN) $(PLANNING_TREE_BIN) $(OPTIMAL_PATH_BIN) $(DSTAR_BIN) $(INTERTASK_BIN) $(MULTICAP_BIN) $(AUTOMATON_STATES_BIN) $(AUTOMATON_STATES_BATCH_BIN) $(NUMBER_ROBOTS_BIN) $(TS_REGIONS_BIN) $(AVG_CAPS_BIN) $(ROBOT_HOMOGENEITY_BIN) $(SAMPLING_TASK_ALLOC_BIN) $(VIS_BIN) $(BS_AUTOMATON_STATES_BIN) $(BS_NUMBER_ROBOTS_BIN) $(BS_TS_REGIONS_BIN) $(BS_AVG_CAPS_BIN) $(BS_ROBOT_HOMOGENEITY_BIN) $(RP_AUTOMATON_STATES_BIN) $(RP_NUMBER_ROBOTS_BIN) $(RP_TS_REGIONS_BIN) $(RP_AVG_CAPS_BIN) $(RP_ROBOT_HOMOGENEITY_BIN)

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
$(AUTOMATON_STATES_BIN): $(AUTOMATON_STATES_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AUTOMATON_STATES_BIN)"

# Build the automaton states batch scaling tests executable
$(AUTOMATON_STATES_BATCH_BIN): $(AUTOMATON_STATES_BATCH_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AUTOMATON_STATES_BATCH_BIN)"

# Build the number robots scaling tests executable
$(NUMBER_ROBOTS_BIN): $(NUMBER_ROBOTS_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(NUMBER_ROBOTS_BIN)"

# Build the transition system regions scaling tests executable
$(TS_REGIONS_BIN): $(TS_REGIONS_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TS_REGIONS_BIN)"

# Build the average capabilities scaling tests executable
$(AVG_CAPS_BIN): $(AVG_CAPS_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(AVG_CAPS_BIN)"

# Build the robot homogeneity scaling tests executable
$(ROBOT_HOMOGENEITY_BIN): $(ROBOT_HOMOGENEITY_OBJ) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
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

# Header dependencies recorded by -MMD. Without these, changing a header leaves stale
# objects that disagree with the rest of the build about class layouts.
-include $(ALL_OBJS:.o=.d)

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

# Run every sweep, then build the cross-model comparison figures from their combined CSVs
compare: test-automaton-states test-automaton-states-batch test-number-robots test-ts-regions test-average-capabilities test-robot-homogeneity
	@echo "Building cross-model comparison..."
	@cd organizedtests && python3 plot_model_comparison.py

# Run tests and show output
test: test-all

# Rebuild from scratch
rebuild: clean all
	@echo "✓ Rebuild complete"

# Clean build artifacts
# ---------------------------------------------------------------------------
# Sampler budget study: runs ONLY the sampler, at 5/20/40/70/100x the tree planner's
# recorded runtime. The tree is never re-run; its timings come from the sweep CSVs,
# so these binaries require the matching organizedtests sweep to have been run first.
# ---------------------------------------------------------------------------

$(BS_AUTOMATON_STATES_BIN): $(BS_AUTOMATON_STATES_OBJS) $(BUDGET_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(BS_AUTOMATON_STATES_BIN)"

budget-automaton-states: $(BS_AUTOMATON_STATES_BIN)
	@echo "Running the sampler budget study for automaton_states..."
	@cd budget_study && ../$(BS_AUTOMATON_STATES_BIN) --trials=$(BUDGET_TRIALS) $(BUDGET_APPEND)

$(BS_NUMBER_ROBOTS_BIN): $(BS_NUMBER_ROBOTS_OBJS) $(BUDGET_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(BS_NUMBER_ROBOTS_BIN)"

budget-number-robots: $(BS_NUMBER_ROBOTS_BIN)
	@echo "Running the sampler budget study for number_robots..."
	@cd budget_study && ../$(BS_NUMBER_ROBOTS_BIN) --trials=$(BUDGET_TRIALS) $(BUDGET_APPEND)

$(BS_TS_REGIONS_BIN): $(BS_TS_REGIONS_OBJS) $(BUDGET_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(BS_TS_REGIONS_BIN)"

budget-ts-regions: $(BS_TS_REGIONS_BIN)
	@echo "Running the sampler budget study for transition_system_regions..."
	@cd budget_study && ../$(BS_TS_REGIONS_BIN) --trials=$(BUDGET_TRIALS) $(BUDGET_APPEND)

$(BS_AVG_CAPS_BIN): $(BS_AVG_CAPS_OBJS) $(BUDGET_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(BS_AVG_CAPS_BIN)"

budget-average-capabilities: $(BS_AVG_CAPS_BIN)
	@echo "Running the sampler budget study for average_capabilities..."
	@cd budget_study && ../$(BS_AVG_CAPS_BIN) --trials=$(BUDGET_TRIALS) $(BUDGET_APPEND)

$(BS_ROBOT_HOMOGENEITY_BIN): $(BS_ROBOT_HOMOGENEITY_OBJS) $(BUDGET_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(BS_ROBOT_HOMOGENEITY_BIN)"

budget-robot-homogeneity: $(BS_ROBOT_HOMOGENEITY_BIN)
	@echo "Running the sampler budget study for robot_homogeneity..."
	@cd budget_study && ../$(BS_ROBOT_HOMOGENEITY_BIN) --trials=$(BUDGET_TRIALS) $(BUDGET_APPEND)

budget-study: budget-automaton-states budget-number-robots budget-ts-regions budget-average-capabilities budget-robot-homogeneity
	@echo "All budget studies complete. Plot with: cd budget_study && python3 plot_budget_study.py"

# ---------------------------------------------------------------------------
# Showcase replay: re-plans the configurations one planner clearly won, with BOTH planners,
# and draws both plans. The budget study throws its planners away, so the plans have to be
# rebuilt to be looked at. Needs budget_study/data/showcase_targets.csv, which
# select_showcase.py writes from the study CSVs.
# ---------------------------------------------------------------------------

$(RP_AUTOMATON_STATES_BIN): $(RP_AUTOMATON_STATES_OBJS) $(REPLAY_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(RP_AUTOMATON_STATES_BIN)"

replay-automaton-states: $(RP_AUTOMATON_STATES_BIN)
	@echo "Replaying the showcased automaton_states configurations..."
	@cd budget_study && ../$(RP_AUTOMATON_STATES_BIN)

$(RP_NUMBER_ROBOTS_BIN): $(RP_NUMBER_ROBOTS_OBJS) $(REPLAY_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(RP_NUMBER_ROBOTS_BIN)"

replay-number-robots: $(RP_NUMBER_ROBOTS_BIN)
	@echo "Replaying the showcased number_robots configurations..."
	@cd budget_study && ../$(RP_NUMBER_ROBOTS_BIN)

$(RP_TS_REGIONS_BIN): $(RP_TS_REGIONS_OBJS) $(REPLAY_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(RP_TS_REGIONS_BIN)"

replay-ts-regions: $(RP_TS_REGIONS_BIN)
	@echo "Replaying the showcased transition_system_regions configurations..."
	@cd budget_study && ../$(RP_TS_REGIONS_BIN)

$(RP_AVG_CAPS_BIN): $(RP_AVG_CAPS_OBJS) $(REPLAY_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(RP_AVG_CAPS_BIN)"

replay-average-capabilities: $(RP_AVG_CAPS_BIN)
	@echo "Replaying the showcased average_capabilities configurations..."
	@cd budget_study && ../$(RP_AVG_CAPS_BIN)

$(RP_ROBOT_HOMOGENEITY_BIN): $(RP_ROBOT_HOMOGENEITY_OBJS) $(REPLAY_COMMON_OBJS) $(SAMPLING_TASK_ALLOC_IMPL_OBJS) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(RP_ROBOT_HOMOGENEITY_BIN)"

replay-robot-homogeneity: $(RP_ROBOT_HOMOGENEITY_BIN)
	@echo "Replaying the showcased robot_homogeneity configurations..."
	@cd budget_study && ../$(RP_ROBOT_HOMOGENEITY_BIN)

showcase: replay-automaton-states replay-number-robots replay-ts-regions replay-average-capabilities replay-robot-homogeneity
	@cd budget_study && python3 plot_win_regimes.py && python3 build_showcase.py

clean:
	@echo "Cleaning build artifacts..."
	@rm -f $(ALL_OBJS)
	@rm -f $(ALL_OBJS:.o=.d)
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
