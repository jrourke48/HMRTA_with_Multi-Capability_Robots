# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3
INCLUDES = -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev" -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev/buddy/src" -I"../MCTB-PDT/include"
LDFLAGS = -L"C:/Users/johnn/Spot Library/spot-2.14.4.dev/lib"
LIBS = -lspot -lbddx

# Output directory
OUTDIR = output

# Create output directory if it doesn't exist
$(OUTDIR):
	mkdir -p $(OUTDIR)

# Default target
all: LTLtoBuchiAutomaton

# Individual targets
LTLtoBuchiAutomaton: $(OUTDIR) LTLtoBuchiAutomaton.cpp ../Transition_Systems/GridWorldTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) LTLtoBuchiAutomaton.cpp ../Transition_Systems/GridWorldTransitionSystem.cpp -o $(OUTDIR)/$@.exe $(LIBS)

TestingAutomaton: $(OUTDIR) TestingAutomaton.cpp ../Transition_Systems/GridWorldTransitionSystem.cpp TS.cpp BuchiAutomaton.cpp ProductAutomaton.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) TestingAutomaton.cpp ../Transition_Systems/GridWorldTransitionSystem.cpp TS.cpp BuchiAutomaton.cpp ProductAutomaton.cpp ../MCTB-PDT/src/Environment/GridWorld.cpp ../MCTB-PDT/src/Environment/Point.cpp ../MCTB-PDT/src/Environment/Environment.cpp ../MCTB-PDT/src/MultiRobotSystem/MultiRobotSystem.cpp ../MCTB-PDT/src/MultiRobotSystem/Robot.cpp ../MCTB-PDT/src/LTLFormula/LTLFormula.cpp ../MCTB-PDT/src/LTLFormula/BatchAtomicProposition.cpp ../MCTB-PDT/src/TaskAllocationAlgorithms.cpp ../MCTB-PDT/src/AlgorithmMetrics.cpp ../MCTB-PDT/src/Tree/PlanningDecisionTree.cpp ../MCTB-PDT/src/Tree/Tree_Node.cpp -o $(OUTDIR)/$@.exe $(LIBS)

GlobalState: $(OUTDIR) GlobalState.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) GlobalState.cpp -o $(OUTDIR)/$@.exe $(LIBS)

ParsingandPrintingLTL: $(OUTDIR) ParsingandPrintingLTL.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) ParsingandPrintingLTL.cpp -o $(OUTDIR)/$@.exe $(LIBS)

RelablingFormulas: $(OUTDIR) RelablingFormulas.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) RelablingFormulas.cpp -o $(OUTDIR)/$@.exe $(LIBS)

TestingEquivalance: $(OUTDIR) TestingEquivalance.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) TestingEquivalance.cpp -o $(OUTDIR)/$@.exe $(LIBS)

BasicTransitionSystem: $(OUTDIR) ../Transition_Systems/BasicTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) ../Transition_Systems/BasicTransitionSystem.cpp -o $(OUTDIR)/$@.exe $(LIBS)

GeneralTransitionSystem: $(OUTDIR) ../Transition_Systems/GeneralTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) ../Transition_Systems/GeneralTransitionSystem.cpp -o $(OUTDIR)/$@.exe $(LIBS)

GridWorldTransitionSystem: $(OUTDIR) ../Transition_Systems/GridWorldTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) ../Transition_Systems/GridWorldTransitionSystem.cpp -o $(OUTDIR)/$@.exe $(LIBS)

SpotGridWorldTS: $(OUTDIR) ../Transition_Systems/SpotGridWorldTS.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) ../Transition_Systems/SpotGridWorldTS.cpp -o $(OUTDIR)/$@.exe $(LIBS)

# Run targets (build and execute)
run-LTLtoBuchiAutomaton: LTLtoBuchiAutomaton plots-LTLtoBuchiAutomaton
	./$(OUTDIR)/LTLtoBuchiAutomaton.exe

run-TestingAutomaton: TestingAutomaton plots-TestingAutomaton
	./$(OUTDIR)/TestingAutomaton.exe

# Plot generation targets
plots-LTLtoBuchiAutomaton:
	dot -Tpng $(OUTDIR)/buchi_automaton.dot -o $(OUTDIR)/buchi_automaton.png
	dot -Tpng $(OUTDIR)/ts_automaton.dot -o $(OUTDIR)/ts_automaton.png
	dot -Tpng $(OUTDIR)/product_automaton.dot -o $(OUTDIR)/product_automaton.png
	@echo "Generated PNG plots in $(OUTDIR)/"

run-GlobalState: GlobalState
	./$(OUTDIR)/GlobalState.exe

run-ParsingandPrintingLTL: ParsingandPrintingLTL
	./$(OUTDIR)/ParsingandPrintingLTL.exe

run-RelablingFormulas: RelablingFormulas
	./$(OUTDIR)/RelablingFormulas.exe

run-TestingEquivalance: TestingEquivalance
	./$(OUTDIR)/TestingEquivalance.exe

run-BasicTransitionSystem: BasicTransitionSystem
	./$(OUTDIR)/BasicTransitionSystem.exe

run-GeneralTransitionSystem: GeneralTransitionSystem
	./$(OUTDIR)/GeneralTransitionSystem.exe

run-GridWorldTransitionSystem: GridWorldTransitionSystem
	./$(OUTDIR)/GridWorldTransitionSystem.exe

run-SpotGridWorldTS: SpotGridWorldTS
	./$(OUTDIR)/SpotGridWorldTS.exe

# Clean build artifacts
clean:
	rm -f $(OUTDIR)/*.exe

.PHONY: all clean run-LTLtoBuchiAutomaton run-TestingAutomaton run-GlobalState run-ParsingandPrintingLTL run-RelablingFormulas run-TestingEquivalance run-BasicTransitionSystem run-GeneralTransitionSystem run-GridWorldTransitionSystem run-SpotGridWorldTS plots-LTLtoBuchiAutomaton plots-TestingAutomaton
