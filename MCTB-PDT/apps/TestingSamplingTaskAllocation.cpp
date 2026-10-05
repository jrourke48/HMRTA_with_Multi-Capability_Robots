#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <cstdlib>
#include <algorithm>
#include "../include/TaskAllocationAlgorithms.h"
#include "../include/Environment/gridvis.h"
#include "../include/Tree/PlanningDecisionTree.h"
#include "../include/Tree/Tree_Node.h"
#include "../include/Environment/Environment.h"
#include "../include/MultiRobotSystem/MultiRobotSystem.h"
#include "../include/LTLFormula/LTLFormula.h"
#include "../include/TestRunManager.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/ProductAutomaton.h"
#include "RandomSamplingAlgo/RandomSamplingTaskAllocation.h"
#include "RandomSamplingAlgo/RandomNode.h"

using namespace std;
using TaskAllocPath = RandomSamplingTaskAllocation::TaskAllocPath;

// ============================================================================
// MINIMAL TEST HARNESS
// ============================================================================

static int numPassed = 0;
static int numFailed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { numPassed++; cout << "    [PASS] " << msg << endl; } \
    else { numFailed++; cout << "    [FAIL] " << msg << "  (" << __FILE__ << ":" << __LINE__ << ")" << endl; } \
} while (0)

// Number of random trials used by tests that exercise randomized code
static const int NUM_TRIALS = 50;

// Tests
void testConstructorsAndAccessors();
void testPruneInfeasibleNBAPaths();
void testGetMinLengthPath();
void testGetRandomFeasibleTaskAllocation();
void testBuildRandomPath();
void testRunFinitePlanner();
void testRunInfinitePlanner();
void testInitialStateAccepting();
void testTimeLimitStopsSampling();

// Fixtures
void createTestSystemComponents2(TS*& ts, GridWorld*& grid, Environment*& env);
MultiRobotSystem* createTestMultiRobotSystem2();
MultiRobotSystem* createNoCameraMultiRobotSystem();
BuchiAutomaton* createTestBuchiAutomaton();
BuchiAutomaton* createTestBuchiAutomaton2();
BuchiAutomaton* createTestBuchiAutomaton3();
BuchiAutomaton* createInitialAcceptingBuchiAutomaton();
void cleanup(BuchiAutomaton* buchi, MultiRobotSystem* mrs, Environment* env, TS* ts, GridWorld* grid);

// Independent reference helpers (do not call the code under test)
map<uint16_t, vector<uint16_t>> snapshotEdges(BuchiAutomaton* buchi);
uint16_t referenceMinLength(BuchiAutomaton* buchi, uint16_t srcId, uint16_t goalId);
vector<vector<uint16_t>> edgeAPSets(BuchiAutomaton* buchi, uint16_t srcId, uint16_t dstId);
bool apSetCoveredByTeam(BuchiAutomaton* buchi, MultiRobotSystem* mrs, uint16_t ap, const vector<uint8_t>& robots);
bool edgeIsDeterministicallyFeasible(BuchiAutomaton* buchi, MultiRobotSystem* mrs, uint16_t srcId, uint16_t dstId);
bool validateStep(BuchiAutomaton* searchNBA, MultiRobotSystem* mrs, uint16_t srcId, uint16_t dstId,
                  const vector<vector<uint8_t>>& robotsByAP, const vector<uint16_t>& satisfiedAPs, string& err);
bool validatePath(const TaskAllocPath& p, BuchiAutomaton* searchNBA, MultiRobotSystem* mrs,
                  uint16_t startId, uint16_t goalId, bool isCycle, string& err);
void printAutomaton(BuchiAutomaton* buchi, const string& name);

// Reports what the planner actually produced, rather than asserting anything about it
void printTaskAllocPath(const string& label, const TaskAllocPath& p);
void printPlan(RandomSamplingTaskAllocation& sampler);
void runPlannerShowcase();

int main() {
    srand(12345);  // Fixed seed so failures are reproducible
    cout << "\n=== TESTING RANDOM SAMPLING TASK ALLOCATION ===" << endl;

    cout << "\n--- Test 1: Constructors, getters and setters ---" << endl;
    testConstructorsAndAccessors();

    cout << "\n--- Test 2: pruneInfeasibleNBAPaths ---" << endl;
    testPruneInfeasibleNBAPaths();

    cout << "\n--- Test 3: getMinLengthPath ---" << endl;
    testGetMinLengthPath();

    cout << "\n--- Test 4: getRandomFeasibleTaskAllocation ---" << endl;
    testGetRandomFeasibleTaskAllocation();

    cout << "\n--- Test 5: buildRandomPath (prefix paths and cycles) ---" << endl;
    testBuildRandomPath();

    cout << "\n--- Test 6: run() on a finite NBA (runFinitePathPlanner) ---" << endl;
    testRunFinitePlanner();

    cout << "\n--- Test 7: run() on an infinite NBA (runInfinitePathPlanner) ---" << endl;
    testRunInfinitePlanner();

    cout << "\n--- Test 8: Accepting initial state still walks a first cycle ---" << endl;
    testInitialStateAccepting();

    cout << "\n--- Test 9: Time limit stops sampling ---" << endl;
    testTimeLimitStopsSampling();

    cout << "\n=== SUMMARY: " << numPassed << " passed, " << numFailed << " failed ===" << endl;

    runPlannerShowcase();

    return numFailed == 0 ? 0 : 1;
}

// ============================================================================
// TESTS
// ============================================================================

void testConstructorsAndAccessors() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton2();

    {
        RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)100);
        CHECK(sampler.getMaxIterations() == 100, "iteration constructor stores maxIterations");
        CHECK(sampler.getTimeLimit() == 0.0, "iteration constructor leaves timeLimit at 0");
        CHECK(sampler.getIterations() == 0, "iterations start at 0");
        CHECK(sampler.getNumNBAStates() == buchi->getNumStates(), "numNBAStates matches the NBA");
        CHECK(sampler.getNBA() == buchi && sampler.getEnvironment() == env && sampler.getMultiRobotSystem() == mrs,
              "system component getters return constructor arguments");
        CHECK(sampler.getPrunedNBA() == nullptr, "pruned NBA is null before pruning");
        CHECK(sampler.getAllPaths().empty(), "no paths stored before run()");
        const auto& missing = sampler.getPath(9999);
        CHECK(missing.first.path.empty() && missing.second.path.empty() && missing.first.pathLength == 0,
              "getPath on an unknown accepting state returns empty paths");

        sampler.setMaxIterations(7);
        sampler.setTimeLimit(1.5);
        sampler.setIterations(3);
        CHECK(sampler.getMaxIterations() == 7 && sampler.getTimeLimit() == 1.5 && sampler.getIterations() == 3,
              "setters round-trip through getters");
    }
    {
        RandomSamplingTaskAllocation sampler(buchi, env, mrs, 5.0);
        CHECK(sampler.getTimeLimit() == 5.0, "time constructor stores timeLimit");
        CHECK(sampler.getMaxIterations() == 0, "time constructor leaves maxIterations at 0");
    }

    cleanup(buchi, mrs, env, ts, grid);
}

void testPruneInfeasibleNBAPaths() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* fullMRS = createTestMultiRobotSystem2();
    MultiRobotSystem* noCameraMRS = createNoCameraMultiRobotSystem();

    struct Case { string name; MultiRobotSystem* mrs; };
    vector<Case> cases = {{"all capabilities", fullMRS}, {"no camera robots", noCameraMRS}};

    for (const auto& c : cases) {
        cout << "  Case: " << c.name << endl;
        BuchiAutomaton* buchi = createTestBuchiAutomaton2();
        map<uint16_t, vector<uint16_t>> original = snapshotEdges(buchi);

        RandomSamplingTaskAllocation sampler(buchi, env, c.mrs, (uint16_t)1);
        sampler.pruneInfeasibleNBAPaths();
        BuchiAutomaton* pruned = sampler.getPrunedNBA();

        CHECK(pruned != nullptr && pruned != buchi, "pruning creates a separate NBA");
        if (!pruned) { delete buchi; continue; }

        CHECK(pruned->getNumStates() == buchi->getNumStates() &&
              pruned->getInitialState() == buchi->getInitialState() &&
              pruned->getAcceptingStates() == buchi->getAcceptingStates(),
              "pruned NBA keeps the same states, initial state and accepting states");

        // Pruning must not modify the caller's NBA
        CHECK(snapshotEdges(buchi) == original, "original NBA edges are unchanged after pruning");

        // Every kept edge existed originally
        bool subset = true;
        for (const auto& [id, dsts] : snapshotEdges(pruned)) {
            for (uint16_t d : dsts) {
                if (find(original[id].begin(), original[id].end(), d) == original[id].end()) subset = false;
            }
        }
        CHECK(subset, "pruned edges are a subset of the original edges");

        // An edge should be kept exactly when some team of robots can satisfy one of its AP sets.
        // Edge labels are read from a freshly built, unpruned copy of the same NBA.
        BuchiAutomaton* reference = createTestBuchiAutomaton2();
        int wronglyRemoved = 0, wronglyKept = 0, unconstrainedRemoved = 0;
        map<uint16_t, vector<uint16_t>> prunedEdges = snapshotEdges(pruned);
        for (const auto& [id, dsts] : original) {
            for (uint16_t d : dsts) {
                bool kept = find(prunedEdges[id].begin(), prunedEdges[id].end(), d) != prunedEdges[id].end();
                // An edge requires no tasks when one of its conjunctions has no APs at all
                vector<vector<uint16_t>> refSets = edgeAPSets(reference, id, d);
                bool unconstrained = any_of(refSets.begin(), refSets.end(),
                                            [](const vector<uint16_t>& s) { return s.empty(); });
                bool feasible = edgeIsDeterministicallyFeasible(reference, c.mrs, id, d);
                if (unconstrained) { if (!kept) unconstrainedRemoved++; }
                else if (feasible && !kept) wronglyRemoved++;
                else if (!feasible && kept) wronglyKept++;
            }
        }
        delete reference;
        CHECK(wronglyKept == 0, "no infeasible edge is kept (" + to_string(wronglyKept) + " wrongly kept)");
        CHECK(wronglyRemoved == 0, "no feasible edge is removed (" + to_string(wronglyRemoved) + " wrongly removed)");
        CHECK(unconstrainedRemoved == 0,
              "edges requiring no tasks (e.g. '!p1' self-loops or 'true') are kept (" + to_string(unconstrainedRemoved) + " removed)");

        delete buchi;
    }

    // Pruning should give the same result every time for the same inputs
    set<map<uint16_t, vector<uint16_t>>> distinctResults;
    for (int t = 0; t < 10; t++) {
        BuchiAutomaton* buchi = createTestBuchiAutomaton2();
        RandomSamplingTaskAllocation sampler(buchi, env, fullMRS, (uint16_t)1);
        sampler.pruneInfeasibleNBAPaths();
        distinctResults.insert(snapshotEdges(sampler.getPrunedNBA()));
        delete buchi;
    }
    CHECK(distinctResults.size() == 1,
          "pruning is deterministic across 10 runs (" + to_string(distinctResults.size()) + " distinct results)");

    delete noCameraMRS;
    cleanup(nullptr, fullMRS, env, ts, grid);
}

void testGetMinLengthPath() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton2();

    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)1);
    sampler.pruneInfeasibleNBAPaths();
    BuchiAutomaton* pruned = sampler.getPrunedNBA();
    Node* anyNode = pruned->getNode(pruned->getInitialState());

    CHECK(sampler.getMinLengthPath(nullptr, anyNode).pathLength == 0, "null source returns length 0");
    CHECK(sampler.getMinLengthPath(anyNode, nullptr).pathLength == 0, "null goal returns length 0");
    CHECK(sampler.getMinLengthPath(anyNode, anyNode).pathLength == 1, "source == goal returns length 1");

    // Compare every pair of states against an independent BFS over the pruned NBA
    int mismatches = 0, pairs = 0, reachable = 0;
    for (const auto& [srcId, srcNode] : pruned->getNodes()) {
        for (const auto& [dstId, dstNode] : pruned->getNodes()) {
            uint16_t expected = referenceMinLength(pruned, srcId, dstId);
            uint16_t actual = sampler.getMinLengthPath(srcNode, dstNode).pathLength;
            if (expected != actual) {
                mismatches++;
                cout << "      mismatch " << srcId << "->" << dstId << ": expected " << expected << ", got " << actual << endl;
            }
            if (expected > 1) reachable++;
            pairs++;
        }
    }
    CHECK(mismatches == 0, "matches reference BFS for all " + to_string(pairs) + " state pairs (" +
          to_string(reachable) + " reachable, rest unreachable or same)");

    cleanup(buchi, mrs, env, ts, grid);
}

void testGetRandomFeasibleTaskAllocation() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton2();

    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)1);
    sampler.pruneInfeasibleNBAPaths();
    BuchiAutomaton* pruned = sampler.getPrunedNBA();

    int edgesTested = 0, invalid = 0;
    string firstErr;
    for (const auto& [srcId, srcNode] : pruned->getNodes()) {
        for (const auto& edge : srcNode->getEdges()) {
            uint16_t dstId = edge.getDstId();
            // Only call on edges that can be satisfied, otherwise the method loops forever by design
            if (!edgeIsDeterministicallyFeasible(pruned, mrs, srcId, dstId)) continue;
            edgesTested++;
            for (int t = 0; t < NUM_TRIALS; t++) {
                auto [robotsByAP, satisfiedAPs] = sampler.getRandomFeasibleTaskAllocation(srcNode, pruned->getNode(dstId));
                string err;
                if (!validateStep(pruned, mrs, srcId, dstId, robotsByAP, satisfiedAPs, err)) {
                    if (invalid == 0) firstErr = to_string(srcId) + "->" + to_string(dstId) + ": " + err;
                    invalid++;
                }
            }
        }
    }
    CHECK(edgesTested > 0, "found feasible edges to test (" + to_string(edgesTested) + ")");
    CHECK(invalid == 0, "every allocation over " + to_string(NUM_TRIALS) + " trials per edge is valid" +
          (invalid ? " (first error: " + firstErr + ")" : ""));

    cleanup(buchi, mrs, env, ts, grid);
}

void testBuildRandomPath() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton();  // infinite NBA, has cycles

    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)1);
    sampler.pruneInfeasibleNBAPaths();
    BuchiAutomaton* pruned = sampler.getPrunedNBA();
    printAutomaton(pruned, "pruned infinite NBA");
    uint16_t initialId = pruned->getInitialState();
    uint16_t numStates = pruned->getNumStates();
    Node* initialNode = pruned->getNode(initialId);

    int prefixCases = 0, cycleCases = 0;
    for (uint16_t acc : pruned->getAcceptingStates()) {
        Node* accNode = pruned->getNode(acc);

        // Prefix: initial -> accepting
        uint16_t minLen = referenceMinLength(pruned, initialId, acc);
        if (acc != initialId && minLen > 0) {
            prefixCases++;
            string err;
            TaskAllocPath shortest = sampler.buildRandomPath(pruned, initialNode, accNode, minLen, false, 0);
            bool ok = validatePath(shortest, pruned, mrs, initialId, acc, false, err) && shortest.pathLength == minLen;
            CHECK(ok, "accepting " + to_string(acc) + ": target = min length (" + to_string(minLen) + ") gives a valid shortest path" +
                  (ok ? "" : " (" + err + ")"));
            for (Random_Node* n : shortest.path) delete n;

            TaskAllocPath tooShort = sampler.buildRandomPath(pruned, initialNode, accNode, minLen - 1, false, 0);
            CHECK(tooShort.path.empty() && tooShort.pathLength == 0,
                  "accepting " + to_string(acc) + ": target below min length returns an empty path");
            for (Random_Node* n : tooShort.path) delete n;

            int invalid = 0; string firstErr; set<uint16_t> lengthsSeen;
            for (int t = 0; t < NUM_TRIALS; t++) {
                TaskAllocPath p = sampler.buildRandomPath(pruned, initialNode, accNode, numStates, false, 0);
                string e;
                if (!validatePath(p, pruned, mrs, initialId, acc, false, e) || p.pathLength > numStates) {
                    if (!invalid) firstErr = e;
                    invalid++;
                }
                lengthsSeen.insert(p.pathLength);
                for (Random_Node* n : p.path) delete n;
            }
            CHECK(invalid == 0, "accepting " + to_string(acc) + ": " + to_string(NUM_TRIALS) +
                  " random prefixes with target " + to_string(numStates) + " are valid" + (invalid ? " (" + firstErr + ")" : ""));
            cout << "      distinct prefix lengths sampled: " << lengthsSeen.size() << endl;
        }

        // Suffix: accepting -> accepting cycle
        uint16_t minCycle = 0;
        for (const auto& edge : accNode->getEdges()) {
            uint16_t back = referenceMinLength(pruned, edge.getDstId(), acc);
            if (back != 0 && (minCycle == 0 || back + 1 < minCycle)) minCycle = back + 1;
        }
        if (minCycle > 0) {
            cycleCases++;
            int invalid = 0; string firstErr;
            for (int t = 0; t < NUM_TRIALS; t++) {
                TaskAllocPath p = sampler.buildRandomPath(pruned, accNode, accNode, numStates + 1, false, 0);
                string e;
                if (!validatePath(p, pruned, mrs, acc, acc, true, e)) {
                    if (!invalid) firstErr = e;
                    invalid++;
                }
                for (Random_Node* n : p.path) delete n;
            }
            CHECK(invalid == 0, "accepting " + to_string(acc) + ": " + to_string(NUM_TRIALS) +
                  " random cycles are valid and take at least one step" + (invalid ? " (" + firstErr + ")" : ""));

            TaskAllocPath tooShort = sampler.buildRandomPath(pruned, accNode, accNode, minCycle - 1, false, 0);
            CHECK(tooShort.path.empty(), "accepting " + to_string(acc) + ": cycle target below min cycle length returns empty");
            for (Random_Node* n : tooShort.path) delete n;
        }
    }
    CHECK(prefixCases + cycleCases > 0, "exercised " + to_string(prefixCases) + " prefix cases and " +
          to_string(cycleCases) + " cycle cases");

    // Makespan bound: a bound below the path's makespan must reject it. With placeholder zero times
    // every makespan is 0, so the only observable rule is that a bound of 0 does not reject.
    if (!pruned->getAcceptingStates().empty()) {
        uint16_t acc = pruned->getAcceptingStates()[0];
        uint16_t minLen = referenceMinLength(pruned, initialId, acc);
        if (acc != initialId && minLen > 0) {
            TaskAllocPath p = sampler.buildRandomPath(pruned, initialNode, pruned->getNode(acc), minLen, true, 0);
            CHECK(!p.path.empty() && p.makespan == 0, "bound equal to the makespan (0) does not reject the path");
            for (Random_Node* n : p.path) delete n;
        }
    }

    cleanup(buchi, mrs, env, ts, grid);
}

void testRunFinitePlanner() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton3();
    CHECK(buchi->isFinite(), "fixture NBA is finite");

    // Iteration constructor alone should run the requested number of iterations
    {
        RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)20);
        sampler.run();
        CHECK(sampler.getIterations() == 20,
              "iteration constructor alone runs all 20 iterations (ran " + to_string(sampler.getIterations()) + ")");
    }

    // Set both limits so the sampling loop runs regardless of how the limits are combined
    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)200);
    sampler.setTimeLimit(30.0);
    sampler.run();
    BuchiAutomaton* pruned = sampler.getPrunedNBA();
    uint16_t initialId = pruned->getInitialState();

    CHECK(sampler.getIterations() == 200, "runs 200 iterations when both limits are set (ran " +
          to_string(sampler.getIterations()) + ")");
    CHECK(sampler.getAllPaths().size() == pruned->getAcceptingStates().size(), "one entry per accepting state");

    int reachable = 0;
    for (uint16_t acc : pruned->getAcceptingStates()) {
        const auto& [prefix, suffix] = sampler.getPath(acc);
        uint16_t minLen = referenceMinLength(pruned, initialId, acc);
        CHECK(suffix.path.empty() && suffix.pathLength == 0, "accepting " + to_string(acc) + ": finite NBA has no suffix");
        if (minLen == 0) {
            CHECK(prefix.path.empty(), "accepting " + to_string(acc) + ": unreachable, so no path is stored");
            continue;
        }
        reachable++;
        string err;
        bool ok = validatePath(prefix, pruned, mrs, initialId, acc, false, err);
        CHECK(ok, "accepting " + to_string(acc) + ": stored path is a valid path from the initial state" + (ok ? "" : " (" + err + ")"));
        CHECK(prefix.pathLength >= minLen && prefix.pathLength <= pruned->getNumStates(),
              "accepting " + to_string(acc) + ": path length " + to_string(prefix.pathLength) + " is within [" +
              to_string(minLen) + ", " + to_string(pruned->getNumStates()) + "]");
    }
    CHECK(reachable > 0, "at least one accepting state is reachable (" + to_string(reachable) + ")");

    cleanup(buchi, mrs, env, ts, grid);
}

void testRunInfinitePlanner() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton();
    CHECK(buchi->isInfinite(), "fixture NBA is infinite");

    // State 0 is both the initial state and accepting here, so its lasso is a bare suffix
    const auto& fixtureAccepting = buchi->getAcceptingStates();
    CHECK(find(fixtureAccepting.begin(), fixtureAccepting.end(), buchi->getInitialState()) != fixtureAccepting.end(),
          "fixture NBA has an accepting initial state");

    {
        RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)20);
        sampler.run();
        CHECK(sampler.getIterations() == 20,
              "iteration constructor alone runs all 20 iterations (ran " + to_string(sampler.getIterations()) + ")");
    }

    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)200);
    sampler.setTimeLimit(30.0);
    sampler.run();
    BuchiAutomaton* pruned = sampler.getPrunedNBA();
    uint16_t initialId = pruned->getInitialState();

    CHECK(sampler.getIterations() == 200, "runs 200 iterations when both limits are set (ran " +
          to_string(sampler.getIterations()) + ")");

    int withPlan = 0, initialAcceptingPlans = 0;
    for (uint16_t acc : pruned->getAcceptingStates()) {
        const auto& [prefix, suffix] = sampler.getPath(acc);
        bool reachable = acc == initialId || referenceMinLength(pruned, initialId, acc) > 0;
        bool onCycle = false;
        for (const auto& edge : pruned->getNode(acc)->getEdges()) {
            if (referenceMinLength(pruned, edge.getDstId(), acc) > 0) onCycle = true;
        }
        if (!reachable || !onCycle) {
            CHECK(suffix.path.empty(), "accepting " + to_string(acc) + ": not reachable or not on a cycle, so no plan is stored");
            continue;
        }
        withPlan++;
        string err;
        bool sufOk = validatePath(suffix, pruned, mrs, acc, acc, true, err);
        CHECK(sufOk, "accepting " + to_string(acc) + ": suffix is a valid cycle" + (sufOk ? "" : " (" + err + ")"));
        CHECK(suffix.pathLength <= pruned->getNumStates() + 1, "accepting " + to_string(acc) + ": suffix length is within bound");
        if (acc == initialId) {
            initialAcceptingPlans++;
            // Starting on the accepting state is not a visit to it, so the run still owes a first
            // loop. The prefix is therefore a cycle, and the lasso reaches the state twice over.
            string pcerr;
            bool preCycleOk = validatePath(prefix, pruned, mrs, acc, acc, true, pcerr);
            CHECK(preCycleOk, "accepting " + to_string(acc) +
                  ": is the initial state, so the prefix is a first cycle" +
                  (preCycleOk ? "" : " (" + pcerr + ")"));
            CHECK(prefix.pathLength >= 2, "accepting " + to_string(acc) +
                  ": the first cycle takes at least one step");
        } else {
            string perr;
            bool preOk = validatePath(prefix, pruned, mrs, initialId, acc, false, perr);
            CHECK(preOk, "accepting " + to_string(acc) + ": prefix runs from the initial state to it" + (preOk ? "" : " (" + perr + ")"));
        }
    }
    CHECK(withPlan > 0, "at least one accepting state is reachable and on a cycle (" + to_string(withPlan) + ")");
    // Guards against the accepting-initial branch above passing vacuously if state 0 stops
    // being planned for, e.g. because pruning leaves it off every cycle
    CHECK(initialAcceptingPlans > 0, "the accepting initial state was planned for (" +
          to_string(initialAcceptingPlans) + ")");

    cleanup(buchi, mrs, env, ts, grid);
}

void testInitialStateAccepting() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createInitialAcceptingBuchiAutomaton();
    printAutomaton(buchi, "G F p0 NBA");

    uint16_t initialId = buchi->getInitialState();
    bool initialAccepting = buchi->isAcceptingState(initialId);
    CHECK(initialAccepting, "fixture NBA has an accepting initial state");
    if (initialAccepting) {
        RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)50);
        sampler.setTimeLimit(30.0);
        sampler.run();
        const auto& [prefix, suffix] = sampler.getPath(initialId);
        string perr;
        bool preOk = validatePath(prefix, sampler.getPrunedNBA(), mrs, initialId, initialId, true, perr);
        CHECK(preOk, "a first cycle is built as the prefix" + (preOk ? string("") : " (" + perr + ")"));
        string err;
        bool ok = validatePath(suffix, sampler.getPrunedNBA(), mrs, initialId, initialId, true, err);
        CHECK(ok, "a valid suffix cycle is built from the initial state" + (ok ? string("") : " (" + err + ")"));
        // Both halves are cycles on the same state, so the run visits it twice after starting there
        CHECK(prefix.pathLength >= 2 && suffix.pathLength >= 2,
              "the lasso visits the accepting state twice after the start");
    }

    cleanup(buchi, mrs, env, ts, grid);
}

void testTimeLimitStopsSampling() {
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton();

    // Time constructor alone: should keep sampling until the time limit
    RandomSamplingTaskAllocation sampler(buchi, env, mrs, 0.2);
    sampler.run();
    CHECK(sampler.getIterations() > 0, "time constructor alone runs at least one iteration (ran " +
          to_string(sampler.getIterations()) + ")");
    CHECK(sampler.getIterations() > 0 && sampler.getComputationTime() >= 0.2,
          "sampling runs until the 0.2s time limit (took " + to_string(sampler.getComputationTime()) + "s)");

    cleanup(buchi, mrs, env, ts, grid);
}

// ============================================================================
// REFERENCE HELPERS
// ============================================================================

map<uint16_t, vector<uint16_t>> snapshotEdges(BuchiAutomaton* buchi) {
    map<uint16_t, vector<uint16_t>> result;
    for (const auto& [id, node] : buchi->getNodes()) {
        vector<uint16_t> dsts;
        for (const auto& edge : node->getEdges()) dsts.push_back(edge.getDstId());
        sort(dsts.begin(), dsts.end());
        result[id] = dsts;
    }
    return result;
}

// Fewest nodes on a path from srcId to goalId (1 if equal, 0 if unreachable)
uint16_t referenceMinLength(BuchiAutomaton* buchi, uint16_t srcId, uint16_t goalId) {
    if (srcId == goalId) return 1;
    map<uint16_t, uint16_t> dist;
    queue<uint16_t> q;
    dist[srcId] = 1;
    q.push(srcId);
    while (!q.empty()) {
        uint16_t cur = q.front(); q.pop();
        Node* node = buchi->getNode(cur);
        if (!node) continue;
        for (const auto& edge : node->getEdges()) {
            uint16_t nxt = edge.getDstId();
            if (dist.count(nxt)) continue;
            dist[nxt] = dist[cur] + 1;
            if (nxt == goalId) return dist[nxt];
            q.push(nxt);
        }
    }
    return 0;
}

bool apSetCoveredByTeam(BuchiAutomaton* buchi, MultiRobotSystem* mrs, uint16_t ap, const vector<uint8_t>& robots) {
    vector<bool> required = buchi->getLTLFormula()->getRequiredCapabilities(ap);
    vector<bool> combined(required.size(), false);
    for (uint8_t r : robots) {
        vector<bool> caps = mrs->getRobotCapabilities(r);  // teams already carry 1-based ids
        for (size_t j = 0; j < caps.size() && j < combined.size(); j++) combined[j] = combined[j] || caps[j];
    }
    for (size_t j = 0; j < required.size(); j++) {
        if (required[j] && !combined[j]) return false;
    }
    return true;
}

// Read AP sets off the edges directly, as the sampler does. BuchiAutomaton::getTrueAPs drops
// multi-AP conjunctions, which the sampler does serve, so it cannot be the reference here.
vector<vector<uint16_t>> edgeAPSets(BuchiAutomaton* buchi, uint16_t srcId, uint16_t dstId) {
    vector<vector<uint16_t>> sets;
    Node* src = buchi->getNode(srcId);
    if (!src) return sets;
    for (const Edge& e : src->getEdgestoNode(dstId)) {
        for (const auto& apSet : e.getTrueAPs()) sets.push_back(apSet);
    }
    return sets;
}

// An edge is feasible when at least one of its conjunctions can be served by the team. A robot
// serves one proposition only, so for every capability the conjunction needs there must be at
// least as many robots carrying it as there are propositions requiring it.
bool edgeIsDeterministicallyFeasible(BuchiAutomaton* buchi, MultiRobotSystem* mrs, uint16_t srcId, uint16_t dstId) {
    map<size_t, int> supply;
    for (uint8_t r = 0; r < mrs->getNumRobots(); r++) {
        vector<bool> caps = mrs->getRobotCapabilities(r + 1);  // robot IDs are 1-based
        for (size_t j = 0; j < caps.size(); j++) {
            if (caps[j]) supply[j]++;
        }
    }
    for (const auto& apSet : edgeAPSets(buchi, srcId, dstId)) {
        map<size_t, int> demand;
        bool all = true;
        for (uint16_t ap : apSet) {
            vector<bool> required = buchi->getLTLFormula()->getRequiredCapabilities(ap);
            for (size_t j = 0; j < required.size() && all; j++) {
                if (required[j] && ++demand[j] > supply[j]) all = false;
            }
            if (!all) break;
        }
        if (all) return true;
    }
    return false;
}

bool validateStep(BuchiAutomaton* searchNBA, MultiRobotSystem* mrs, uint16_t srcId, uint16_t dstId,
                  const vector<vector<uint8_t>>& robotsByAP, const vector<uint16_t>& satisfiedAPs, string& err) {
    auto options = edgeAPSets(searchNBA, srcId, dstId);
    if (find(options.begin(), options.end(), satisfiedAPs) == options.end()) {
        err = "satisfied AP set is not one of the edge's AP sets"; return false;
    }
    if (robotsByAP.size() != satisfiedAPs.size()) { err = "one team per satisfied AP expected"; return false; }
    set<uint8_t> used;
    for (size_t i = 0; i < satisfiedAPs.size(); i++) {
        for (uint8_t r : robotsByAP[i]) {
            if (r < 1 || r > mrs->getNumRobots()) { err = "robot id out of range"; return false; }
            if (!used.insert(r).second) { err = "robot assigned to two APs"; return false; }
        }
        if (!apSetCoveredByTeam(searchNBA, mrs, satisfiedAPs[i], robotsByAP[i])) {
            err = "team for AP p" + to_string(satisfiedAPs[i]) + " lacks required capabilities"; return false;
        }
    }
    return true;
}

bool validatePath(const TaskAllocPath& p, BuchiAutomaton* searchNBA, MultiRobotSystem* mrs,
                  uint16_t startId, uint16_t goalId, bool isCycle, string& err) {
    if (p.path.empty()) { err = "path is empty"; return false; }
    if (p.pathLength != p.path.size()) { err = "pathLength does not match number of nodes"; return false; }
    if (isCycle && p.path.size() < 2) { err = "cycle must take at least one step"; return false; }
    if (p.path.front()->getautomatonState()->getId() != startId) { err = "path does not start at " + to_string(startId); return false; }
    if (p.path.back()->getautomatonState()->getId() != goalId) { err = "path does not end at " + to_string(goalId); return false; }
    if (p.path.back()->getNext() != nullptr) { err = "last node has a next pointer"; return false; }
    uint16_t makespan = 0;
    for (size_t k = 0; k < p.path.size(); k++) {
        Random_Node* n = p.path[k];
        if (n->getNodeId() != k) { err = "node " + to_string(k) + " has id " + to_string(n->getNodeId()); return false; }
        if (k + 1 < p.path.size() && n->getNext() != p.path[k + 1]) { err = "next pointer chain broken at " + to_string(k); return false; }
        if (k == 0) continue;
        uint16_t prevId = p.path[k - 1]->getautomatonState()->getId();
        uint16_t curId = n->getautomatonState()->getId();
        if (!searchNBA->getNode(prevId)->isAdjacent(curId)) {
            err = "step " + to_string(prevId) + "->" + to_string(curId) + " is not an edge of the pruned NBA"; return false;
        }
        string stepErr;
        if (!validateStep(searchNBA, mrs, prevId, curId, n->getTaskAllocations(), n->getTrueAPs(), stepErr)) {
            err = "step " + to_string(prevId) + "->" + to_string(curId) + ": " + stepErr; return false;
        }
        makespan += n->getCurmakespan();
    }
    if (makespan != p.makespan) { err = "stored makespan does not match sum of node makespans"; return false; }
    return true;
}

void printAutomaton(BuchiAutomaton* buchi, const string& name) {
    cout << "    " << name << ": initial " << buchi->getInitialState() << ", accepting { ";
    for (uint16_t a : buchi->getAcceptingStates()) cout << a << " ";
    cout << "}" << endl;
    for (const auto& [id, node] : buchi->getNodes()) {
        cout << "      " << id << " ->";
        for (const auto& edge : node->getEdges()) cout << " " << edge.getDstId() << "[" << edge.getLabel() << "]";
        cout << endl;
    }
}

// ============================================================================
// TEST FIXTURES
// ============================================================================

void cleanup(BuchiAutomaton* buchi, MultiRobotSystem* mrs, Environment* env, TS* ts, GridWorld* grid) {
    delete buchi;
    delete mrs;
    delete env;
    delete ts;
    delete grid;
}

/**
 * Create test environment with TS and GridWorld
 */
void createTestSystemComponents2(TS*& ts, GridWorld*& grid, Environment*& env) {
    // Allocate GridWorld
    grid = new GridWorld(210, 210);
    cout << "✓ GridWorld created (210x210)" << endl;
    
    // Allocate Transition System
    ts = new TS();
    
    // Add 6 states 
    Node* node0 = new Node(0, "R0");
    Node* node1 = new Node(1, "R1");
    Node* node2 = new Node(2, "R2");
    Node* node3 = new Node(3, "R3");
    Node* node4 = new Node(4, "R4");
    Node* node5 = new Node(5, "R5");

    //with edges: 0-2 1-2 2-3 2-4 2-5
    node0->addEdge(Edge(2));
    node2->addEdge(Edge(0));
    node1->addEdge(Edge(2));
    node2->addEdge(Edge(1));
    node3->addEdge(Edge(2));
    node2->addEdge(Edge(3));
    node4->addEdge(Edge(2));
    node2->addEdge(Edge(4));
    node5->addEdge(Edge(2));
    node2->addEdge(Edge(5));
    
    // Add nodes to TS
    ts->add_Node(node0);
    ts->add_Node(node1);
    ts->add_Node(node2);
    ts->add_Node(node3);
    ts->add_Node(node4);
    ts->add_Node(node5);
    ts->setInitial(0);
    
    cout << "✓ Transition System created" << endl;
    cout << "  - States: " << ts->getNumStates() << endl;
    cout << "  - Initial state: 0" << endl;
    
    // Allocate Environment
    env = new Environment(ts, grid);
    cout << "✓ Environment created" << endl;
    
    // Map states to grid regions
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
}

/**
 * 6 robots, two each of GPS (5), ground movement (0) and camera (3)
 */
MultiRobotSystem* createTestMultiRobotSystem2() {
    MultiRobotSystem* mrs = new MultiRobotSystem();
    RobotCapability caps[] = {RobotCapability::SENSOR_GPS, RobotCapability::MOVEMENT_GROUND, RobotCapability::SENSOR_CAMERA};
    for (uint32_t id = 1; id <= 6; id++) {
         int col = (id - 1) % 3;  // 0-2 horizontal
        int row = (id - 1) / 3;  // 0-4 vertical
        Robot* r = new Robot(id, "Rover_" + to_string(id), Point(180+col, 140+row));
        r->initializeCapabilities(13);
        r->enableCapability(caps[(id - 1) % 3]);
        mrs->addRobot(r);
    }
    return mrs;
}

/**
 * 4 robots with GPS and ground movement only, so any AP needing a camera is infeasible
 */
MultiRobotSystem* createNoCameraMultiRobotSystem() {
    MultiRobotSystem* mrs = new MultiRobotSystem();
    RobotCapability caps[] = {RobotCapability::SENSOR_GPS, RobotCapability::MOVEMENT_GROUND};
    for (uint32_t id = 1; id <= 4; id++) {
        int col = (id - 1) % 3;  // 0-2 horizontal
        int row = (id - 1) / 3;  // 0-1
        Robot* r = new Robot(id, "Rover_" + to_string(id), Point(180+col, 140+row));
        r->initializeCapabilities(13);
        r->enableCapability(caps[(id - 1) % 2]);
        mrs->addRobot(r);
    }
    return mrs;
}

/**
 * Infinite NBA: G F p0 && G F (p1 & X p2)
 */
BuchiAutomaton* createTestBuchiAutomaton() {
    string ltl_str = "G(F(\"p0\")) && G(F(\"p1\" & X (\"p2\")))";
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    return new BuchiAutomaton(ltlFormula);
}

/**
 * infinite NBA: F p1 && F p4 && F p5 && F p3
 */
BuchiAutomaton* createTestBuchiAutomaton2() {
    string ltl_str = "G(F(\"p1\")) && G(F(\"p4\")) && G(F(\"p5\"))";
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p1: needs 0,5
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));   // p4: needs 3,5
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p5: needs 0,5
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    return new BuchiAutomaton(ltlFormula);
}

/**
 * Finite NBA: F p1 && F p4 && F p5 && F p3
 */
BuchiAutomaton* createTestBuchiAutomaton3() {
    string ltl_str = "(F(\"p1\")) && (F(\"p4\")) && (F(\"p5\"))";
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p1: needs 0,5
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));   // p4: needs 3,5
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p5: needs 0,5
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    return new BuchiAutomaton(ltlFormula);
}


/**
 * Infinite NBA whose initial state should be accepting: G F p0
 */
BuchiAutomaton* createInitialAcceptingBuchiAutomaton() {
    string ltl_str = "G(F(\"p0\"))";
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p0: needs 0,5
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    return new BuchiAutomaton(ltlFormula);
}

// ============================================================================
// PLANNER SHOWCASE - prints what the planner produced, asserts nothing
// ============================================================================

void printTaskAllocPath(const string& label, const TaskAllocPath& p) {
    if (p.path.empty()) {
        cout << "      " << label << ": none" << endl;
        return;
    }
    cout << "      " << label << " (" << p.pathLength << " states, makespan " << p.makespan << "): ";
    for (size_t k = 0; k < p.path.size(); k++) {
        if (k) cout << " -> ";
        cout << p.path[k]->getautomatonState()->getId();
    }
    cout << endl;

    // Node 0 is the starting state, so the transitions start at index 1
    for (size_t k = 1; k < p.path.size(); k++) {
        Random_Node* n = p.path[k];
        const vector<uint16_t>& aps = n->getTrueAPs();
        const vector<vector<uint8_t>>& teams = n->getTaskAllocations();
        cout << "        step " << k << " -> state " << n->getautomatonState()->getId() << ": ";
        if (aps.empty()) {
            cout << "no tasks";
        } else {
            for (size_t a = 0; a < aps.size(); a++) {
                if (a) cout << ", ";
                cout << "p" << aps[a] << " <- {";
                if (a < teams.size()) {
                    for (size_t t = 0; t < teams[a].size(); t++) {
                        if (t) cout << " ";
                        cout << "r" << (int)teams[a][t];
                    }
                }
                cout << "}";
            }
        }
        cout << "  (step makespan " << n->getCurmakespan() << ")" << endl;
    }
}

void printPlan(RandomSamplingTaskAllocation& sampler) {
    BuchiAutomaton* pruned = sampler.getPrunedNBA();
    size_t edgeCount = 0;
    for (const auto& nodePair : pruned->getNodes()) edgeCount += nodePair.second->getEdges().size();

    cout << "  ran " << sampler.getIterations() << " iterations in "
         << sampler.getComputationTime() << " s" << endl;
    cout << "  pruned NBA: " << pruned->getNumStates() << " states, " << edgeCount
         << " edges, initial " << pruned->getInitialState() << ", accepting {";
    for (uint16_t a : pruned->getAcceptingStates()) cout << " " << a;
    cout << " }" << endl;

    for (const auto& [acc, plan] : sampler.getAllPaths()) {
        const TaskAllocPath& prefix = plan.first;
        const TaskAllocPath& suffix = plan.second;
        bool planned = !prefix.path.empty() || !suffix.path.empty();
        cout << "\n    accepting state " << acc;
        if (planned) {
            cout << "  (lasso makespan " << (prefix.makespan + suffix.makespan) << ")" << endl;
        } else {
            cout << "  (no plan found)" << endl;
        }
        printTaskAllocPath("prefix", prefix);
        printTaskAllocPath("suffix", suffix);
    }
}

void runPlannerShowcase() {
    cout << "\n\n================================================================================" << endl;
    cout << "PLANNER OUTPUT: 3 specifications x 2 budgets" << endl;
    cout << "================================================================================" << endl;

    struct Spec {
        const char* name;
        const char* formula;
        BuchiAutomaton* (*make)();
    };
    const Spec specs[] = {
        {"Buchi 1", "G F p0 && G F (p1 & X p2)",   createTestBuchiAutomaton},
        {"Buchi 2", "G F p1 && G F p4 && G F p5",  createTestBuchiAutomaton2},
        {"Buchi 3", "F p1 && F p4 && F p5",        createTestBuchiAutomaton3},
    };

    for (const Spec& spec : specs) {
        for (int mode = 0; mode < 2; mode++) {
            TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
            createTestSystemComponents2(ts, grid, env);
            MultiRobotSystem* mrs = createTestMultiRobotSystem2();
            BuchiAutomaton* buchi = spec.make();

            cout << "\n--------------------------------------------------------------------------------" << endl;
            cout << spec.name << ":  " << spec.formula
                 << "   [" << (buchi->isFinite() ? "finite" : "infinite") << ", "
                 << (int)mrs->getNumRobots() << " robots]" << endl;
            cout << "budget: " << (mode == 0 ? "1000 iterations" : "2 seconds") << endl;

            if (mode == 0) {
                //run random sampler
                RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)1000);
                sampler.run();
                printPlan(sampler);
                // Create and run TaskAllocationAlgorithm to compare
                TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
          
                //build the planning decision tree
                allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
                //visualize the optimal path for certain configurations and automata
                    allocAlg->visualizeOptimalPath(string("output/automaton_test_") + spec.formula);
                    allocAlg->visualizeTree(string("output/automaton_test_") + spec.formula + string("_tree"));
                allocAlg->getMetrics().printSummary();
            } else {
                RandomSamplingTaskAllocation sampler(buchi, env, mrs, (double)2.0);
                sampler.run();
                printPlan(sampler);
            }

            cleanup(buchi, mrs, env, ts, grid);
        }
    }
    cout << "\n================================================================================" << endl;
}
