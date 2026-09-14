#include <iostream>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>
#include "../TaskAllocationAlgorithms.h"
#include "../Tree/PlanningDecisionTree.h"
#include "../Tree/Tree_Node.h"
#include "../Environment/Environment.h"
#include "../MultiRobotSystem/MultiRobotSystem.h"
#include "../LTLFormula/LTLFormula.h"
#include "../../Automatons/BuchiAutomaton.h"

using namespace std;
/*
 * ============================================================================
 * LTL FORMULA SYNTAX GUIDE & OPERATOR PRECEDENCE
 * ============================================================================
 *
 * BASIC OPERATORS (in order of precedence, highest to lowest):
 *   1. !    not (highest precedence)
 *   2. &    and
 *   3. |    or
 *   4. ^    xor
 *   5. i    implies
 *   6. e    equivalent (lowest precedence)
 *
 * TEMPORAL OPERATORS (always have lower precedence than logical operators):
 *   X    next (state transitions)
 *   F    eventually
 *   G    globally / always
 *   U    strong until
 *   V    weak release
 *   M    strong release
 *   W    weak until
 *
 * OPERATOR PRECEDENCE RULES:
 *   - Logical operators bind tighter than temporal operators
 *   - Use parentheses for clarity and to override defaults
 *   - F(a & b) ≠ F a & F b
 *   - G(F a) means "infinitely often a"
 *   - F a & F b means "eventually a" AND "eventually b"
 *
 * ============================================================================
 * EXAMPLES FROM OUR TEST SUITE
 * ============================================================================
 *
 * FINITE AUTOMATA (no G, just F - eventuality):
 *
 *   (F"p0" & F"p1")
 *     Eventually visit location p0, and eventually visit location p1.
 *     Once both are satisfied, mission can complete.
 *
 *   (F"p0" & F"p1" & (!\"p2\"U"p0") X "p1")
 *     Eventually p0 and p1, avoid p2 until p0, then next go to p1.
 *     Complex sequence with ordering constraints.
 *
 *   (F"p0" & F"p1" & F"p2" & F"p3")
 *     Visit four locations in any order. Finite - all must eventually occur.
 *
 * INFINITE AUTOMATA (has G - liveness/infinite repetition):
 *
 *   G(F"p0")
 *     Infinitely often visit p0. Must repeatedly return to location p0.
 *     Creates cycles - cannot terminate.
 *
 *   G(F("p0" & "p1"))
 *     Infinitely often satisfy both p0 and p1 together.
 *     Robots must repeatedly visit both locations simultaneously.
 *
 *   G((F"p0") & (F"p1") & (F"p2"))
 *     Always eventually p0, AND always eventually p1, AND always eventually p2.
 *     All three locations must be visited infinitely often (patrol pattern).
 *
 * ============================================================================
 * PRACTICAL TIPS
 * ============================================================================
 *
 * To check if an automaton is finite or infinite:
 *   - No G operator → likely FINITE (eventuality)
 *   - Has G operator → likely INFINITE (liveness requirement)
 *   - isFinite() DFS cycle detection confirms actual behavior
 *
 * For task allocation
 *   - Finite formulas: search completes when all tasks done
 *   - Infinite formulas: search continues, must find repeating path
 *
 * ============================================================================
 */
// ============================================================================
//Create test environment with TS and GridWorld
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(21, 21);
    cout << "✓ GridWorld created (20x20)" << endl;
    
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
    env->mapTSStateToGrid(0, Point(18, 14), 5, 14);    // State 0 centered at (17,15), 4x6 region
    env->mapTSStateToGrid(1, Point(18, 4), 5, 7);   // State 1 centered at (17,4)
    env->mapTSStateToGrid(2, Point(10, 10), 6, 20);   // State 2 centered at (10,11)
    env->mapTSStateToGrid(3, Point(5, 3), 5, 18);   // State 3 centered at (5,5)
    env->mapTSStateToGrid(4, Point(5, 10), 5, 11);   // State 4 centered at (5,10)
    env->mapTSStateToGrid(5, Point(5, 15), 5, 4);   // State 5 centered at (5,15)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem
    mrs = new MultiRobotSystem();
    
    Robot* r1 = new Robot(1, "Rover_1", Point(0, 1));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::SENSOR_GPS); //C
    mrs->addRobot(r1);
    
    Robot* r2 = new Robot(2, "Rover_2", Point(1, 1));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND); //A
    mrs->addRobot(r2);
    
    Robot* r3 = new Robot(3, "Rover_3", Point(2, 1));
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::SENSOR_CAMERA); // B
    mrs->addRobot(r3);
    Robot* r4 = new Robot(4, "Rover_4", Point(0, 1));
    r4->initializeCapabilities(13);
    r4->enableCapability(RobotCapability::SENSOR_GPS); // C
    mrs->addRobot(r4);
    
    Robot* r5 = new Robot(5, "Rover_5", Point(1, 1));
    r5->initializeCapabilities(13);
    r5->enableCapability(RobotCapability::MOVEMENT_GROUND);
    mrs->addRobot(r5);
    
    Robot* r6 = new Robot(6, "Rover_6", Point(2, 1));
    r6->initializeCapabilities(13);
    r6->enableCapability(RobotCapability::SENSOR_CAMERA);
    mrs->addRobot(r6);
    
    cout << "✓ MultiRobotSystem created with 6 robots" << endl;
}

/**
 * Create test Büchi automaton: eventually p1 and eventually p2
 */
BuchiAutomaton* createTestBuchiAutomaton() {
    string ltl_str = "(F\"p1\" && F\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}

/**
 * Create test Büchi automaton: eventually p1 and eventually p4 and eventually p5 and eventually p3
 */
BuchiAutomaton* createTestBuchiAutomaton2() {
    string ltl_str = "(F\"p1\" && F\"p4\" && F\"p5\" && F\"p3\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}
/** 
 * Create test Büchi automaton: eventually p1 and eventually p4 and eventually p5 and eventually p3
 * with different capabilities for each atomic proposition
*/

// ============================================================================
// FINITE AUTOMATA TESTS (4)
// ============================================================================

/**
 * FINITE Test 1: Simple two-step sequential (F p0 && F p1)
 * Linear progression with no cycles
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton1() {
    string ltl_str = "(F\"p0\" & F\"p2\" & (!\"p2\"U\"p0\") X \"p1\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {}))

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_1");
    cout << "✓ Finite Test 1 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * FINITE Test 2: Three sequential tasks (F p0 && F p1 && F p2)
 * Linear progression with different capabilities
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton2() {
    string ltl_str = "(F\"p0\" && F\"p1\" && F\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_2");
    cout << "✓ Finite Test 2 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * FINITE Test 3: Four sequential with repeated capabilities
 * Tests handling of same capability requirement across multiple APs
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton3() {
    string ltl_str = "(F\"p0\" && F\"p1\" && F\"p2\" && F\"p3\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_3");
    cout << "✓ Finite Test 3 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * FINITE Test 4: Repeated AP values across multiple propositions
 * Mixed single and grouped capability requirements
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton4() {
    string ltl_str = "(F\"p0\" && F\"p1\" && F\"p0\")";  // p0 appears twice
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_4");
    cout << "✓ Finite Test 4 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

// ============================================================================
// INFINITE AUTOMATA TESTS (4)
// ============================================================================

/**
 * INFINITE Test 1: Globally always eventually (G F p0)
 * Infinite repetition - must satisfy p0 infinitely often
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1() {
    string ltl_str = "G(F\"p0\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_1");
    cout << "✓ Infinite Test 1 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * INFINITE Test 2: Repeating pattern (G F (p0 && p1))
 * Infinitely often must satisfy both p0 and p1
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton2() {
    string ltl_str = "G(F(\"p0\" && \"p1\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_2");
    cout << "✓ Infinite Test 2 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * INFINITE Test 3: Complex cycle pattern (G(F p0 && F p1))
 * Always eventually p0, and always eventually p1 - both must repeat
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3() {
    string ltl_str = "G((F\"p0\") && (F\"p1\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_3");
    cout << "✓ Infinite Test 3 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * INFINITE Test 4: Three-way cycle with repeated capabilities
 * Globally eventually must see p0, p1, and p2 infinitely often
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4() {
    string ltl_str = "G((F\"p0\") && (F\"p1\") && (F\"p2\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));  // Same as p0
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_4");
    cout << "✓ Infinite Test 4 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    cout << "\n" << string(80, '=') << endl;
    cout << "   FINITE & INFINITE BÜCHI AUTOMATON TEST SUITE" << endl;
    cout << "   Testing with intensiveInterTaskRelationshipTreeSearch" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Create test environment once
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    MultiRobotSystem* mrs = nullptr;
    createTestEnvironment(ts, grid, env, mrs);


    cout << "\n" << string(80, '-') << endl;
    cout << "   FINITE AUTOMATA TESTS (Expected: isFinite() = YES)" << endl;
    cout << string(80, '-') << "\n" << endl;

    // Run 4 Finite Tests with intensiveInterTaskRelationshipTreeSearch
    vector<BuchiAutomaton*> finiteAutomata;
    vector<PlanningDecisionTree*> finiteSearchResults;
    vector<TaskAllocationAlgorithms*> finiteAlgorithms;
    
    BuchiAutomaton* finite1 = createTestFiniteBuchiAutomaton1();
    finiteAutomata.push_back(finite1);
    TaskAllocationAlgorithms* taa1 = new TaskAllocationAlgorithms(finite1, env, mrs);
    finiteAlgorithms.push_back(taa1);
    finiteSearchResults.push_back(taa1->intensiveInterTaskRelationshipTreeSearch(finite1, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        taa1->visualizeTree("output/finite_test_1_tree");
        taa1->visualizeOptimalPath("output/finite_test_1_path");
    }
    
    BuchiAutomaton* finite2 = createTestFiniteBuchiAutomaton2();
    finiteAutomata.push_back(finite2);
    TaskAllocationAlgorithms* taa2 = new TaskAllocationAlgorithms(finite2, env, mrs);
    finiteAlgorithms.push_back(taa2);
    finiteSearchResults.push_back(taa2->intensiveInterTaskRelationshipTreeSearch(finite2, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        taa2->visualizeTree("output/finite_test_2_tree");
        taa2->visualizeOptimalPath("output/finite_test_2_path");
    }
    
    BuchiAutomaton* finite3 = createTestFiniteBuchiAutomaton3();
    finiteAutomata.push_back(finite3);
    TaskAllocationAlgorithms* taa3 = new TaskAllocationAlgorithms(finite3, env, mrs);
    finiteAlgorithms.push_back(taa3);
    finiteSearchResults.push_back(taa3->intensiveInterTaskRelationshipTreeSearch(finite3, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        taa3->visualizeTree("output/finite_test_3_tree");
        taa3->visualizeOptimalPath("output/finite_test_3_path");
    }
    
    BuchiAutomaton* finite4 = createTestFiniteBuchiAutomaton4();
    finiteAutomata.push_back(finite4);
    TaskAllocationAlgorithms* taa4 = new TaskAllocationAlgorithms(finite4, env, mrs);
    finiteAlgorithms.push_back(taa4);
    finiteSearchResults.push_back(taa4->intensiveInterTaskRelationshipTreeSearch(finite4, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        taa4->visualizeTree("output/finite_test_4_tree");
        taa4->visualizeOptimalPath("output/finite_test_4_path");
    }

    cout << "\n" << string(80, '-') << endl;
    cout << "   INFINITE AUTOMATA TESTS (Expected: isFinite() = NO)" << endl;
    cout << string(80, '-') << "\n" << endl;

    // Run 4 Infinite Tests with intensiveInterTaskRelationshipTreeSearch
    vector<BuchiAutomaton*> infiniteAutomata;
    vector<PlanningDecisionTree*> infiniteSearchResults;
    vector<TaskAllocationAlgorithms*> infiniteAlgorithms;
    
    BuchiAutomaton* infinite1 = createTestInfiniteBuchiAutomaton1();
    infiniteAutomata.push_back(infinite1);
    TaskAllocationAlgorithms* taa5 = new TaskAllocationAlgorithms(infinite1, env, mrs);
    infiniteAlgorithms.push_back(taa5);
    infiniteSearchResults.push_back(taa5->intensiveInterTaskRelationshipTreeSearch(infinite1, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        taa5->visualizeTree("output/infinite_test_1_tree");
        taa5->visualizeOptimalPath("output/infinite_test_1_path");
    }
    
    BuchiAutomaton* infinite2 = createTestInfiniteBuchiAutomaton2();
    infiniteAutomata.push_back(infinite2);
    TaskAllocationAlgorithms* taa6 = new TaskAllocationAlgorithms(infinite2, env, mrs);
    infiniteAlgorithms.push_back(taa6);
    infiniteSearchResults.push_back(taa6->intensiveInterTaskRelationshipTreeSearch(infinite2, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        taa6->visualizeTree("output/infinite_test_2_tree");
        taa6->visualizeOptimalPath("output/infinite_test_2_path");
    }
    
    BuchiAutomaton* infinite3 = createTestInfiniteBuchiAutomaton3();
    infiniteAutomata.push_back(infinite3);
    TaskAllocationAlgorithms* taa7 = new TaskAllocationAlgorithms(infinite3, env, mrs);
    infiniteAlgorithms.push_back(taa7);
    infiniteSearchResults.push_back(taa7->intensiveInterTaskRelationshipTreeSearch(infinite3, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        taa7->visualizeTree("output/infinite_test_3_tree");
        taa7->visualizeOptimalPath("output/infinite_test_3_path");
    }
    
    BuchiAutomaton* infinite4 = createTestInfiniteBuchiAutomaton4();
    infiniteAutomata.push_back(infinite4);
    TaskAllocationAlgorithms* taa8 = new TaskAllocationAlgorithms(infinite4, env, mrs);
    infiniteAlgorithms.push_back(taa8);
    infiniteSearchResults.push_back(taa8->intensiveInterTaskRelationshipTreeSearch(infinite4, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        taa8->visualizeTree("output/infinite_test_4_tree");
        taa8->visualizeOptimalPath("output/infinite_test_4_path");
    }

    cout << "\n" << string(80, '=') << endl;
    cout << "   TEST SUMMARY" << endl;
    cout << string(80, '=') << "\n" << endl;

    int finiteSuccess = 0, infiniteSuccess = 0;
    for (auto& tree : finiteSearchResults) {
        if (tree) finiteSuccess++;
    }
    for (auto& tree : infiniteSearchResults) {
        if (tree) infiniteSuccess++;
    }

    cout << "✓ Finite Automata Tests: " << finiteSuccess << "/4 successful" << endl;
    cout << "✓ Infinite Automata Tests: " << infiniteSuccess << "/4 successful" << endl;
    cout << "\nTotal Tests Run: 8" << endl;
    cout << "All tests completed!\n" << endl;

    // Cleanup
    for (auto& buchi : finiteAutomata) {
        delete buchi;
    }
    for (auto& buchi : infiniteAutomata) {
        delete buchi;
    }
    for (auto& tree : finiteSearchResults) {
        if (tree) delete tree;
    }
    for (auto& tree : infiniteSearchResults) {
        if (tree) delete tree;
    }
    for (auto& taa : finiteAlgorithms) {
        delete taa;
    }
    for (auto& taa : infiniteAlgorithms) {
        delete taa;
    }
    delete mrs;
    delete env;
    delete grid;
    delete ts;

    cout << string(80, '=') << "\n" << endl;
    return 0;
}
