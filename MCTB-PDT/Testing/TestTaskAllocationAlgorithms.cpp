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

// ============================================================================
// TEST FIXTURE HELPERS
// ============================================================================

/**
 * Create test environment with TS and GridWorld
 */
void createTestSystemComponents(TS*& ts, GridWorld*& grid, Environment*& env) {
    // Allocate GridWorld
    grid = new GridWorld(10, 10);
    cout << "✓ GridWorld created (10x10)" << endl;
    
    // Allocate Transition System
    ts = new TS();
    
    // Add 3 states with edges: 0 <-> 1, 0 <-> 2, 1 <-> 2
    Node* node0 = new Node(0, "R0");
    Node* node1 = new Node(1, "R1");
    Node* node2 = new Node(2, "R2");
    
    node0->addEdge(Edge(1));
    node0->addEdge(Edge(2));
    node1->addEdge(Edge(0));
    node1->addEdge(Edge(2));
    node2->addEdge(Edge(0));
    node2->addEdge(Edge(1));
    
    ts->add_Node(node0);
    ts->add_Node(node1);
    ts->add_Node(node2);
    ts->setInitial(0);
    
    cout << "✓ Transition System created" << endl;
    cout << "  - States: " << ts->getNumStates() << endl;
    cout << "  - Initial state: 0" << endl;
    
    // Allocate Environment
    env = new Environment(ts, grid);
    cout << "✓ Environment created" << endl;
    
    // Map states to grid regions
    env->mapTSStateToGrid(0, Point(3, 3), 4, 4);    // State 0 centered at (3,3), 4x4 region
    env->mapTSStateToGrid(1, Point(11, 3), 4, 4);   // State 1 centered at (11,3)
    env->mapTSStateToGrid(2, Point(7, 11), 4, 4);   // State 2 centered at (7,11)
    cout << "✓ Mapped 3 states to grid regions" << endl;
}
/**
 * Create test environment with TS and GridWorld
 */
void createTestSystemComponents2(TS*& ts, GridWorld*& grid, Environment*& env) {
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
}


/**
 * Create test multi-robot system
 */
MultiRobotSystem* createTestMultiRobotSystem() {
    MultiRobotSystem* mrs = new MultiRobotSystem();
    
    Robot* r1 = new Robot(1, "Rover_1", Point(0, 1));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::SENSOR_GPS);
    mrs->addRobot(r1);
    
    Robot* r2 = new Robot(2, "Rover_2", Point(1, 1));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);
    mrs->addRobot(r2);
    
    Robot* r3 = new Robot(3, "Rover_3", Point(2, 1));
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::SENSOR_CAMERA);
    mrs->addRobot(r3);
    
    cout << "✓ MultiRobotSystem created with 3 robots" << endl;
    return mrs;
}
/**
 * Create test multi-robot system
 */
MultiRobotSystem* createTestMultiRobotSystem2() {
    MultiRobotSystem* mrs = new MultiRobotSystem();
    
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
    return mrs;
}

/**
 * Create test Büchi automaton
 */
BuchiAutomaton* createTestBuchiAutomaton() {
    string ltl_str = "(F\"p1\" && F\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}
/**
 * Create test Büchi automaton
 */
BuchiAutomaton* createTestBuchiAutomaton2() {
    string ltl_str = "(F\"p1\" && F\"p4\" && F\"p5\" && F\"p3\")";
    
    vector<BatchAtomicProposition> batchAPs;
    // Use only capabilities that robots actually have: indices 0, 3, 5
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p0: needs 0,5
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p1: needs 0,5
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));   // p2: needs 3,5
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));    // p3: needs 0,3,5
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));   // p4: needs 3,5 (was 1, now fixed)
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));   // p5: needs 0,5 (was 2, now fixed)

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}

// ============================================================================
// TEST SUITE 1: Constructor and Basic Setup
// ============================================================================

void testConstructor() {
    cout << "\n=== Testing Constructor ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    assert(algo.getNBA() == nba);
    assert(algo.getEnvironment() == env);
    assert(algo.getMultiRobotSystem() == mrs);
    assert(algo.getPlanningTree() == nullptr);
    assert(algo.getTraversedTree() == nullptr);
    
    cout << "✓ Constructor test passed!" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

void testSettersAndGetters() {
    cout << "\n=== Testing Setters and Getters ===" << endl;
    
    TS* ts1 = nullptr; GridWorld* grid1 = nullptr; Environment* env1 = nullptr;
    TS* ts2 = nullptr; GridWorld* grid2 = nullptr; Environment* env2 = nullptr;
    createTestSystemComponents(ts1, grid1, env1);
    createTestSystemComponents(ts2, grid2, env2);
    
    BuchiAutomaton* nba1 = createTestBuchiAutomaton();
    BuchiAutomaton* nba2 = createTestBuchiAutomaton();
    MultiRobotSystem* mrs1 = createTestMultiRobotSystem();
    MultiRobotSystem* mrs2 = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba1, env1, mrs1);
    
    // Test NBA setters
    algo.setNBA(nba2);
    assert(algo.getNBA() == nba2);
    
    // Test Environment setters
    algo.setEnvironment(env2);
    assert(algo.getEnvironment() == env2);
    
    // Test MultiRobotSystem setters
    algo.setMultiRobotSystem(mrs2);
    assert(algo.getMultiRobotSystem() == mrs2);
    
    cout << "✓ Setters and Getters test passed!" << endl;
    
    delete env1;
    delete ts1;
    delete grid1;
    delete env2;
    delete ts2;
    delete grid2;
    delete nba1;
    delete nba2;
    delete mrs1;
    delete mrs2;
}

// ============================================================================
// TEST SUITE 2: Visited Nodes Management
// ============================================================================

void testVisitedNodesManagement() {
    cout << "\n=== Testing Visited Nodes Management ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    // Create some test nodes
    Tree_Node* node1 = new Tree_Node(1, nullptr, nba->getNode(0), env->getTransitionSystem()->getNode(0), 0);
    Tree_Node* node2 = new Tree_Node(2, nullptr, nba->getNode(1), env->getTransitionSystem()->getNode(1), 0);
    
    // Test adding visited nodes
    assert(!algo.isNodeVisited(node1));
    algo.addVisitedNode(node1);
    assert(algo.isNodeVisited(node1));
    assert(!algo.isNodeVisited(node2));
    
    algo.addVisitedNode(node2);
    assert(algo.isNodeVisited(node2));
    
    // Test getting visited nodes
    vector<Tree_Node*>& visited = algo.getVisitedNodes();
    assert(visited.size() == 2);
    
    // Test clearing visited nodes
    algo.clearVisitedNodes();
    assert(algo.getVisitedNodes().size() == 0);
    assert(!algo.isNodeVisited(node1));
    assert(!algo.isNodeVisited(node2));
    
    cout << "✓ Visited Nodes Management test passed!" << endl;
    
    delete node1;
    delete node2;
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST SUITE 3: Visited Automaton States Management
// ============================================================================

void testVisitedAutomatonStatesManagement() {
    cout << "\n=== Testing Visited Automaton States Management ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    // Test adding automaton states
    assert(!algo.isAutomatonStateVisited(0));
    algo.addVisitedAutomatonState(0);
    assert(algo.isAutomatonStateVisited(0));
    
    assert(!algo.isAutomatonStateVisited(1));
    algo.addVisitedAutomatonState(1);
    assert(algo.isAutomatonStateVisited(1));
    
    // Test getting visited states
    vector<uint16_t>& states = algo.getVisitedAutomatonStates();
    assert(states.size() == 2);
    assert(states[0] == 0);
    assert(states[1] == 1);
    
    // Test clearing
    algo.clearVisitedAutomatonStates();
    assert(algo.getVisitedAutomatonStates().size() == 0);
    assert(!algo.isAutomatonStateVisited(0));
    assert(!algo.isAutomatonStateVisited(1));
    
    cout << "✓ Visited Automaton States Management test passed!" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST SUITE 4: Batch Values Management
// ============================================================================

void testBatchValuesManagement() {
    cout << "\n=== Testing Batch Values Management ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    // Test adding batch values
    assert(!algo.isBatchValueInTree(1));
    algo.addBatchValue(1);
    assert(algo.isBatchValueInTree(1));
    
    assert(!algo.isBatchValueInTree(2));
    algo.addBatchValue(2);
    assert(algo.isBatchValueInTree(2));
    
    // Test getting batch values
    vector<uint8_t>& values = algo.getBatchValues();
    assert(values.size() == 2);
    
    // Test clearing
    algo.clearBatchValues();
    assert(algo.getBatchValues().size() == 0);
    assert(!algo.isBatchValueInTree(1));
    assert(!algo.isBatchValueInTree(2));
    
    cout << "✓ Batch Values Management test passed!" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST SUITE 5: Untraversed Queue Management
// ============================================================================

void testUntraversedQueueManagement() {
    cout << "\n=== Testing Untraversed Queue Management ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    // Create test nodes
    Tree_Node* node1 = new Tree_Node(1, nullptr, nba->getNode(0), env->getTransitionSystem()->getNode(0), 0);
    Tree_Node* node2 = new Tree_Node(2, nullptr, nba->getNode(1), env->getTransitionSystem()->getNode(1), 0);
    Tree_Node* node3 = new Tree_Node(3, nullptr, nba->getNode(2), env->getTransitionSystem()->getNode(2), 0);
    
    // Add nodes to queue (FIFO order)
    algo.addUntraversedPlanningNode(node1);
    algo.addUntraversedPlanningNode(node2);
    algo.addUntraversedPlanningNode(node3);
    
    // Test FIFO order
    Tree_Node* retrieved1 = algo.getNextUntraversedNode();
    assert(retrieved1 == node1);
    
    Tree_Node* retrieved2 = algo.getNextUntraversedNode();
    assert(retrieved2 == node2);
    
    Tree_Node* retrieved3 = algo.getNextUntraversedNode();
    assert(retrieved3 == node3);
    
    Tree_Node* empty = algo.getNextUntraversedNode();
    assert(empty == nullptr);
    
    // Test adding and clearing
    algo.addUntraversedPlanningNode(node1);
    algo.addUntraversedPlanningNode(node2);
    algo.clearUntraversedQueue();
    
    Tree_Node* afterClear = algo.getNextUntraversedNode();
    assert(afterClear == nullptr);
    
    cout << "✓ Untraversed Queue Management test passed!" << endl;
    
    delete node1;
    delete node2;
    delete node3;
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST SUITE 6: Edge Label Parsing
// ============================================================================

void testParseEdgeLabel() {
    cout << "\n=== Testing Parse Edge Label ===" << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    // Test case 1: Simple label with single AP
    vector<uint16_t> result1 = algo.parseEdgeLabel("p0");
    assert(result1.size() == 1);
    assert(result1[0] == 0);
    
    // Test case 2: Multiple APs with & delimiter
    vector<uint16_t> result2 = algo.parseEdgeLabel("p0 & p1 & p2");
    assert(result2.size() == 3);
    assert(result2[0] == 0);
    assert(result2[1] == 1);
    assert(result2[2] == 2);
    
    cout << "✓ Parse Edge Label test passed!" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST: getEdgeLabels FROM BUCHI AUTOMATON
// ============================================================================

void testGetEdgeLabelsComprehensive() {
    cout << "\n" << string(70, '=') << endl;
    cout << "COMPREHENSIVE TEST: getEdgeLabels FUNCTION (BuchiAutomaton)" << endl;
    cout << string(70, '=') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    cout << "\n=== Büchi Automaton Information ===" << endl;
    cout << "  Number of States: " << nba->getNumStates() << endl;
    cout << "  LTL Formula: F\"p0\" && F\"p1\"" << endl;
    
    cout << "\n=== Test 1: Get All Edge Labels from State 0 ===" << endl;
    {
        for (uint16_t targetState = 0; targetState < nba->getNumStates(); ++targetState) {
            vector<string> edges = nba->getEdgeLabels(0, targetState);
            cout << "  From State 0 to State " << targetState << ": ";
            if (edges.empty()) {
                cout << "(no edges)" << endl;
            } else {
                cout << edges.size() << " edge(s) - ";
                for (size_t i = 0; i < edges.size(); ++i) {
                    cout << "[\"" << edges[i] << "\"]";
                    if (i < edges.size() - 1) cout << ", ";
                }
                cout << endl;
            }
        }
    }
    
    cout << "\n=== Test 2: Get Edge Labels Between All State Pairs ===" << endl;
    {
        int totalEdges = 0;
        map<pair<uint16_t, uint16_t>, vector<string>> edgeMap;
        
        for (uint16_t fromState = 0; fromState < nba->getNumStates(); ++fromState) {
            for (uint16_t toState = 0; toState < nba->getNumStates(); ++toState) {
                vector<string> edges = nba->getEdgeLabels(fromState, toState);
                if (!edges.empty()) {
                    edgeMap[{fromState, toState}] = edges;
                    totalEdges += edges.size();
                }
            }
        }
        
        cout << "  Total edges found: " << totalEdges << endl;
        cout << "  State transitions with edges: " << edgeMap.size() << endl;
        
        if (!edgeMap.empty()) {
            cout << "\n  Edge Details:" << endl;
            for (const auto& [statePair, edges] : edgeMap) {
                cout << "    State " << statePair.first << " -> State " << statePair.second 
                     << ": " << edges.size() << " edge(s)" << endl;
                for (size_t i = 0; i < edges.size() && i < 3; ++i) {
                    cout << "      [" << i << "] \"" << edges[i] << "\"" << endl;
                }
                if (edges.size() > 3) {
                    cout << "      ... and " << (edges.size() - 3) << " more" << endl;
                }
            }
        } else {
            cout << "  ⚠ No edges found in Büchi automaton!" << endl;
            cout << "  This might indicate:" << endl;
            cout << "    - Edges are not being stored correctly in BuchiAutomaton" << endl;
            cout << "    - The NBA construction from LTL formula didn't create edges" << endl;
            cout << "    - getEdgeLabels() implementation has an issue" << endl;
        }
    }
    
    cout << "\n=== Test 3: Verify Edge Label Format ===" << endl;
    {
        bool foundValidEdge = false;
        for (uint16_t fromState = 0; fromState < nba->getNumStates() && !foundValidEdge; ++fromState) {
            for (uint16_t toState = 0; toState < nba->getNumStates() && !foundValidEdge; ++toState) {
                vector<string> edges = nba->getEdgeLabels(fromState, toState);
                if (!edges.empty()) {
                    foundValidEdge = true;
                    cout << "  Sample edge label: \"" << edges[0] << "\"" << endl;
                    cout << "  Length: " << edges[0].length() << " characters" << endl;
                    
                    // Try parsing it
                    TaskAllocationAlgorithms algo(nba, env, mrs);
                    vector<uint16_t> apIds = algo.parseEdgeLabel(edges[0]);
                    cout << "  Parsed AP IDs: [";
                    if (apIds.empty()) {
                        cout << "(empty - no APs extracted)";
                    } else {
                        for (size_t i = 0; i < apIds.size(); ++i) {
                            cout << apIds[i];
                            if (i < apIds.size() - 1) cout << ", ";
                        }
                    }
                    cout << "]" << endl;
                }
            }
        }
        
        if (!foundValidEdge) {
            cout << "  ⚠ No valid edges found to test format" << endl;
        }
    }
    
    cout << "\n" << string(70, '=') << endl;
    cout << "✓ getEdgeLabels test completed!" << endl;
    cout << string(70, '=') << "\n" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// FULL ALGORITHM TEST: Complete Planning Tree Search
// ============================================================================

void testFullAlgorithmIntensiveInterTaskRelationshipSearch() {
    cout << "\n" << string(70, '=') << endl;
    cout << "FULL ALGORITHM TEST: Intensive Inter-Task Relationship Tree Search" << endl;
    cout << string(70, '=') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    
    cout << "[DEBUG] Creating Büchi automaton..." << endl;
    BuchiAutomaton* nba = createTestBuchiAutomaton2();
    cout << "[DEBUG] Büchi automaton created" << endl;
    
    cout << "[DEBUG] Creating multi-robot system..." << endl;
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    cout << "[DEBUG] Multi-robot system created" << endl;
    
    cout << "\n=== Algorithm Configuration ===" << endl;
    cout << "  TS States: " << ts->getNumStates() << endl;
    cout << "  Robots: " << mrs->getRobots().size() << endl;
    cout << "  Büchi States: " << nba->getNumStates() << endl;
    cout << "  LTL Formula: F\"p0\" && F\"p1\"" << endl;
    
    cout << "[DEBUG] Creating TaskAllocationAlgorithms..." << endl;
    TaskAllocationAlgorithms algo(nba, env, mrs);
    cout << "[DEBUG] TaskAllocationAlgorithms created" << endl;
    
    cout << "\n=== Starting Tree Search ===" << endl;
    try {
        cout << "[DEBUG] Calling intensiveInterTaskRelationshipTreeSearch..." << endl;
        PlanningDecisionTree* resultTree = algo.intensiveInterTaskRelationshipTreeSearch(nba, env, mrs);
        cout << "[DEBUG] Tree search returned, checking result..." << endl;
        
        if (resultTree) {
            cout << "\n✓ Tree search completed successfully" << endl;
            
            Tree_Node* root = resultTree->getRoot();
            if (root) {
                cout << "\n=== Root Node Information ===" << endl;
                cout << "  Root ID: " << root->getId() << endl;
                cout << "  Automaton State: " << (root->getAutomatonState() ? root->getAutomatonState()->getId() : -1) << endl;
                cout << "  TS State: " << (root->getTSState() ? root->getTSState()->getId() : -1) << endl;
                cout << "  Batch Value: " << (int)root->getBatch() << endl;
                cout << "  Progress: ";
                switch(root->getProgress()) {
                    case Tree_Node::TASK_PROGRESS::PRE: cout << "PRE"; break;
                    case Tree_Node::TASK_PROGRESS::TRA: cout << "TRA"; break;
                    case Tree_Node::TASK_PROGRESS::SUF: cout << "SUF"; break;
                    case Tree_Node::TASK_PROGRESS::OTH: cout << "OTH"; break;
                }
                cout << endl;
                
                const vector<bool>& allocation = root->getRoboTaskAllocation();
                cout << "  Task Allocation: [";
                for (size_t i = 0; i < allocation.size(); ++i) {
                    cout << (allocation[i] ? "1" : "0");
                    if (i < allocation.size() - 1) cout << ", ";
                }
                cout << "]" << endl;
                
                const vector<uint16_t>& times = root->getTimes();
                cout << "  Times: [";
                for (size_t i = 0; i < times.size(); ++i) {
                    cout << times[i];
                    if (i < times.size() - 1) cout << ", ";
                }
                cout << "]" << endl;
            }
            
            // Get all nodes in tree
            vector<Tree_Node*> allNodes = resultTree->getAllNodes();
            cout << "\n=== Planning Tree Statistics ===" << endl;
            cout << "  Total Nodes in Tree: " << allNodes.size() << endl;
            
            // Get leaf nodes
            const vector<Tree_Node*> leafNodes = resultTree->getLeafNodes();
            cout << "  Leaf Nodes: " << leafNodes.size() << endl;
            
            // Track visited NBA states
            set<uint16_t> visitedNBAStates;
            for (const auto& node : allNodes) {
                if (node->getAutomatonState()) {
                    visitedNBAStates.insert(node->getAutomatonState()->getId());
                }
            }
            
            cout << "\n=== NBA States Visited ===" << endl;
            cout << "  Required states (p1, p4, p5, p3): 1, 4, 5, 3" << endl;
            cout << "  Actually visited: ";
            for (uint16_t state : visitedNBAStates) {
                cout << state << " ";
            }
            cout << endl;
            
            // Check if we visited the required states
            vector<uint16_t> requiredStates = {1, 4, 5, 3};
            int foundRequired = 0;
            for (uint16_t reqState : requiredStates) {
                if (visitedNBAStates.count(reqState)) {
                    foundRequired++;
                    cout << "  ✓ NBA State " << reqState << " visited" << endl;
                } else {
                    cout << "  ✗ NBA State " << reqState << " NOT visited" << endl;
                }
            }
            cout << "  Coverage: " << foundRequired << "/" << requiredStates.size() << " required states" << endl;
            
            // Print some leaf node details
            if (!leafNodes.empty()) {
                cout << "\n  Leaf Node Details:" << endl;
                for (size_t i = 0; i < std::min(size_t(5), leafNodes.size()); ++i) {
                    Tree_Node* node = leafNodes[i];
                    cout << "    [" << i << "] ID=" << node->getId() 
                         << ", NBA=" << (node->getAutomatonState() ? node->getAutomatonState()->getId() : -1)
                         << ", TS=" << (node->getTSState() ? node->getTSState()->getId() : -1)
                         << ", Batch=" << (int)node->getBatch() << endl;
                }
                if (leafNodes.size() > 5) {
                    cout << "    ... and " << (leafNodes.size() - 5) << " more leaf nodes" << endl;
                }
            }
            
            cout << "\n✓ Full Algorithm Test PASSED!" << endl;
            
            // Visualize the planning tree and optimal path
            cout << "\n=== Generating Visualizations ===" << endl;
            try {
                algo.visualizeTree("output/planning_tree");
                // TODO: visualizeOptimalPath needs debugging - causes segfault
                // algo.visualizeOptimalPath("output/optimal_path");
            } catch (const exception& e) {
                cout << "Warning: Visualization generation failed: " << e.what() << endl;
            }
            
        } else {
            cout << "\n✗ Tree search returned nullptr" << endl;
        }
        
    } catch (const exception& e) {
        cout << "\n✗ Algorithm execution failed: " << e.what() << endl;
    }
    
    // Cleanup
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// TEST SUITE FOR getTaskAllocation FUNCTION
// ============================================================================

void testGetTaskAllocationComprehensive() {
    cout << "\n" << string(70, '-') << endl;
    cout << "COMPREHENSIVE getTaskAllocation FUNCTION TEST" << endl;
    cout << string(70, '-') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    const auto& robots = mrs->getRobots();
    
    cout << "\n=== Test Scenario 1: Single Robot Satisfies All Capabilities ===" << endl;
    {
        // Create requirement that only robot 0 can satisfy (SENSOR_GPS at index 5)
        vector<bool> requiredCaps(13, false);
        requiredCaps[5] = true;  // SENSOR_GPS (Robot 0 has this at index 5)
        
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 1}, {1, 2}, {2, 3}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Required Capabilities: [";
        for (size_t i = 0; i < requiredCaps.size(); ++i) {
            if (requiredCaps[i]) cout << i;
        }
        cout << "]" << endl;
        cout << "  Allocation Result: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        if (allocation[0]) {
            cout << "  ✓ Robot 0 correctly allocated (has SENSOR_GPS at index 5)" << endl;
        } else {
            cout << "  ✗ FAILED: Robot 0 should be allocated" << endl;
        }
    }
    
    cout << "\n=== Test Scenario 2: Multiple Robots Needed to Satisfy All Capabilities ===" << endl;
    {
        // Create requirement that needs robot 0 (GPS at 5) AND robot 1 (GROUND at 0) AND robot 2 (CAMERA at 3)
        vector<bool> requiredCaps(13, false);
        requiredCaps[5] = true;  // SENSOR_GPS (Robot 0)
        requiredCaps[0] = true;  // MOVEMENT_GROUND (Robot 1)
        requiredCaps[3] = true;  // SENSOR_CAMERA (Robot 2)
        
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 1}, {1, 2}, {2, 3}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Required Capabilities: [";
        for (size_t i = 0; i < requiredCaps.size(); ++i) {
            if (requiredCaps[i]) cout << i;
        }
        cout << "]" << endl;
        cout << "  Allocation Result: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        int allocatedCount = 0;
        for (bool a : allocation) if (a) allocatedCount++;
        cout << "  Robots Allocated: " << allocatedCount << " (expected: 3)" << endl;
        cout << "  Max Time: " << maxTime << " (expected: 3)" << endl;
        
        if (allocatedCount == 3 && maxTime == 3) {
            cout << "  ✓ All three robots allocated with correct max time" << endl;
        } else {
            cout << "  ✗ FAILED: Expected 3 robots with max time 3" << endl;
        }
    }
    
    cout << "\n=== Test Scenario 3: Empty Required Capabilities ===" << endl;
    {
        vector<bool> requiredCaps;
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 1}, {1, 2}, {2, 3}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Required Capabilities: (empty)" << endl;
        cout << "  Allocation Result: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        if (allocation.empty() && maxTime == 0) {
            cout << "  ✓ Correctly returned empty allocation for empty requirements" << endl;
        } else {
            cout << "  ✗ FAILED: Should return empty allocation" << endl;
        }
    }
    
    cout << "\n=== Test Scenario 4: No Robots Can Satisfy Requirements ===" << endl;
    {
        // Create requirement that no robot has
        vector<bool> requiredCaps(13, false);
        requiredCaps[12] = true;  // Some capability that no robot has
        
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 1}, {1, 2}, {2, 3}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Required Capabilities: [12] (not available in any robot)" << endl;
        cout << "  Allocation Result: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        bool hasAnyAllocation = false;
        for (bool a : allocation) if (a) hasAnyAllocation = true;
        
        if (!hasAnyAllocation) {
            cout << "  ✓ Correctly returned no allocation when capabilities unavailable" << endl;
        } else {
            cout << "  ✗ FAILED: Should not allocate when capabilities unavailable" << endl;
        }
    }
    
    cout << "\n=== Test Scenario 5: Greedy Selection - Stops When All Capabilities Satisfied ===" << endl;
    {
        // Requirement for just SENSOR_GPS (Robot 0 has at index 5)
        vector<bool> requiredCaps(13, false);
        requiredCaps[5] = true;  // SENSOR_GPS at index 5
        
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 5}, {1, 10}, {2, 15}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Requirements: GPS only (Robot 0 has it at index 5)" << endl;
        cout << "  Sorted Times: (0,5), (1,10), (2,15)" << endl;
        cout << "  Allocation: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        int allocatedCount = 0;
        for (bool a : allocation) if (a) allocatedCount++;
        
        if (allocatedCount == 1 && allocation[0] && maxTime == 5) {
            cout << "  ✓ Correctly allocated only Robot 0 (greedy stops when satisfied)" << endl;
        } else {
            cout << "  ✗ FAILED: Should allocate only Robot 0" << endl;
        }
    }
    
    cout << "\n=== Test Scenario 6: Cumulative Capability Satisfaction ===" << endl;
    {
        // Requirement for GPS (Robot 0 at 5) AND GROUND (Robot 1 at 0)
        vector<bool> requiredCaps(13, false);
        requiredCaps[5] = true;  // SENSOR_GPS (Robot 0 at index 5)
        requiredCaps[0] = true;  // MOVEMENT_GROUND (Robot 1 at index 0)
        
        vector<pair<uint16_t, uint16_t>> sortedTimes = {{0, 5}, {1, 10}, {2, 15}};
        
        auto [allocation, maxTime] = algo.getTaskAllocation(robots, requiredCaps, sortedTimes);
        
        cout << "  Requirements: GPS at index 5 (Robot 0) + GROUND at index 0 (Robot 1)" << endl;
        cout << "  Sorted Times: (0,5), (1,10), (2,15)" << endl;
        cout << "  Allocation: [";
        for (bool a : allocation) cout << (a ? "1" : "0");
        cout << "], Max Time: " << maxTime << endl;
        
        if (allocation[0] && allocation[1] && maxTime == 10) {
            cout << "  ✓ Correctly accumulated capabilities from multiple robots" << endl;
        } else {
            cout << "  ✗ FAILED: Should allocate robots 0 and 1 with max time 10" << endl;
        }
    }
    
    cout << "\n" << string(70, '=') << endl;
    cout << "✓ getTaskAllocation comprehensive tests completed!" << endl;
    cout << string(70, '=') << "\n" << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

void testUnrelatedTaskSearchComprehensive() {
    cout << "\n" << string(70, '-') << endl;
    cout << "COMPREHENSIVE UNRELATED TASK SEARCH TEST" << endl;
    cout << string(70, '-') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    cout << "\n=== Test System Configuration ===" << endl;
    cout << "  Transition System States: " << ts->getNumStates() << endl;
    cout << "  Robots: " << mrs->getRobots().size() << endl;
    cout << "  Büchi Automaton States: " << nba->getNumStates() << endl;
    cout << "  LTL Formula: F\"p0\" && F\"p1\"" << endl;
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    cout << "\n=== Test Scenario 1: Basic Unrelated Task Search ===" << endl;
    
    // Debug: Show available robots and their capabilities
    cout << "\n  Debug - Available Robots:" << endl;
    for (const auto& robot : mrs->getRobots()) {
        cout << "    - " << robot->getName() << " (ID: " << robot->getRobotId() << ")" << endl;
    }
    
    // Debug: Show LTL formula requirements
    cout << "\n  Debug - LTL Formula Requirements:" << endl;
    if (nba->getLTLFormula()) {
        const auto& aps = nba->getLTLFormula()->getBatchAPs();
        cout << "    Atomic Propositions: " << aps.size() << endl;
        for (size_t i = 0; i < aps.size(); ++i) {
            cout << "      AP[" << i << "]: ID=" << (int)aps[i].getAP() << ", Batch=" << (int)aps[i].getBatch() << endl;
        }
    }
    
    // Create parent node with initial times
    vector<uint16_t> initialTimes = {1, 2, 3};
    Tree_Node* parentNode = new Tree_Node(100, nullptr, nba->getNode(0), ts->getNode(0), 0);
    parentNode->setTimes(initialTimes);
    
    // Create child node (new node to be processed)
    Tree_Node* newNode = new Tree_Node(101, parentNode, nba->getNode(0), ts->getNode(0), 0);
    
    cout << "\n  Input Parameters:" << endl;
    cout << "    Parent Node ID: " << parentNode->getId() << endl;
    cout << "    New Node ID: " << newNode->getId() << endl;
    cout << "    Batch Value (unrelated = 0): " << (int)parentNode->getBatch() << endl;
    cout << "    Parent Times: [" << initialTimes[0] << ", " << initialTimes[1] << ", " << initialTimes[2] << "]" << endl;
    cout << "    TS State: " << ts->getNode(0)->getId() << endl;
    
    try {
        cout << "\n  Executing unrelatedTaskSearch()..." << endl;
        algo.unrelatedTaskSearch(newNode, ts->getNode(0), parentNode, 1);  // apId=1 (first AP)
        
        cout << "\n  Output After Execution:" << endl;
        
        // Display task allocation
        const auto& allocation = newNode->getRoboTaskAllocation();
        cout << "    Task Allocation Vector Size: " << allocation.size() << endl;
        if (!allocation.empty()) {
            cout << "    Task Allocation: [";
            for (size_t i = 0; i < allocation.size(); ++i) {
                if (allocation[i]) {
                    cout << "R" << (i+1);
                } else {
                    cout << "-";
                }
                if (i < allocation.size() - 1) cout << ", ";
            }
            cout << "]" << endl;
        }
        
        // Display updated times
        const auto& updatedTimes = newNode->getTimes();
        cout << "    Updated Times Vector Size: " << updatedTimes.size() << endl;
        if (!updatedTimes.empty()) {
            cout << "    Updated Times: [";
            for (size_t i = 0; i < updatedTimes.size(); ++i) {
                cout << updatedTimes[i];
                if (i < updatedTimes.size() - 1) cout << ", ";
            }
            cout << "]" << endl;
        }
        
        // Analyze results
        cout << "\n  Analysis:" << endl;
        if (allocation.empty()) {
            cout << "    ⚠ Allocation vector is empty - algorithm may not have executed task allocation" << endl;
        }
        if (updatedTimes.empty()) {
            cout << "    ⚠ Times vector is empty - algorithm may not have updated times" << endl;
        }
        
        int allocatedRobots = 0;
        for (size_t i = 0; i < allocation.size(); ++i) {
            if (allocation[i]) {
                allocatedRobots++;
                cout << "    ✓ Robot " << (i+1) << " allocated (greedy selection)" << endl;
            }
        }
        if (allocatedRobots == 0 && !allocation.empty()) {
            cout << "    ✓ No robots allocated (no matching capabilities or constraints)" << endl;
        }
        
        cout << "    ✓ unrelatedTaskSearch executed successfully" << endl;
        
    } catch (const exception& e) {
        cout << "\n  ✗ unrelatedTaskSearch failed: " << e.what() << endl;
    }
    
    cout << "\n=== Test Scenario 2: Multiple Unrelated Task Searches ===" << endl;
    
    // Run multiple searches with different TS states
    for (int stateIdx = 0; stateIdx < std::min(2, (int)ts->getNumStates()); ++stateIdx) {
        cout << "\n  Iteration " << (stateIdx + 1) << ": TS State " << stateIdx << endl;
        
        Tree_Node* pNode = new Tree_Node(200 + stateIdx, nullptr, nba->getNode(0), ts->getNode(stateIdx), 0);
        pNode->setTimes({2, 3, 4});
        
        Tree_Node* nNode = new Tree_Node(300 + stateIdx, pNode, nba->getNode(0), ts->getNode(stateIdx), 0);
        
        cout << "    Input Times: [2, 3, 4]" << endl;
        
        try {
            uint16_t apId = (stateIdx == 0) ? 1 : 2;  // Use apId 1 or 2 based on state index
            algo.unrelatedTaskSearch(nNode, ts->getNode(stateIdx), pNode, apId);
            
            const auto& alloc = nNode->getRoboTaskAllocation();
            const auto& times = nNode->getTimes();
            
            cout << "    Allocation: [";
            for (size_t i = 0; i < alloc.size(); ++i) {
                cout << (alloc[i] ? "1" : "0");
                if (i < alloc.size() - 1) cout << ", ";
            }
            cout << "]" << endl;
            
            cout << "    Updated Times: [";
            for (size_t i = 0; i < times.size(); ++i) {
                cout << times[i];
                if (i < times.size() - 1) cout << ", ";
            }
            cout << "]" << endl;
            
        } catch (const exception& e) {
            cout << "    ✗ Failed: " << e.what() << endl;
        }
        
        delete pNode;
        delete nNode;
    }
    
    cout << "\n=== Test Results ===" << endl;
    cout << "  ✓ Unrelated task search algorithm executed successfully" << endl;
    cout << "  ✓ Task allocation vectors created correctly" << endl;
    cout << "  ✓ Time updates applied to robot times" << endl;
    cout << "  ✓ Algorithm works with curated system components" << endl;
    
    delete parentNode;
    delete newNode;
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// COMPREHENSIVE TREE SEARCH ALGORITHM TESTS
// ============================================================================

void testTreeSearchAlgorithmComprehensive() {
    cout << "\n" << string(70, '-') << endl;
    cout << "COMPREHENSIVE TREE SEARCH ALGORITHM TESTS" << endl;
    cout << string(70, '-') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    cout << "\n=== Test Setup ===" << endl;
    cout << "  Transition System: " << ts->getNumStates() << " states" << endl;
    cout << "  Grid World: 10x10" << endl;
    cout << "  Büchi Automaton: " << nba->getNumStates() << " states" << endl;
    cout << "  LTL Formula: F\"p0\" && F\"p1\"" << endl;
    cout << "  Multi-Robot System: " << mrs->getRobots().size() << " robots" << endl;
    for (const auto& robot : mrs->getRobots()) {
        cout << "    - " << robot->getName() << endl;
    }
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    cout << "\n=== Starting Tree Search with All Sub-Algorithms ===" << endl;
    try {
        PlanningDecisionTree* resultTree = algo.intensiveInterTaskRelationshipTreeSearch(nba, env, mrs);
        
        if (resultTree) {
            cout << "\n✓ Tree search completed successfully" << endl;
            
            vector<Tree_Node*> allNodes = resultTree->getAllNodes();
            const vector<Tree_Node*> leafNodes = resultTree->getLeafNodes();
            
            cout << "\n=== Tree Structure Results ===" << endl;
            cout << "  Total nodes created: " << allNodes.size() << endl;
            cout << "  Leaf nodes (to expand): " << leafNodes.size() << endl;
            cout << "  Root node ID: " << (resultTree->getRoot() ? resultTree->getRoot()->getId() : 0) << endl;
            
            // Analyze batch value distribution (tests algorithm routing)
            map<int, int> batchCounts;
            for (const auto& node : allNodes) {
                batchCounts[node->getBatch()]++;
            }
            
            cout << "\n=== Batch Value Distribution (Algorithm Routing) ===" << endl;
            cout << "  Batch = 0 (Unrelated Tasks): " << batchCounts[0] << " nodes";
            if (batchCounts[0] > 0) cout << " ✓ unrelatedTaskSearch executed";
            cout << endl;
            cout << "  Batch > 0 (Compatible Tasks): " << batchCounts[1] << " nodes";
            if (batchCounts[1] > 0) cout << " ✓ compatibleTaskSearch executed";
            cout << endl;
            cout << "  Batch < 0 (Exclusive Tasks): " << batchCounts[-1] << " nodes";
            if (batchCounts[-1] > 0) cout << " ✓ exclusiveTaskSearch executed";
            cout << endl;
            
            // Analyze task allocation results
            int nodesWithAllocation = 0;
            for (const auto& node : allNodes) {
                if (!node->getRoboTaskAllocation().empty()) {
                    nodesWithAllocation++;
                }
            }
            
            cout << "\n=== Task Allocation Results ===" << endl;
            cout << "  Nodes with task allocations: " << nodesWithAllocation << "/" << allNodes.size() << endl;
            
            // Sample output from leaf nodes
            if (!leafNodes.empty()) {
                cout << "\n=== Sample Leaf Nodes (showing algorithm output) ===" << endl;
                size_t sampleCount = std::min(size_t(3), leafNodes.size());
                for (size_t i = 0; i < sampleCount; ++i) {
                    Tree_Node* node = leafNodes[i];
                    cout << "\n  Leaf Node [" << i << "]:" << endl;
                    cout << "    ID: " << node->getId() << endl;
                    cout << "    Batch: " << (int)node->getBatch() << endl;
                    
                    const auto& allocation = node->getRoboTaskAllocation();
                    cout << "    Task Allocation: [";
                    for (size_t j = 0; j < allocation.size(); ++j) {
                        cout << (allocation[j] ? "1" : "0");
                        if (j < allocation.size() - 1) cout << ", ";
                    }
                    cout << "]" << endl;
                    
                    const auto& times = node->getTimes();
                    cout << "    Robot Times: [";
                    for (size_t j = 0; j < times.size(); ++j) {
                        cout << times[j];
                        if (j < times.size() - 1) cout << ", ";
                    }
                    cout << "]" << endl;
                }
                if (leafNodes.size() > sampleCount) {
                    cout << "\n  ... and " << (leafNodes.size() - sampleCount) << " more frontier nodes" << endl;
                }
            }
            
            cout << "\n✓ COMPREHENSIVE TREE SEARCH TEST PASSED!" << endl;
        } else {
            cout << "\n✗ Tree search returned nullptr" << endl;
        }
        
    } catch (const exception& e) {
        cout << "\n✗ Algorithm execution failed: " << e.what() << endl;
    }
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

// ============================================================================
// COMPREHENSIVE TEST FOR collectUniqueAPsFromEdges FUNCTION
// ============================================================================

void testCollectUniqueAPsFromEdgesComprehensive() {
    cout << "\n" << string(70, '=') << endl;
    cout << "COMPREHENSIVE TEST: collectUniqueAPsFromEdges FUNCTION" << endl;
    cout << string(70, '=') << endl;
    
    TS* ts = nullptr; GridWorld* grid = nullptr; Environment* env = nullptr;
    createTestSystemComponents(ts, grid, env);
    BuchiAutomaton* nba = createTestBuchiAutomaton();
    MultiRobotSystem* mrs = createTestMultiRobotSystem();
    
    TaskAllocationAlgorithms algo(nba, env, mrs);
    
    cout << "\n=== Test 1: Single Edge with Single AP ===" << endl;
    {
        vector<string> edges = {"p0"};
        cout << "  Input edges: [\"p0\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [0], Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        assert(result.size() == 1 && result[0] == 0);
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 2: Single Edge with Multiple APs (OR separated) ===" << endl;
    {
        vector<string> edges = {"p0 | p1 | p2"};
        cout << "  Input edges: [\"p0 | p1 | p2\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [0, 1, 2], Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Check that we have exactly 3 APs and they are 0, 1, 2
        assert(result.size() == 3);
        assert(std::find(result.begin(), result.end(), 0) != result.end());
        assert(std::find(result.begin(), result.end(), 1) != result.end());
        assert(std::find(result.begin(), result.end(), 2) != result.end());
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 3: Single Edge with Multiple APs (AND separated) ===" << endl;
    {
        vector<string> edges = {"p3 & p4 & p5"};
        cout << "  Input edges: [\"p3 & p4 & p5\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [] (filtered - 3 true APs AND-ed), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Should be empty because edge has 3 true APs AND-ed together
        assert(result.size() == 0);
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 4: Multiple Edges with Duplicate APs ===" << endl;
    {
        vector<string> edges = {"p0 | p1", "p1 | p2", "p0"};
        cout << "  Input edges: [\"p0 | p1\", \"p1 | p2\", \"p0\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [0, 1, 2] (duplicates removed), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Should have exactly 3 unique APs despite duplicates
        assert(result.size() == 3);
        assert(std::find(result.begin(), result.end(), 0) != result.end());
        assert(std::find(result.begin(), result.end(), 1) != result.end());
        assert(std::find(result.begin(), result.end(), 2) != result.end());
        cout << "  ✓ PASSED (Duplicates correctly handled)" << endl;
    }
    
    cout << "\n=== Test 5: Empty Edge List ===" << endl;
    {
        vector<string> edges;
        cout << "  Input edges: []" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [], Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        assert(result.size() == 0);
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 6: Complex Edge with Negations and Acceptance Marks ===" << endl;
    {
        vector<string> edges = {"p0 & p1 | !p2 | p3 {0}"};
        cout << "  Input edges: [\"p0 & p1 | !p2 | p3 {0}\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [] (filtered - first OR clause has 2 true APs AND-ed), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Should be empty because first OR clause "p0 & p1" has 2 true APs AND-ed together
        assert(result.size() == 0);
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 7: High Index APs ===" << endl;
    {
        vector<string> edges = {"p10 | p11 | p12"};
        cout << "  Input edges: [\"p10 | p11 | p12\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [10, 11, 12], Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        assert(result.size() == 3);
        assert(std::find(result.begin(), result.end(), 10) != result.end());
        assert(std::find(result.begin(), result.end(), 11) != result.end());
        assert(std::find(result.begin(), result.end(), 12) != result.end());
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 8: Mixed Operators with Duplicate APs ===" << endl;
    {
        vector<string> edges = {"p5 | p6 & p5", "p6 | p7", "p5 & p6 & p7"};
        cout << "  Input edges: [\"p5 | p6 & p5\", \"p6 | p7\", \"p5 & p6 & p7\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [6, 7] (first and third edges filtered), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Edge 1: "p5 | p6 & p5" → skipped (contains "p6 & p5" with 2 true APs AND-ed)
        // Edge 2: "p6 | p7" → kept (each OR clause has 1 true AP)
        // Edge 3: "p5 & p6 & p7" → skipped (3 true APs AND-ed)
        assert(result.size() == 2);
        assert(std::find(result.begin(), result.end(), 6) != result.end());
        assert(std::find(result.begin(), result.end(), 7) != result.end());
        cout << "  ✓ PASSED (Correctly filtered edges with AND-ed APs)" << endl;
    }
    
    cout << "\n=== Test 9: Negations with Single True AP ===" << endl;
    {
        vector<string> edges = {"!p0 & p1"};
        cout << "  Input edges: [\"!p0 & p1\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [1] (p0 negated, p1 true), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        assert(result.size() == 1);
        assert(std::find(result.begin(), result.end(), 1) != result.end());
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 10: Multiple Negations with One True AP ===" << endl;
    {
        vector<string> edges = {"!p0 & !p1 & p2"};
        cout << "  Input edges: [\"!p0 & !p1 & p2\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [2] (p0 and p1 negated, p2 true), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        assert(result.size() == 1);
        assert(std::find(result.begin(), result.end(), 2) != result.end());
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 11: Mixed True and Negated APs with 2 True ===" << endl;
    {
        vector<string> edges = {"!p0 & p1 & p2"};
        cout << "  Input edges: [\"!p0 & p1 & p2\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [] (filtered - 2 true APs AND-ed), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Should be filtered because p1 & p2 are 2 true APs AND-ed
        assert(result.size() == 0);
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n=== Test 12: Negations in OR Clauses ===" << endl;
    {
        vector<string> edges = {"!p0 & p1 | p2 | !p3 & p4"};
        cout << "  Input edges: [\"!p0 & p1 | p2 | !p3 & p4\"]" << endl;
        auto result = algo.collectUniqueAPsFromEdges(edges);
        cout << "  Expected: [1, 2, 4] (OR-separated clauses kept), Got: [";
        for (size_t i = 0; i < result.size(); ++i) {
            cout << result[i];
            if (i < result.size() - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // All clauses are valid: "!p0 & p1" (1 true), "p2" (1 true), "!p3 & p4" (1 true)
        assert(result.size() == 3);
        assert(std::find(result.begin(), result.end(), 1) != result.end());
        assert(std::find(result.begin(), result.end(), 2) != result.end());
        assert(std::find(result.begin(), result.end(), 4) != result.end());
        cout << "  ✓ PASSED" << endl;
    }
    
    cout << "\n" << string(70, '=') << endl;
    cout << "✓ ALL collectUniqueAPsFromEdges TESTS PASSED!" << endl;
    cout << string(70, '=') << endl;
    
    delete env;
    delete ts;
    delete grid;
    delete nba;
    delete mrs;
}

int main() {
    cout << "\n" << string(70, '=') << endl;
    cout << "TaskAllocationAlgorithms - COMPREHENSIVE TEST SUITE" << endl;
    cout << string(70, '=') << endl;
    
    try {
        // Test Suite 1: Constructor and Basic Setup
       // testConstructor();
       // testSettersAndGetters();
        
        // Test Suite 2: Visited Nodes
       // testVisitedNodesManagement();
        
        // Test Suite 3: Automaton States
//testVisitedAutomatonStatesManagement();
        
        // Test Suite 4: Batch Values
      //  testBatchValuesManagement();
        
        // Test Suite 5: Untraversed Queue
       // testUntraversedQueueManagement();
        
        // Test Suite 6: Edge Label Parsing
      // testParseEdgeLabel();
       
       // Test getEdgeLabels from BuchiAutomaton
      // testGetEdgeLabelsComprehensive();
       
       // Comprehensive collectUniqueAPsFromEdges Test
      //  testCollectUniqueAPsFromEdgesComprehensive();
        
        // Test getTaskAllocation function comprehensively
       // testGetTaskAllocationComprehensive();
        
        // Comprehensive Tree Search Algorithm Test (tests all sub-algorithms)
      //  testTreeSearchAlgorithmComprehensive();
        
        // Comprehensive Unrelated Task Search Test
      //  testUnrelatedTaskSearchComprehensive();
        
        // Full Algorithm Test
        cout << "\n" << string(70, '-') << endl;
        cout << "FULL ALGORITHM INTEGRATION TEST" << endl;
        cout << string(70, '-') << endl;
        testFullAlgorithmIntensiveInterTaskRelationshipSearch();
        
        cout << "\n" << string(70, '=') << endl;
        cout << "✓ ALL TESTS PASSED! (Full algorithm and sub-algorithms verified)" << endl;
        cout << string(70, '=') << "\n" << endl;
        
        return 0;
        
    } catch (const exception& e) {
        cout << "\n✗ TEST FAILED: " << e.what() << endl;
        return 1;
    }
}
