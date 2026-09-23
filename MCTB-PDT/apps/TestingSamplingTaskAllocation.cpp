#include <iostream>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>
#include "TaskAllocationAlgorithms.h"
#include "Tree/PlanningDecisionTree.h"
#include "Tree/Tree_Node.h"
#include "Environment/Environment.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "LTLFormula/LTLFormula.h"
#include "../Automatons/BuchiAutomaton.h"
#include "RandomSamplingAlgo/RandomSamplingTaskAllocation.h"
#include "RandomSamplingAlgo/RandomNode.h"

using namespace std;

// Forward declarations
void testinggetAcceptingSCCS();
void testingRandomFeasibleTaskAllocation();
void testingRun();
void createTestSystemComponents(TS*& ts, GridWorld*& grid, Environment*& env);
void createTestSystemComponents2(TS*& ts, GridWorld*& grid, Environment*& env);
MultiRobotSystem* createTestMultiRobotSystem();
MultiRobotSystem* createTestMultiRobotSystem2();
BuchiAutomaton* createTestBuchiAutomaton();
BuchiAutomaton* createTestBuchiAutomaton2();

int main(){
    cout << "\n=== TESTING RANDOM SAMPLING TASK ALLOCATION ===" << endl;
    
    cout << "\n--- Test 1: Testing and Visualizing Accepting SCCs ---" << endl;
    testinggetAcceptingSCCS();
    
    cout << "\n--- Test 2: Testing with Time Limit ---" << endl;
    testingRandomFeasibleTaskAllocation();
    
    cout << "\n--- Test 3: Testing Run Method ---" << endl;
    testingRun();
    
    cout << "\n=== ALL TESTS COMPLETED ===" << endl;
    return 0;
}

void testinggetAcceptingSCCS(){
    cout << "Testing and Visualizing Accepting SCCs..." << endl;
    
    // Create test components
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton();
    
    // Debug: Print automaton structure
    cout << "\n  DEBUG: Büchi Automaton Structure:" << endl;
    const auto& nodeMap = buchi->getNodes();
    const auto& acceptingStates = buchi->getAcceptingStates();
    cout << "    Total nodes: " << nodeMap.size() << endl;
    cout << "    Accepting states: ";
    for (uint16_t acceptingId : acceptingStates) {
        cout << acceptingId << " ";
    }
    cout << endl;
    for (const auto& pair : nodeMap) {
        uint16_t nodeId = pair.first;
        Node* node = pair.second;
        cout << "    Node " << nodeId << " edges: ";
        const auto& edges = node->getEdges();
        for (const auto& edge : edges) {
            cout << nodeId << "->" << edge.getDstId() << " ";
        }
        if (edges.empty()) cout << "(no edges)";
        cout << endl;
    }
    cout << endl;
    
    // Create RandomSamplingTaskAllocation with 100 max iterations
    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)100);
    
    // Test setAcceptingSCCs (now public)
    sampler.setAcceptingSCCs();
    
    // Verify results
    uint16_t numSCCs = sampler.getNumSCCs();
    cout << "  Number of accepting SCCs found: " << (int)numSCCs << endl;
    
    if (numSCCs > 0) {
        cout << "  ✓ Found accepting SCCs" << endl;
        auto acceptingSCCs = sampler.getAcceptingSCCs();
        for (size_t i = 0; i < acceptingSCCs.size(); ++i) {
            cout << "    SCC " << i << " contains nodes: ";
            for (uint16_t nodeId : acceptingSCCs[i]) {
                cout << nodeId << " ";
            }
            cout << endl;
        }
    } else {
        cout << "  ✗ No accepting SCCs found" << endl;
    }
    
    // Cleanup
    delete buchi;
    delete mrs;
    delete env;
    delete ts;
    delete grid;
    
    cout << "✓ testinggetAcceptingSCCS completed" << endl;
}

void testingRandomFeasibleTaskAllocation(){
    cout << "Testing getRandomFeasibleTaskAllocation method..." << endl;
    
    // Create test components
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton2();
    
    // Create RandomSamplingTaskAllocation with time limit of 5 seconds
    RandomSamplingTaskAllocation sampler(buchi, env, mrs, 5.0);
    
    // Test with actual nodes from Büchi automaton
    sampler.setAcceptingSCCs();
    auto acceptingSCCs = sampler.getAcceptingSCCs();
    
    if (acceptingSCCs.size() > 0) {
        // Test with first two accepting states if available
        if (acceptingSCCs[0].size() >= 2) {
            uint16_t nodeId1 = acceptingSCCs[0][0];
            uint16_t nodeId2 = acceptingSCCs[0][1];
            
            Node* node1 = buchi->getNode(nodeId1);
            Node* node2 = buchi->getNode(nodeId2);
            
            if (node1 && node2) {
                cout << "  Testing allocation between nodes " << nodeId1 << " and " << nodeId2 << endl;
                
                auto [robotsByAP, satisfiedAPs] = sampler.getRandomFeasibleTaskAllocation(node1, node2);
                
                cout << "  Satisfied APs: ";
                for (uint16_t ap : satisfiedAPs) {
                    cout << ap << " ";
                }
                cout << endl;
                
                cout << "  Robots assigned to each AP:" << endl;
                for (size_t i = 0; i < robotsByAP.size(); ++i) {
                    cout << "    AP " << i << ": ";
                    for (uint8_t robotId : robotsByAP[i]) {
                        cout << (int)robotId << " ";
                    }
                    cout << endl;
                }
                
                cout << "  ✓ Successfully allocated feasible task allocation" << endl;
            }
        }
    } else {
        cout << "  ✗ No accepting SCCs found to test" << endl;
    }
    
    // Cleanup
    delete buchi;
    delete mrs;
    delete env;
    delete ts;
    delete grid;
    
    cout << "✓ testingRandomFeasibleTaskAllocation completed" << endl;
}

void testingRun(){
    cout << "Testing RandomSamplingTaskAllocation initialization..." << endl;
    
    // Create test components
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    createTestSystemComponents2(ts, grid, env);
    
    MultiRobotSystem* mrs = createTestMultiRobotSystem2();
    BuchiAutomaton* buchi = createTestBuchiAutomaton2();
    
    // Create RandomSamplingTaskAllocation with 10 max iterations (cast to uint16_t to avoid ambiguity)
    RandomSamplingTaskAllocation sampler(buchi, env, mrs, (uint16_t)10);
    
    cout << "  ✓ RandomSamplingTaskAllocation created successfully" << endl;
    
    // Note: run() method is not ready yet
    // sampler.run();
    
    // Cleanup
    delete buchi;
    delete mrs;
    delete env;
    delete ts;
    delete grid;
    
    cout << "✓ testingRun completed" << endl;
}

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
    string ltl_str = "(G(F\"p1\") && G(F\"p2\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton");
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
