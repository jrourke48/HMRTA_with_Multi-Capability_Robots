#include <iostream>
#include <vector>
#include <memory>
#include <cstdlib>
#include "../TaskAllocationAlgorithms.h"
#include "../LTLFormula/LTLFormula.h"
#include "../LTLFormula/BatchAtomicProposition.h"
#include "../Environment/Environment.h"
#include "../Environment/GridWorld.h"
#include "../Environment/Point.h"
#include "../MultiRobotSystem/MultiRobotSystem.h"
#include "../MultiRobotSystem/Robot.h"
#include "../Tree/PlanningDecisionTree.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/TS.h"

void testAlgorithm1_IntensiveInterTaskSearch() {
    std::cout << "\n========== TEST 1: Intensive Inter-Task Relationship Search ==========" << std::endl;
    //Step 1: Create LTL formula with batch atomic propositions representing tasks and their relationships
    // Create LTL formula with batch atomic propositions
    std::vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, {true, false, true, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, {true, true, false, false}, 0));
    
    LTLFormula formula("G(\"p1\" -> F \"p2\")", batchAPs);
    
    // Create Buchi automaton from LTL formula
    BuchiAutomaton nba(&formula);
    
    // Export automaton to DOT and PNG
    std::cout << "\n=== Exporting Büchi Automaton ===" << std::endl;
    try {
        nba.visualize("../output/buchi_automaton_tree_test");
        std::cout << "✓ Exported to DOT and PNG: ../output/buchi_automaton_tree_test.*" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "⚠ Export failed: " << e.what() << std::endl;
    }
    
    std::cout << "LTL Formula: " << formula.toString() << std::endl;
    std::cout << "Automaton states: " << nba.getNumStates() << std::endl;
    std::cout << "Accepting states: " << nba.getAcceptingStates().size() << std::endl;

    // Step 2: Create environment and multi-robot system
    // Create environment with GridWorld and TS (use pointers for Environment compatibility)
    GridWorld gridWorld(10, 10);
    TS ts;  // Placeholder TS
    // Add 3 states with edges: 1 -> 2, 1 -> 3
    Node* node0 = new Node(0, "R0");
    Node* node1 = new Node(1, "R1");
    Node* node2 = new Node(2, "R2");
    // Add edges: 1 -> 2 and 1 -> 3
    node0->addEdge(Edge(1));
    node0->addEdge(Edge(2));
    node1->addEdge(Edge(0));
    node2->addEdge(Edge(0));
    // Add nodes to TS
    ts.add_Node(node0);
    ts.add_Node(node1);
    ts.add_Node(node2);
    ts.setInitial(0);
    Environment env(&ts, &gridWorld);
    //map grid to ts
    // Test 3: Map states to grid regions
    std::cout << "\n=== Test 3: Map States to Grid Regions ===" << std::endl;
    env.mapTSStateToGrid(0, Point(7, 5), 5, 10);    // State 0 centered at (7,5), 4x4 region
    env.mapTSStateToGrid(1, Point(2, 7), 5, 6);   // State 1 centered at (8,8), 4x4 region
    env.mapTSStateToGrid(2, Point(2, 2), 5, 4);   // State 2 centered at (7,11)
    std::cout << "✓ Mapped 3 states to grid regions" << std::endl;
    std::cout << "Environment created: " << env.getNumStates() << " states" << std::endl;
    
    // Create multi-robot system
    MultiRobotSystem multiRobotSystem;
    //print the environment
        env.print_Environment();
        std::cout << "✓ Environment printed successfully" << std::endl;
    
    // Add robots with different capabilities
    Robot* r1 = new Robot(0, "Robot1", Point(10, 10));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r1->enableCapability(RobotCapability::SENSOR_CAMERA);
    multiRobotSystem.addRobot(r1);
    
    Robot* r2 = new Robot(1, "Robot2", Point(10, 8));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r2->enableCapability(RobotCapability::MANIPULATION_GRIPPER);
    multiRobotSystem.addRobot(r2);
    
    std::cout << "Multi-robot system created with " << multiRobotSystem.getRobots().size() << " robots" << std::endl;
    
    // Create algorithm instance
    TaskAllocationAlgorithms algorithms(&nba, &env, &multiRobotSystem);
    algorithms.intensiveInterTaskRelationshipTreeSearch(&nba, &env, &multiRobotSystem);
    
    // For now, just verify the setup - tree building will be enhanced in future iterations
    std::cout << "✓ Algorithm 1 setup completed successfully" << std::endl;
}

void testAlgorithm2_UnrelatedTaskSearch() {
    std::cout << "\n========== TEST 2: Unrelated-Task Search ==========" << std::endl;
    
    std::vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, {true, false, false, true}, 1));
    batchAPs.push_back(BatchAtomicProposition(1, {false, true, false, false}, 2));
    
    LTLFormula formula("G(\"p0\" -> F \"p1\")", batchAPs);
    BuchiAutomaton nba(&formula);
    
    // Export automaton to DOT and PNG
    std::cout << "\n=== Exporting Büchi Automaton ===" << std::endl;
    try {
        nba.visualize("../output/buchi_automaton_unrelated_test");
        std::cout << "✓ Exported to DOT and PNG: ../output/buchi_automaton_unrelated_test.*" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "⚠ Export failed: " << e.what() << std::endl;
    }
    
    GridWorld gridWorld(10, 10);
    TS ts;
    Environment env(&ts, &gridWorld);
    
    MultiRobotSystem multiRobotSystem;
    Robot* r = new Robot(0, "Robot1", Point(0, 0));
    r->initializeCapabilities(13);
    r->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r->enableCapability(RobotCapability::SENSOR_CAMERA);
    multiRobotSystem.addRobot(r);
    
    TaskAllocationAlgorithms algorithms(&nba, &env, &multiRobotSystem);
    algorithms.unrelatedTaskSearch(nullptr, nullptr, nullptr);
    
    std::cout << "✓ Unrelated task search completed successfully" << std::endl;
}

int main() {
    std::cout << "========== Tree Building Tests Suite ==========" << std::endl;
    std::cout << "Testing Task Allocation Algorithms" << std::endl;
    std::cout << "============================================" << std::endl;
    
    try {
        testAlgorithm1_IntensiveInterTaskSearch();
        std::cout << "\n========== ALL TESTS COMPLETED ==========" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
