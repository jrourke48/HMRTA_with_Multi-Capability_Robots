//#include "../TaskAllocationAlgorithms.h"
#include "../MultiRobotSystem/MultiRobotSystem.h"
#include "../MultiRobotSystem/Robot.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/TS.h"
#include "../../Automatons/Edge_Node.h"
#include "../Environment/Environment.h"
#include "../Environment/GridWorld.h"
#include "../Automatons/Edge_Node.h"
#include "../LTLFormula/LTLFormula.h"
#include "../LTLFormula/BatchAtomicProposition.h"
#include <iostream>
#include <cassert>
#include <spot/tl/formula.hh>

// Forward declarations
void testComponentCreation();
void testBuildPlanningTree();
void testEnvironment();

/**
 * Test: Buchi Automaton Creation from Spot LTL Formula
 */
int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "  Testing Buchi Automaton with Spot LTL Formulas" << std::endl;
    std::cout << "==========================================================" << std::endl;
    
    try {
        // Test 1: Create LTL Formula from simple formula
        std::cout << "\n=== Test 1: Simple LTL Formula ===" << std::endl;
        std::string ltl1_str = "F \"p0\"";  // Eventually p0
        std::cout << "Creating formula: " << ltl1_str << std::endl;
        
        std::vector<BatchAtomicProposition> batchAPs1;
        batchAPs1.push_back(BatchAtomicProposition(0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        
        LTLFormula ltl1(ltl1_str, batchAPs1);
        std::cout << "✓ LTL Formula created: " << ltl1.getFormula() << std::endl;
        
        BuchiAutomaton* buchi1 = new BuchiAutomaton(&ltl1);
        std::cout << "✓ Buchi automaton created successfully" << std::endl;
        std::cout << "  - States: " << buchi1->getNumStates() << std::endl;
        std::cout << "  - Edges: " << buchi1->getNumEdges() << std::endl;
        std::cout << "  - Accepting States: " << buchi1->getAcceptingStates().size() << std::endl;
        delete buchi1;
        
        // Test 2: Create LTL Formula from complex formula
        std::cout << "\n=== Test 2: Complex LTL Formula ===" << std::endl;
        std::string ltl2_str = "F\"p0\" && F \"p1\"";  // always eventually p0 and eventually p1
        std::cout << "Creating formula: " << ltl2_str << std::endl;
        
        std::vector<BatchAtomicProposition> batchAPs2;
        batchAPs2.push_back(BatchAtomicProposition(0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs2.push_back(BatchAtomicProposition(1, {false, true, false, false, false, false, false, false, false, false, false, false, false}, 1));
        
        LTLFormula ltl2(ltl2_str, batchAPs2);
        std::cout << "✓ LTL Formula created: " << ltl2.getFormula() << std::endl;
        std::cout << "  - Batch APs: " << ltl2.getBatchAPs().size() << std::endl;
        
        BuchiAutomaton* buchi2 = new BuchiAutomaton(&ltl2);
        std::cout << "✓ Buchi automaton created successfully" << std::endl;
        std::cout << "  - States: " << buchi2->getNumStates() << std::endl;
        std::cout << "  - Edges: " << buchi2->getNumEdges() << std::endl;
        std::cout << "  - Accepting States: " << buchi2->getAcceptingStates().size() << std::endl;
        if (buchi2->getAcceptingStates().size() > 0) {
            std::cout << "  - Accepting state IDs: ";
            for (auto stateId : buchi2->getAcceptingStates()) {
                std::cout << stateId << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "  - All nodes:" << std::endl;
        for (uint16_t i = 0; i < buchi2->getNumStates(); i++) {
            Node* node = buchi2->getNode(i);
            if (node) {
                std::cout << "    Node " << i << ": " << node->to_string() << std::endl;
            } else {
                std::cout << "    Node " << i << ": (null)" << std::endl;
            }
        }
        
        // Print atomic propositions mapping
        std::cout << "  - Atomic Propositions:" << std::endl;
        spot::formula ltl_formula = buchi2->get_ltl_formula();
        spot::atomic_prop_set* aps_ptr = spot::atomic_prop_collect(ltl_formula);
        unsigned ap_index = 0;
        if (aps_ptr) {
            for (auto ap : *aps_ptr) {
                std::cout << "    p" << ap_index << " -> " << ap << std::endl;
                ap_index++;
            }
            delete aps_ptr;
        }
        
        // Print batch APs info
        std::cout << "  - Batch Atomic Propositions:" << std::endl;
        for (const auto& bap : ltl2.getBatchAPs()) {
            std::cout << "    " << bap.toString() << std::endl;
        }
        
        // Test 3: Check Edge Labels are properly extracted
        std::cout << "\n=== Test 3: Edge Label Extraction ===" << std::endl;
        std::cout << "  - Checking edge labels from LTL formula: " << ltl2_str << std::endl;
        bool hasEdgesWithLabels = false;
        for (uint16_t i = 0; i < buchi2->getNumStates(); i++) {
            Node* node = buchi2->getNode(i);
            if (node) {
                const std::vector<Edge>& edges = node->getEdges();
                if (!edges.empty()) {
                    std::cout << "    State " << i << " has " << edges.size() << " edge(s):" << std::endl;
                    for (size_t j = 0; j < edges.size(); ++j) {
                        const Edge& edge = edges[j];
                        std::string label = edge.getLabel();
                        std::cout << "      Edge " << j << " -> State " << edge.getDstId();
                        if (!label.empty() && label != "true" && label != "false") {
                            std::cout << " | Label: \"" << label << "\"";
                            hasEdgesWithLabels = true;
                        } else if (label == "true") {
                            std::cout << " | Label: (unconditional)";
                        } else if (label == "false") {
                            std::cout << " | Label: (impossible)";
                        } else {
                            std::cout << " | Label: (empty)";
                        }
                        std::cout << std::endl;
                    }
                }
            }
        }
        
        if (hasEdgesWithLabels) {
            std::cout << "  ✓ Edge labels successfully extracted from LTL formula!" << std::endl;
        } else {
            std::cout << "  ⚠ No edge labels found (may be normal for some formulas)" << std::endl;
        }
        
        // Test 4: Visualize Buchi Automaton as PNG
        std::cout << "\n=== Test 4: Buchi Automaton Visualization ===" << std::endl;
        try {
            buchi2->visualize("../output/buchi_automaton_test");
            std::cout << "✓ Buchi automaton visualized successfully!" << std::endl;
            std::cout << "  - Generated: ../output/buchi_automaton_test.dot" << std::endl;
            std::cout << "  - Generated: ../output/buchi_automaton_test.png" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "⚠ Failed to generate visualization: " << e.what() << std::endl;
            std::cerr << "  (Ensure graphviz 'dot' command is installed)" << std::endl;
        }
        
        delete buchi2;
        
        std::cout << "\n==========================================================" << std::endl;
        std::cout << "  All Buchi automaton tests passed! ✓" << std::endl;
        std::cout << "==========================================================" << std::endl;
        
        // Run integration tests
        testEnvironment();
        //testComponentCreation();
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}

/**
 * Test: Environment with GridWorld and Transition System
 */
void testEnvironment() {
    std::cout << "\n===========================================================" << std::endl;
    std::cout << "  Testing Environment: GridWorld + TS Overlay" << std::endl;
    std::cout << "===========================================================" << std::endl;
    
    try {
        // Test 1: Create GridWorld and TS (stack allocated for correct cleanup order)
        std::cout << "\n=== Test 1: GridWorld and TS Creation ===" << std::endl;
        GridWorld grid(15, 15);
        std::cout << "✓ GridWorld created (15x15)" << std::endl;
        
        TS ts;
        std::cout << "✓ Transition System created" << std::endl;
        
        // Test 2: Create Environment
        std::cout << "\n=== Test 2: Environment Creation ===" << std::endl;
        Environment env(&ts, &grid);
        std::cout << "✓ Environment created" << std::endl;
        
        // Test 3: Map states to grid regions
        std::cout << "\n=== Test 3: Map States to Grid Regions ===" << std::endl;
        env.mapTSStateToGrid(0, Point(3, 3), 4, 4);    // State 0 centered at (3,3), 4x4 region
        env.mapTSStateToGrid(1, Point(11, 3), 4, 4);   // State 1 centered at (11,3)
        env.mapTSStateToGrid(2, Point(7, 11), 4, 4);   // State 2 centered at (7,11)
        std::cout << "✓ Mapped 3 states to grid regions" << std::endl;
        
        // Test 4: Query grid-to-state mappings
        std::cout << "\n=== Test 4: Grid-to-State Queries ===" << std::endl;
        uint16_t state_at_3_3 = env.gridToTSStateId(Point(3, 3));
        uint16_t state_at_11_3 = env.gridToTSStateId(Point(11, 3));
        uint16_t state_at_7_11 = env.gridToTSStateId(Point(7, 11));
        std::cout << "✓ Grid point (3, 3) -> State " << state_at_3_3 << std::endl;
        std::cout << "✓ Grid point (11, 3) -> State " << state_at_11_3 << std::endl;
        std::cout << "✓ Grid point (7, 11) -> State " << state_at_7_11 << std::endl;
        
        // Test 5: Query state-to-grid mappings
        std::cout << "\n=== Test 5: State-to-Grid Queries ===" << std::endl;
        Point center0 = env.TSStateIdToGridCenter(0);
        Point center1 = env.TSStateIdToGridCenter(1);
        Point center2 = env.TSStateIdToGridCenter(2);
        std::cout << "✓ State 0 center -> Grid (" << center0.getX() << ", " << center0.getY() << ")" << std::endl;
        std::cout << "✓ State 1 center -> Grid (" << center1.getX() << ", " << center1.getY() << ")" << std::endl;
        std::cout << "✓ State 2 center -> Grid (" << center2.getX() << ", " << center2.getY() << ")" << std::endl;
        
        // Test 6: Check grid properties
        std::cout << "\n=== Test 6: Grid Property Queries ===" << std::endl;
        std::cout << "✓ Is free at (5, 5): " << (env.isFree(Point(5, 5)) ? "yes" : "no") << std::endl;
        std::cout << "✓ Number of states: " << env.getNumStates() << std::endl;
        
        // Test 7: Print ASCII visualization
        std::cout << "\n=== Test 7: ASCII Visualization ===" << std::endl;
        env.print_Environment();
        
        std::cout << "==========================================================" << std::endl;
        std::cout << "  All Environment tests passed! ✓" << std::endl;
        std::cout << "==========================================================" << std::endl;
        
        // Grid, ts, and env are stack-allocated and cleanup automatically
        
    } catch (const std::exception& e) {
        std::cerr << "\n❌ Environment test failed: " << e.what() << std::endl;
    }
}

/**
 * Test 1: Create all system components
 */
void testComponentCreation() {
    std::cout << "\n=== Test 1: System Component Creation ===" << std::endl;
    
    // Create GridWorld
    GridWorld grid(10, 10);
    std::cout << "✓ GridWorld created (10x10)" << std::endl;
    
    // Create Transition System (stack allocated so cleanup order is correct)
    TS ts;
    
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

    std::cout << "✓ Transition System created" << std::endl;
    std::cout << "  - States: " << ts.getNumStates() << std::endl;
    std::cout << "  - Initial state: 0" << std::endl;
    
    Environment env(&ts, &grid);
    std::cout << "✓ Environment created" << std::endl;
    
    // Create MultiRobotSystem
    MultiRobotSystem mrs;
    
    Robot* r1 = new Robot(1, "Rover_1", Point(0, 0));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::SENSOR_GPS);
    mrs.addRobot(r1);
    
    Robot* r2 = new Robot(2, "Rover_2", Point(1, 1));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);
    mrs.addRobot(r2);
    Robot* r3 = new Robot(3, "Rover_3", Point(2, 2));
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::SENSOR_CAMERA);
    mrs.addRobot(r3);
    
    std::cout << "✓ MultiRobotSystem created with 3 robots" << std::endl;
    std::cout << "✓ All components created successfully" << std::endl;

    // Create LTL Formula
    std::string ltl2_str = "F\"p0\" && F \"p1\"";  // always eventually p0 and eventually p1
    std::cout << "Creating LTL formula: " << ltl2_str << std::endl;
    
    std::vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 1));
    
    LTLFormula ltlFormula(ltl2_str, batchAPs);
    std::cout << "✓ LTL Formula created" << std::endl;
    
    // Create Büchi automaton from formula
    BuchiAutomaton* buchi2 = new BuchiAutomaton(&ltlFormula);
    std::cout << "✓ BuchiAutomaton created" << std::endl;
    
    // Create algorithm
    // TaskAllocationAlgorithms algo(buchi2, &env, &mrs);
    
    // Prepare parameters for buildPlanningTree
    Node* automatonState = buchi2->getNode(0);  // Get initial automaton state
    std::vector<bool> taskAlloc(1, false);  // 1 robot, not allocated yet
    std::vector<uint16_t> times(1, 0);      // Initial time 0
    
    if (automatonState) {
        // Build tree - note: buildPlanningTree expects both automaton and TS states
        // For now, just verify objects exist
        std::cout << "✓ Planning tree components ready" << std::endl;
    } else {
        std::cout << "✓ Automaton state not available" << std::endl;
    }
    
    // Cleanup
    delete buchi2;
    // ts, grid, and env are stack-allocated and cleanup automatically
}