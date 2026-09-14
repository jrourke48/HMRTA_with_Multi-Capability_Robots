#include <iostream>
#include <vector>
#include <tuple>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
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

using namespace std;
//=================================================================================
// Test: Number of Average Capabilities per robot: 8 total average capabilities configurations ranging from (1-4)
//=================================================================================
// Buchi: 6 automata (8-64) states
//=================================================================================
//Environments: 6-robot, 10-robot, 16-robot all with 6 TS regions
//=================================================================================
    
// Forward declarations
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, int robotCount, double aveCap);
BuchiAutomaton* createTestInfiniteBuchiAutomaton1();  // Originally automaton 2
BuchiAutomaton* createTestInfiniteBuchiAutomaton2();  // Originally automaton 4
BuchiAutomaton* createTestInfiniteBuchiAutomaton3();  // Originally automaton 6
BuchiAutomaton* createTestInfiniteBuchiAutomaton4();  // Originally automaton 8
BuchiAutomaton* createTestInfiniteBuchiAutomaton5();  // Originally automaton 10
BuchiAutomaton* createTestInfiniteBuchiAutomaton6();  // Originally automaton 12


// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   AVERAGE CAPABILITIES TEST SUITE" << endl;
    cout << "   6 Büchi Automata (every other from 2-12)" << endl;
    cout << "   8 Average Capabilities: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0" << endl;
    cout << "   Total Tests: 144 (6 automata × 8 capabilities × 3 robot counts)" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Initialize TestRunManager for AUTOMATON_STATES category
    TestRunManager manager(TestRunManager::TestCategory::AUTOMATON_STATES, ".");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    // Create array of automaton factory functions (6 automatons: every other from 2-12)
    vector<BuchiAutomaton*(*)()> automatonFactories = {
        createTestInfiniteBuchiAutomaton1,
        createTestInfiniteBuchiAutomaton2,
        createTestInfiniteBuchiAutomaton3,
        createTestInfiniteBuchiAutomaton4,
        createTestInfiniteBuchiAutomaton5,
        createTestInfiniteBuchiAutomaton6
    };
    
    vector<double> aveCaps = {0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4};
    vector<int> robotCounts = {6, 10, 16};
    int testNum = 1;
    
    // Run tests: automatonId outer, robotCount middle, aveCap inner
    // This ensures each (automatonId, robotCount) pair gets all 8 average_capabilities values
    for (int automatonId = 1; automatonId <= 6; ++automatonId) {
        for (int robotCount : robotCounts) {
        cout << "\n" << string(80, '=') << endl;
        cout << "   AUTOMATON " << automatonId << " WITH " << robotCount << "-ROBOT ENVIRONMENT" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        cout << "\n" << string(80, '-') << endl;
        cout << "   RUNNING TESTS (average_capabilities varying)" << endl;
        cout << string(80, '-') << "\n" << endl;
        
        // For each of the 8 average capabilities values
        for (double aveCap : aveCaps) {
            // Create test environment for this configuration
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            createTestEnvironment(ts, grid, env, mrs, robotCount, aveCap);
            cout << "\n  Test " << testNum << " (Automaton " << automatonId << ", Robots " << robotCount << ", AveCap " << aveCap << ")... ";
            cout.flush();
            
            try {
                // Create the Buchi automaton
                BuchiAutomaton* buchi = automatonFactories[automatonId - 1]();
                
                if (!buchi) {
                    cout << "ERROR: Failed to create automaton" << endl;
                    testNum++;
                    continue;
                }
                
                // Create TaskAllocationAlgorithms
                TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
                
                // Measure memory 
                double memBefore = getMemoryUsageMB();
                //build the planning decision tree
                allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
                double memAfter = getMemoryUsageMB();
                double memUsed = memAfter - memBefore;
                allocAlg->getMetrics().setTaskMemoryUsageMB(memUsed);
                // bool shouldSkip = (robotCount > 10) && (buchi->getNumStates()*std::pow(ts->getNumStates(), robotCount) > UINT16_MAX/2);
                // if (!shouldSkip) {
                //     //buld the product automaton and store its metrics
                //     double memBeforeProduct = getMemoryUsageMB();
                //     double startTimeProduct = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                //     ProductAutomaton product(*env, *mrs, *buchi);
                //     std::tuple<std::vector<uint16_t>, uint32_t> optimalPath = product.OptimalAcceptingPath();
                //     double memAfterProduct = getMemoryUsageMB();
                //     double endTimeProduct = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                //     double memUsedProduct = memAfterProduct - memBeforeProduct;
                    
                //     //add the full product automaton metrics to the algorithm metrics
                //     allocAlg->getMetrics().setFullProductAutomatonMetrics(
                //         product.getNumStates(),
                //         product.getNumEdges(),
                //         std::get<1>(optimalPath), // makespan for product
                //         (endTimeProduct - startTimeProduct) / 1e6,  // convert from nanoseconds to milliseconds
                //         memUsedProduct
                //     );
                //     // Compute derived metrics after setting full product automaton metrics
                //     allocAlg->getMetrics().computeDerivedMetrics();
                // }

                allocAlg->getMetrics().printSummary();
                
                // Store run in TestRunManager
                // average_capabilities is the independent variable (varies within each CSV)
                map<string, string> parameters;
                parameters["automaton_id"] = to_string(automatonId);
                parameters["num_robots"] = to_string(robotCount);
                parameters["average_capabilities"] = to_string(aveCap);
                
                manager.storeRun(
                    allocAlg->getMetrics(),
                    parameters,
                    to_string(aveCap),  // independent variable: average capabilities varies within each CSV
                    1  // trial number
                );
                
                delete allocAlg;
                delete buchi;
                
            } catch (const exception& e) {
                cout << "ERROR: " << e.what() << endl;
            }
            
            testNum++;
            
            // Cleanup after each test
            delete mrs;
            delete env;
            delete grid;
            delete ts;
        }
        
        cout << "\n" << string(80, '=') << endl;
        cout << "   AUTOMATON " << automatonId << " CONFIGURATION COMPLETE" << endl;
        cout << string(80, '=') << "\n" << endl;
    }
}
    cout << "\n" << string(80, '=') << "\n" << endl;
    cout << "✓ All tests completed!" << endl;
    cout << "   - 144 total tests executed (6 automata × 3 robot counts × 8 average capabilities)" << endl;
    cout << "   - Generating 18 CSV files (6 automata × 3 robot counts)" << endl;
    
    // Export results - ONCE after all data collected
    cout << "\n✓ Exporting results by automaton_id and num_robots configuration..." << endl;
    manager.exportByConfiguration("data", "average_capabilities");
    
    // Export final statistics
    cout << "\n✓ Exporting final statistics..." << endl;
    manager.exportStatisticsToCSV("data/statistics.csv");
    manager.exportSummaryReport("data/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ CSV Results stored in data/ folder:" << endl;
    cout << "   18 CSV files (6 automata × 3 robot counts):" << endl;
    cout << "   Each CSV contains rows for average_capabilities: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0" << endl;
    cout << "\n   Automaton 1:" << endl;
    cout << "   - automaton_states_automaton_id_1_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_1_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_1_num_robots_16.csv" << endl;
    cout << "   Automaton 2:" << endl;
    cout << "   - automaton_states_automaton_id_2_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_2_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_2_num_robots_16.csv" << endl;
    cout << "   Automaton 3:" << endl;
    cout << "   - automaton_states_automaton_id_3_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_3_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_3_num_robots_16.csv" << endl;
    cout << "   Automaton 4:" << endl;
    cout << "   - automaton_states_automaton_id_4_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_4_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_4_num_robots_16.csv" << endl;
    cout << "   Automaton 5:" << endl;
    cout << "   - automaton_states_automaton_id_5_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_5_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_5_num_robots_16.csv" << endl;
    cout << "   Automaton 6:" << endl;
    cout << "   - automaton_states_automaton_id_6_num_robots_6.csv" << endl;
    cout << "   - automaton_states_automaton_id_6_num_robots_10.csv" << endl;
    cout << "   - automaton_states_automaton_id_6_num_robots_16.csv" << endl;
    cout << "\n✓ Statistics and summary stored in data/" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    return 0;
}


// REMOVED: createTestInfiniteBuchiAutomaton1 (originally test 1)

/**
 * Test 2: Nested Next Operators with Sequencing → RENAMED TO 1
 * Combines infinitely-often with chained next operators
 * Complexity: 4 APs, 6 Automaton States
 * G(F("p0" & X("p1" & X"p2"))) & G(F("p3"))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p2\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_infinite_1.dot");
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton1
// REMOVED: createTestInfiniteBuchiAutomaton2 placeholder (now createTestInfiniteBuchiAutomaton1)

/**
 * Test 3: Mixed Next and Until Operators → RENAMED TO 2
 * Combines infinitely-often with until (weak until) patterns
 * Complexity: 5 APs, 10 Automaton States
 * G(F("p0")) & G(F("p1" & X("p2"))) & G(F(!"p3" U "p4") & G(F("p3")))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton2() {
    string ltl_str = "G(F(\"p0\" & X(\"p1\" & X\"p2\"))) & G(F(\"p3\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// REMOVED: createTestInfiniteBuchiAutomaton3 (originally test 3)
// This was moved to createTestInfiniteBuchiAutomaton2

/**
 * Test 4: Until with Disjunctive Branching → RENAMED TO 3
 * Introduces disjunction at top level with complex nested structure
 * Complexity: 5 APs, 16 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) | G(F("p4" & X("p0"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p1\" & X(\"p2\"))) & G(F(!\"p3\" U \"p4\") & G(F(\"p3\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton3
// REMOVED: createTestInfiniteBuchiAutomaton4 placeholder (now createTestInfiniteBuchiAutomaton3)

/**
 * Test 5: Multiple Sequential Until Conditions → RENAMED TO 4
 * Deep nesting of until operators with complex boolean combinations
 * Complexity: 10 APs, 20 Automaton States
 * G((F(!"p0" U ("p1" & F("p2"))) & G(F("p0")) & G(F("p3")) & F(!"p3" U ("p4" & F("p5"))) & F("p3") & F("p6" & X("p7")) & G(F("p8")) & G(F(!"p8" U "p9"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) | G(F(\"p4\" & X(\"p0\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// REMOVED: createTestInfiniteBuchiAutomaton5 (originally test 5)
// This was moved to createTestInfiniteBuchiAutomaton4

/**
 * Test 7: Extended Formula with Infinitely-Often and Next Operators → RENAMED TO 5
 * Enhances Test 6 pattern with additional temporal constraints (p8, p9)
 * Complexity: 10 APs, 27 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) & G(F(("p8") & X("p9")))) | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton5() {
    string ltl_str = "(G((F(!\"p0\" U (\"p1\" & F(\"p2\"))) & G(F(\"p0\")) & G(F(\"p3\")) & F(!\"p3\" U (\"p4\" & F(\"p5\"))) & F(\"p3\") & F(\"p6\" & X(\"p7\")) & G(F(\"p8\")) & G(F(!\"p8\" U \"p9\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    for (int i = 0; i < 10; i++) {
        uint16_t tsState = i % 6;
        bool hasGPS = (i % 2 == 0);
        vector<bool> caps(13, false);
        if (hasGPS) caps[5] = true;
        if (i % 3 == 1) caps[0] = true;
        caps[5] = true;  // All have GPS
        
        batchAPs.push_back(BatchAtomicProposition(i, tsState, caps, 0));
    }
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton5
// REMOVED: createTestInfiniteBuchiAutomaton6 placeholder (now createTestInfiniteBuchiAutomaton5)

/**
 * Test 8: Standardized High-Complexity Formula (Disjunctive Pattern) → RENAMED TO 6
 * 18 APs, 38 Automaton States, until-based liveness properties, variant of Test 7 with OR instead of AND
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern throughout, disjunctive top-level
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton6() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p8\") & X(\"p9\")))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(8, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p8
    batchAPs.push_back(BatchAtomicProposition(9, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p9

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// REMOVED: createTestInfiniteBuchiAutomaton7 (originally test 8 - placeholder, actual moved to createTestInfiniteBuchiAutomaton6)

/**
 * REMOVED: Automaton 8
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton7_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton8 (originally test 6)

/**
 * REMOVED: Automaton 9
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton8_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton9 (originally test 9)

/**
 * REMOVED: Automaton 10
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton9_REMOVED() {
    return nullptr;  // Removed automaton
}
// REMOVED: createTestInfiniteBuchiAutomaton10 (originally test 10)

/**
 * REMOVED: Automaton 11
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton10_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton11 (originally test 11)

/**
 * REMOVED: Automaton 12
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton11_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: Automaton 12 functions 13-16 follow

/**
 * REMOVED: Automaton 13
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton12_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton13 (originally test 12)

/**
 * REMOVED: Automaton 14
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton13_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton14 (originally test 14)

/**
 * REMOVED: Automaton 15
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton14_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton15 (originally test 15)

/**
 * REMOVED: Automaton 16
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton15_REMOVED() {
    return nullptr;  // Removed automaton
}

// REMOVED: createTestInfiniteBuchiAutomaton16 (originally test 16)

/**
 * REMOVED: Automaton 16 - End of automata implementations
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton16_REMOVED() {
    return nullptr;  // Removed automaton
}

// ============================================================================
//Create test environment with TS and GridWorld with 6 robots and 6 regions
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, int robotCount, double aveCap) {
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
    env->mapTSStateToGrid(0, Point(180, 140), 60, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 35), 60, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(120, 105), 60, 210);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(45, 35), 90, 70);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(45, 105), 90, 70);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(45, 175), 90, 70);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 6 robots
    mrs = new MultiRobotSystem();
    // Determine if the average capability is a whole number
    bool wholeAveCap = (static_cast<int>(2*aveCap) % 2 == 0);
    
    // Position 15 robots in a 3x2 grid, directly adjacent (1-unit spacing)
    // Grid starts at (160, 80) in room 0
    for (int i = 1; i <= robotCount; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4 vertical
        int x = 160 + col;
        int y = 80 + row;
        //initialize robot
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        
        //toggle number of capabilites per robot if avecap is not a whole number num robots must be even
        if (!wholeAveCap) {
            if (i % 2 == 0) {
                aveCap = static_cast<int>(2*aveCap) / 2;  // Toggle to half if not whole number
            } else {
                aveCap = static_cast<int>(2*aveCap + 1) / 2;  // Toggle to the other half if not whole number
            }
        }
        for (int j = 0; j < aveCap; j++){
            // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
            RobotCapability cap = (i+j % 6 == 1) ? RobotCapability::SENSOR_GPS : 
                                (i+j % 6 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                                (i+j % 6 == 3) ? RobotCapability::SENSOR_CAMERA :
                                (i+j % 6 == 4) ? RobotCapability::MANIPULATION_GRIPPER :
                                (i+j % 6 == 5) ? RobotCapability::MANIPULATION_TOOL :
                                RobotCapability::CAPABILITY_PAYLOAD;
            r->enableCapability(cap);    
        }
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots in a 3x2 grid" << endl;
}

// ============================================================================
//Create test environment with TS and GridWorld with 3 robots and 6 regions
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, double ave_cap) {
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
    env->mapTSStateToGrid(0, Point(180, 140), 60, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 35), 60, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(120, 105), 60, 210);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(45, 35), 90, 70);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(45, 105), 90, 70);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(45, 175), 90, 70);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 15 robots
    mrs = new MultiRobotSystem();
    
    // Position 3 robots in a 3x1 grid, directly adjacent (1-unit spacing)
    // Grid starts at (160, 80) in room 0
    for (int i = 1; i <= 3; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0 vertical
        int x = 160 + col;
        int y = 80 + row;
        
        // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    
    cout << "✓ MultiRobotSystem created with 3 robots" << endl;
}

// ============================================================================
//Create test environment with TS and GridWorld with 15 robots and 6 regions
void createTestEnvironment15(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, double ave_cap) {
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
    env->mapTSStateToGrid(0, Point(180, 140), 60, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 35), 60, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(120, 105), 60, 210);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(45, 35), 90, 70);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(45, 105), 90, 70);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(45, 175), 90, 70);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 15 robots
    mrs = new MultiRobotSystem();
    
    // Position 15 robots in a 3x5 grid, directly adjacent (1-unit spacing)
    // Grid starts at (160, 80) in room 0
    for (int i = 1; i <= 15; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4 vertical
        int x = 160 + col;
        int y = 80 + row;
        
        // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    
    cout << "✓ MultiRobotSystem created with 15 robots" << endl;
}


// ============================================================================
//Create test environment with TS and GridWorld with 15 robots and 6 regions
void createTestEnvironment45(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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
    env->mapTSStateToGrid(0, Point(180, 140), 60, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 35), 60, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(120, 105), 60, 210);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(45, 35), 90, 70);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(45, 105), 90, 70);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(45, 175), 90, 70);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 15 robots
    mrs = new MultiRobotSystem();
    
    // Position 45 robots in a 3x15 grid, directly adjacent (1-unit spacing)
    // Grid starts at (160, 80) in room 0
    for (int i = 1; i <= 45; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-14 vertical
        int x = 160 + col;
        int y = 80 + row;
        
        // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    
    cout << "✓ MultiRobotSystem created with 45 robots" << endl;
}

