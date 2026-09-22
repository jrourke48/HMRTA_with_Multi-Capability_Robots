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
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, int robotCount, double homogeneity);
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
    cout << "   ROBOT HOMOGENEITY TEST SUITE" << endl;
    cout << "   6 Büchi Automata (every other from 2-12)" << endl;
    cout << "   8 Robot Homogeneity Values: 0.2, 0.6, 1.0, 1.4, 1.8, 2.2, 2.6, 3.0" << endl;
    cout << "   Homogeneity = Independent Capabilities / Num Robots (no double-counting)" << endl;
    cout << "   Total Tests: 144 (6 automata × 8 homogeneity values × 3 robot counts)" << endl;
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
    
    vector<int> robotCounts = {6, 10, 16};
    // Dynamic homogeneity ranges based on robot count: max = 12 / robotCount
    map<int, vector<double>> homogeneityRanges = {
        {6, {0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0}},     // 6 robots: 0.4-2.0
        {10, {0.2, 0.4, 0.6, 0.8, 1.0, 1.2}},                    // 10 robots: 0.2-1.2
        {16, {0.125, 0.25, 0.375, 0.5, 0.625, 0.75}}            // 16 robots: 0.125-0.75
    };
    int testNum = 1;
    
    // Run tests: automatonId outer, robotCount middle, homogeneity inner
    // This ensures each (automatonId, robotCount) pair gets all 8 homogeneity values
    for (int automatonId = 1; automatonId <= 6; ++automatonId) {
        for (int robotCount : robotCounts) {
        cout << "\n" << string(80, '=') << endl;
        cout << "   AUTOMATON " << automatonId << " WITH " << robotCount << "-ROBOT ENVIRONMENT" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        cout << "\n" << string(80, '-') << endl;
        cout << "   RUNNING TESTS (robot_homogeneity varying)" << endl;
        cout << string(80, '-') << "\n" << endl;
        
        // For each robot count's dynamic homogeneity range
        for (double homogeneity : homogeneityRanges[robotCount]) {
            // Create test environment for this configuration
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            createTestEnvironment(ts, grid, env, mrs, robotCount, homogeneity);
            cout << "\n  Test " << testNum << " (Automaton " << automatonId << ", Robots " << robotCount << ", Homogeneity " << homogeneity << ")... ";
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
                bool shouldSkip = (robotCount > 5) && (buchi->getNumStates()*std::pow(ts->getNumStates(), robotCount) > UINT16_MAX/2);
                if (!shouldSkip) {
                    //buld the product automaton and store its metrics
                    double memBeforeProduct = getMemoryUsageMB();
                    double startTimeProduct = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    ProductAutomaton product(*env, *mrs, *buchi);
                    std::tuple<std::vector<uint16_t>, uint32_t> optimalPath = product.OptimalAcceptingPath();
                    double memAfterProduct = getMemoryUsageMB();
                    double endTimeProduct = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    double memUsedProduct = memAfterProduct - memBeforeProduct;
                    
                    //add the full product automaton metrics to the algorithm metrics
                    allocAlg->getMetrics().setFullProductAutomatonMetrics(
                        product.getNumStates(),
                        product.getNumEdges(),
                        std::get<1>(optimalPath), // makespan for product
                        (endTimeProduct - startTimeProduct) / 1e6,  // convert from nanoseconds to milliseconds
                        memUsedProduct
                    );
                    // Compute derived metrics after setting full product automaton metrics
                    allocAlg->getMetrics().computeDerivedMetrics();
                }

                allocAlg->getMetrics().printSummary();
                
                // Store run in TestRunManager
                // robot_homogeneity is the independent variable (varies within each CSV)
                map<string, string> parameters;
                parameters["automaton_id"] = to_string(automatonId);
                parameters["num_robots"] = to_string(robotCount);
                parameters["robot_homogeneity"] = to_string(homogeneity);
                
                manager.storeRun(
                    allocAlg->getMetrics(),
                    parameters,
                    to_string(homogeneity),  // independent variable: robot_homogeneity varies within each CSV
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
    cout << "   - 144 total tests executed (6 automata × 3 robot counts × 8 homogeneity values)" << endl;
    cout << "   - Generating 18 CSV files (6 automata × 3 robot counts)" << endl;
    
    // Export results - ONCE after all data collected
    cout << "\n✓ Exporting results by automaton_id and num_robots configuration..." << endl;
    manager.exportByConfiguration("data", "robot_homogeneity");
    
    // Export final statistics
    cout << "\n✓ Exporting final statistics..." << endl;
    manager.exportStatisticsToCSV("data/statistics.csv");
    manager.exportSummaryReport("data/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ CSV Results stored in data/ folder:" << endl;
    cout << "   18 CSV files (6 automata × 3 robot counts):" << endl;
    cout << "   Each CSV contains rows for robot_homogeneity: 0.2, 0.6, 1.0, 1.4, 1.8, 2.2, 2.6, 3.0" << endl;
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
 * Test 1: Simple Dual Properties → Requires 2-3 independent capabilities
 * G(F("p0")) & G(F("p1")) - 2 properties, 2 APs
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p1\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    // p0: uses capabilities 0, 1 (2 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, false, false, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 0, 2, 3 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, true, true, false, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_infinite_1.dot");
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton1
// REMOVED: createTestInfiniteBuchiAutomaton2 placeholder (now createTestInfiniteBuchiAutomaton1)

/**
 * Test 2: Three Properties with Sequencing → Requires 3-4 independent capabilities
 * G(F("p0")) & G(F("p1" & X("p2"))) - 3 properties, 4 APs
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton2() {
    string ltl_str = "G(F(\"p0\" & X(\"p1\" & X\"p2\"))) & G(F(\"p3\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    // p0: uses capabilities 0, 1, 2 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 1, 2 (2 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, true, true, false, false, false, false, false, false, false, false, false}, 0));
    // p2: uses capabilities 3, 4 (2 capabilities)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, true, false, false, false, false, false, false, false}, 0));
    // p3: uses capabilities 2, 3, 5 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, true, true, false, true, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// REMOVED: createTestInfiniteBuchiAutomaton3 (originally test 3)
// This was moved to createTestInfiniteBuchiAutomaton2

/**
 * Test 3: Five Properties with Mixed Operators → Requires 4-6 independent capabilities
 * G(F("p0")) & G(F("p1" & X("p2"))) & G(F("p3")) & G(F("p4")) - 5 properties
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p1\" & X(\"p2\"))) & G(F(\"p3\")) & G(F(\"p4\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    // p0: uses capabilities 0, 1, 2 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 0, 2, 3, 4 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, true, true, true, false, false, false, false, false, false, false}, 0));
    // p2: uses capabilities 1, 3, 5 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, true, false, true, false, true, false, false, false, false, false, false}, 0));
    // p3: uses capabilities 0, 2, 4 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // p4: uses capabilities 1, 2, 3, 5 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, true, true, true, false, true, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton3
// REMOVED: createTestInfiniteBuchiAutomaton4 placeholder (now createTestInfiniteBuchiAutomaton3)

/**
 * Test 4: Seven Properties with Sequential Conditions → Requires 6-8 independent capabilities
 * Complex formula with multiple temporal constraints
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p3\")) & G(F(\"p4\")) & G(F(\"p5\")) & G(F(\"p6\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    // p0: uses capabilities 0, 1, 2, 3 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, true, true, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 0, 2, 5 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, true, false, false, true, false, false, false, false, false, false}, 0));
    // p2: uses capabilities 1, 2, 4, 5 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, true, true, false, true, true, false, false, false, false, false, false}, 0));
    // p3: uses capabilities 0, 1, 3 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, true, false, true, false, false, false, false, false, false, false, false}, 0));
    // p4: uses capabilities 2, 3, 4 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, true, true, true, false, false, false, false, false, false, false}, 0));
    // p5: uses capabilities 0, 4, 6, 7 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, true, false, true, true, false, false, false, false}, 0));
    // p6: uses capabilities 1, 2, 5, 6 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(6, 2, {false, true, true, false, false, true, true, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// REMOVED: createTestInfiniteBuchiAutomaton5 (originally test 5)
// This was moved to createTestInfiniteBuchiAutomaton4

/**
 * Test 5: Ten Properties with Complex Temporal Constraints → Requires 8-10 independent capabilities
 * Extended formula with until patterns and nested temporal operators (sparse distribution)
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton5() {
    string ltl_str = "(G((F(!\"p0\" U (\"p1\" & F(\"p2\"))) & G(F(\"p0\")) & G(F(\"p3\")) & F(!\"p3\" U (\"p4\" & F(\"p5\"))) & F(\"p3\") & F(\"p6\" & X(\"p7\")) & G(F(\"p8\")) & G(F(!\"p8\" U \"p9\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    // 10 propositions using 8-10 capabilities (sparse distribution across caps 0-9)
    // p0: uses capabilities 0, 1, 2 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 3, 4 (2 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, true, false, false, false, false, false, false, false}, 0));
    // p2: uses capabilities 0, 5, 6 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, true, false, false, false, false, false}, 0));
    // p3: uses capabilities 1, 3, 7 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, true, false, true, false, false, false, true, false, false, false, false}, 0));
    // p4: uses capabilities 2, 4, 8 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, true, false, true, false, false, false, true, false, false, false}, 0));
    // p5: uses capabilities 5, 7, 9 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(5, 5, {false, false, false, false, false, true, false, true, false, true, false, false}, 0));
    // p6: uses capabilities 0, 3, 6, 8 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, true, false, false, true, false, true, false, false, false}, 0));
    // p7: uses capabilities 1, 2, 4, 9 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, true, true, false, true, false, false, false, false, true, false, false}, 0));
    // p8: uses capabilities 5, 6, 7 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(8, 3, {false, false, false, false, false, true, true, true, false, false, false, false}, 0));
    // p9: uses capabilities 0, 2, 3, 4, 8, 9 (6 capabilities)
    batchAPs.push_back(BatchAtomicProposition(9, 4, {true, false, true, true, true, false, false, false, true, true, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// This function was moved to createTestInfiniteBuchiAutomaton5
// REMOVED: createTestInfiniteBuchiAutomaton6 placeholder (now createTestInfiniteBuchiAutomaton5)

/**
 * Test 6: High-Complexity Formula with Disjunctive Patterns → Requires 12 independent capabilities
 * 10 propositions, complex temporal logic with until patterns and disjunctions (sparse distribution)
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton6() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p8\") & X(\"p9\")))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    // 10 propositions using all 12 capabilities from the 12-capability pool (sparse distribution)
    // p0: uses capabilities 0, 1, 2 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, true, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: uses capabilities 3, 4, 5 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, true, true, false, false, false, false, false, false}, 0));
    // p2: uses capabilities 0, 6, 7 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, false, true, true, false, false, false, false}, 0));
    // p3: uses capabilities 1, 8, 9 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, true, false, false, false, false, false, false, true, true, false, false}, 0));
    // p4: uses capabilities 2, 4, 10 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, true, false, true, false, false, false, false, false, true, false}, 0));
    // p5: uses capabilities 3, 5, 11 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(5, 5, {false, false, false, true, false, true, false, false, false, false, false, true}, 0));
    // p6: uses capabilities 0, 1, 6, 7, 8 (5 capabilities)
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, true, false, false, false, false, true, true, true, false, false, false}, 0));
    // p7: uses capabilities 2, 3, 9, 10 (4 capabilities)
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, false, true, true, false, false, false, false, false, true, true, false}, 0));
    // p8: uses capabilities 4, 5, 11 (3 capabilities)
    batchAPs.push_back(BatchAtomicProposition(8, 3, {false, false, false, false, true, true, false, false, false, false, false, true}, 0));
    // p9: uses capabilities 0, 1, 2, 6, 7, 10, 11 (7 capabilities)
    batchAPs.push_back(BatchAtomicProposition(9, 4, {true, true, true, false, false, false, true, true, false, false, true, true}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// ============================================================================
// Create test environment with TS and GridWorld - Robot Homogeneity variant
// Homogeneity = Independent capabilities / num_robots (no double-counting)
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs, int robotCount, double homogeneity) {
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
    
    // Create MultiRobotSystem with robots distributed for robot homogeneity
    mrs = new MultiRobotSystem();
    
    // Available capability types: 12 different robot capabilities
    vector<RobotCapability> capabilityPool = {
        RobotCapability::MOVEMENT_GROUND,          // 0
        RobotCapability::MOVEMENT_AERIAL,          // 1
        RobotCapability::MOVEMENT_AQUATIC,         // 2
        RobotCapability::SENSOR_CAMERA,            // 3
        RobotCapability::SENSOR_LIDAR,             // 4
        RobotCapability::SENSOR_GPS,               // 5
        RobotCapability::SENSOR_IMU,               // 6
        RobotCapability::SENSOR_PROXIMITY,         // 7
        RobotCapability::MANIPULATION_GRIPPER,     // 8
        RobotCapability::MANIPULATION_TOOL,        // 9
        RobotCapability::COMMUNICATION_WIFI,       // 10
        RobotCapability::COMMUNICATION_4G          // 11
    };
    
    // Calculate total independent capabilities needed
    // homogeneity = total_independent_caps / robotCount
    int totalCapabilities = static_cast<int>(round(homogeneity * robotCount));
    
    // Cap at maximum available capability types (12) - but only if it exceeds the pool size
    // For lower homogeneity values, we use fewer independent capabilities
    totalCapabilities = std::min(totalCapabilities, static_cast<int>(capabilityPool.size()));
    
    // Distribute totalCapabilities across robots: each robot gets at least 1 capability
    // This ensures no "capless" robots which would violate the homogeneity definition
    // KEY CHANGE: Use overlapping distribution so robots are more capable
    // Each robot gets ceil(totalCapabilities * 0.75) capabilities with overlap
    for (int i = 1; i <= robotCount; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4+ vertical
        int x = 180 + col;
        int y = 140 + row;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        
        if (totalCapabilities > 0) {
            // Each robot gets a larger subset of available capabilities (with overlap)
            // This makes robots more capable while keeping total independent caps the same
            int capsPerRobot = std::max(2, (totalCapabilities * 3) / 4);  // At least 75% of total
            capsPerRobot = std::min(capsPerRobot, totalCapabilities);      // Cap at total available
            
            // Each robot starts at offset (i-1) and wraps around
            for (int j = 0; j < capsPerRobot; j++) {
                int capIdx = (i - 1 + j) % totalCapabilities;
                r->enableCapability(capabilityPool[capIdx]);
            }
        }
        
        mrs->addRobot(r);
    }
    
    cout << "✓ MultiRobotSystem created with " << robotCount << " robots (homogeneity=" << homogeneity 
         << ", total_independent_caps=" << totalCapabilities << ")" << endl;
}
