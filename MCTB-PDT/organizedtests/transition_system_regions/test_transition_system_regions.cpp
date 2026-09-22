#include <iostream>
#include <vector>
#include <tuple>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <cmath>
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
// Test: Transition System Regions Scaling: 8 different region counts 6-40
//=================================================================================
//Automata: 8 total automata 3-128 automaton states all with 6 robots each with only one capability per robot
//

// Forward declarations
void createTestEnvironmentRegions6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions12(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions18(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions24(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions30(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions36(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions38(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironmentRegions40(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestInfiniteBuchiAutomaton1();
BuchiAutomaton* createTestInfiniteBuchiAutomaton2();
BuchiAutomaton* createTestInfiniteBuchiAutomaton3();
BuchiAutomaton* createTestInfiniteBuchiAutomaton4();
BuchiAutomaton* createTestInfiniteBuchiAutomaton5();
BuchiAutomaton* createTestInfiniteBuchiAutomaton6();
BuchiAutomaton* createTestInfiniteBuchiAutomaton7();
BuchiAutomaton* createTestInfiniteBuchiAutomaton8();


// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   TRANSITION SYSTEM REGIONS SCALING TEST SUITE" << endl;
    cout << "   8 Büchi Automata" << endl;
    cout << "   8 Environments: 6-region, 12-region, 18-region, 24-region, 30-region, 36-region, 38-region, 40-region (all with 6 robots)" << endl;
    cout << "   Total Tests: 64 (8 automata × 8 region counts)" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Initialize TestRunManager for TS_REGIONS category
    TestRunManager manager(TestRunManager::TestCategory::TS_REGIONS, ".");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    // Create array of automaton factory functions
    vector<BuchiAutomaton*(*)()> automatonFactories = {
        createTestInfiniteBuchiAutomaton1,
        createTestInfiniteBuchiAutomaton2,
        createTestInfiniteBuchiAutomaton3,
        createTestInfiniteBuchiAutomaton4,
        createTestInfiniteBuchiAutomaton5,
        createTestInfiniteBuchiAutomaton6,
        createTestInfiniteBuchiAutomaton7,
        createTestInfiniteBuchiAutomaton8
    };
    
    vector<int> regionCounts = {6, 12, 18, 24, 30, 36, 38, 40};
    int testNum = 1;
    
    // Run tests for each automaton (outer loop)
    for (int automatonId = 1; automatonId <= 8; ++automatonId) {
        cout << "\n" << string(80, '=') << endl;
        cout << "   TESTING WITH AUTOMATON " << automatonId << endl;
        cout << string(80, '=') << "\n" << endl;
        
        // For each region count (inner loop)
        for (int regionCount : regionCounts) {
            cout << "\n  Test " << testNum << " (" << regionCount << "-region environment)... ";
            cout.flush();
            
            // Create test environment
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            
            if (regionCount == 6) {
                createTestEnvironmentRegions6(ts, grid, env, mrs);
            } else if (regionCount == 12) {
                createTestEnvironmentRegions12(ts, grid, env, mrs);
            } else if (regionCount == 18) {
                createTestEnvironmentRegions18(ts, grid, env, mrs);
            } else if (regionCount == 24) {
                createTestEnvironmentRegions24(ts, grid, env, mrs);
            } else if (regionCount == 30) {
                createTestEnvironmentRegions30(ts, grid, env, mrs);
            } else if (regionCount == 36) {
                createTestEnvironmentRegions36(ts, grid, env, mrs);
            } else if (regionCount == 38) {
                createTestEnvironmentRegions38(ts, grid, env, mrs);
            } else {
                createTestEnvironmentRegions40(ts, grid, env, mrs);
            }
            
            try {
                // Create the Buchi automaton
                BuchiAutomaton* buchi = automatonFactories[automatonId - 1]();
                
                if (!buchi) {
                    cout << "ERROR: Failed to create automaton" << endl;
                    testNum++;
                    delete mrs;
                    delete env;
                    delete grid;
                    delete ts;
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
                
                allocAlg->getMetrics().printSummary();
                
                // Store run in TestRunManager
                map<string, string> parameters;
                parameters["automaton_id"] = to_string(automatonId);
                parameters["num_ts_regions"] = to_string(regionCount);
                
                manager.storeRun(
                    allocAlg->getMetrics(),
                    parameters,
                    to_string(regionCount),  // independent variable: group by region count
                    1  // trial number
                );
                
                delete allocAlg;
                delete buchi;
                
            } catch (const exception& e) {
                cout << "ERROR: " << e.what() << endl;
            }
            
            // Cleanup for this iteration
            delete mrs;
            delete env;
            delete grid;
            delete ts;
            
            testNum++;
        }
        
        // Export results for this automaton
        cout << "\n✓ Exporting results for automaton " << automatonId << "..." << endl;
        cout << "  Current runs stored: " << manager.getCurrentNumberOfRuns() << endl;
        manager.exportByConfiguration("data", "num_ts_regions");
        cout << "✓ Export complete\n" << endl;
        
        cout << "\n" << string(80, '=') << endl;
        cout << "   AUTOMATON " << automatonId << " TESTING COMPLETE" << endl;
        cout << string(80, '=') << "\n" << endl;
    }

    cout << "\n" << string(80, '=') << "\n" << endl;
    cout << "✓ All tests completed!" << endl;
    cout << "   - 8 robot configuration tested with 8 automata: 64 total tests" << endl;;
    
    // Export final statistics
    cout << "\n✓ Exporting final statistics..." << endl;
    manager.exportStatisticsToCSV("data/statistics.csv");
    manager.exportSummaryReport("data/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ CSV Results stored in data/ folder:" << endl;
    cout << "   - num_robots_3.csv" << endl;
    cout << "   - num_robots_6.csv" << endl;
    cout << "   - num_robots_12.csv" << endl;
    cout << "   - num_robots_18.csv" << endl;
    cout << "   - num_robots_24.csv" << endl;
    cout << "   - num_robots_30.csv" << endl;
    cout << "   - num_robots_36.csv" << endl;
    cout << "   - num_robots_45.csv" << endl;
    cout << "\n✓ Statistics and summary stored in data/" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    return 0;
}


/**
 * Test 1: Basic Conjunctive Liveness
 * Simple conjunction of two infinitely-often conditions
 * Complexity: 2 APs, 3 Automaton States
 * G(F("p0")) & G(F("p2"))
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

/**
 * Test 2: Nested Next Operators with Sequencing
 * Combines infinitely-often with chained next operators
 * Complexity: 4 APs, 6 Automaton States
 * G(F("p0" & X("p1" & X"p2"))) & G(F("p3"))
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

/**
 * Test 3: Mixed Next and Until Operators
 * Combines infinitely-often with until (weak until) patterns
 * Complexity: 5 APs, 10 Automaton States
 * G(F("p0")) & G(F("p1" & X("p2"))) & G(F(!"p3" U "p4") & G(F("p3")))
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

/**
 * Test 4: Until with Disjunctive Branching
 * Introduces disjunction at top level with complex nested structure
 * Complexity: 5 APs, 16 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) | G(F("p4" & X("p0"))))
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

/**
 * Test 5: Extended Formula with Infinitely-Often and Next Operators
 * Enhances Test 6 pattern with additional temporal constraints (p8, p9)
 * Complexity: 10 APs, 27 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) & G(F("p8")) & X("p9") | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton5() {
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

/**
 * Test 6: Conjunctive-Disjunctive Mixed Operators
 * Combines multiple conjunctions with disjunction, nested until and next
 * Complexity: 8 APs, 39 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton6() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 7: Standardized High-Complexity Formula (Variant 1)
 * Complexity: 12 APs, 43 Automaton States, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton7() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    batchAPs.push_back(BatchAtomicProposition(11, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p11
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 8: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 15 APs, 61 Automaton States, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton8() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p12\") & X(\"p13\"))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & F(\"p14\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    batchAPs.push_back(BatchAtomicProposition(11, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p11
    batchAPs.push_back(BatchAtomicProposition(12, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p12
    batchAPs.push_back(BatchAtomicProposition(13, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p13
    batchAPs.push_back(BatchAtomicProposition(14, 5, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p14

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// ============================================================================
// Create test environment with TS and GridWorld with 6 regions and 6 robots
// Hub-spoke topology: 1 central hub + 5 spokes
void createTestEnvironmentRegions6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    // Create hub (node 2) and 5 spokes
    Node* hub = new Node(2, "Hub");
    Node* nodes[6];
    nodes[2] = hub;
    
    for (int i = 0; i < 6; i++) {
        if (i != 2) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    // Connect spokes to hub
    for (int i = 0; i < 6; i++) {
        if (i != 2) {
            nodes[i]->addEdge(Edge(2));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 6; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    // Map 6 states to grid regions
    env->mapTSStateToGrid(0, Point(380, 340), 40, 80);
    env->mapTSStateToGrid(1, Point(380, 80), 40, 80);
    env->mapTSStateToGrid(2, Point(210, 210), 60, 420);
    env->mapTSStateToGrid(3, Point(40, 80), 80, 160);
    env->mapTSStateToGrid(4, Point(40, 210), 80, 160);
    env->mapTSStateToGrid(5, Point(40, 340), 80, 160);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (6-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 12 regions and 6 robots
void createTestEnvironmentRegions12(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    // Hub-spoke: hub at node 6, 11 spokes
    Node* hub = new Node(6, "Hub");
    vector<Node*> nodes(12);
    nodes[6] = hub;
    
    for (int i = 0; i < 12; i++) {
        if (i != 6) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 12; i++) {
        if (i != 6) {
            nodes[i]->addEdge(Edge(6));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 12; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    // Distribute 12 regions in circle around grid
    for (int i = 0; i < 12; i++) {
        if (i == 6) continue; // Hub in center
        double angle = (i < 6 ? i : i - 1) * (2 * M_PI / 11);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 60, 120);
    }
    env->mapTSStateToGrid(6, Point(210, 210), 100, 420); // Hub in center
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (12-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 18 regions and 6 robots
void createTestEnvironmentRegions18(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(9, "Hub");
    vector<Node*> nodes(18);
    nodes[9] = hub;
    
    for (int i = 0; i < 18; i++) {
        if (i != 9) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 18; i++) {
        if (i != 9) {
            nodes[i]->addEdge(Edge(9));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 18; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 18; i++) {
        if (i == 9) continue;
        double angle = (i < 9 ? i : i - 1) * (2 * M_PI / 17);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 50, 100);
    }
    env->mapTSStateToGrid(9, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (18-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 24 regions and 6 robots
void createTestEnvironmentRegions24(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(12, "Hub");
    vector<Node*> nodes(24);
    nodes[12] = hub;
    
    for (int i = 0; i < 24; i++) {
        if (i != 12) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 24; i++) {
        if (i != 12) {
            nodes[i]->addEdge(Edge(12));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 24; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 24; i++) {
        if (i == 12) continue;
        double angle = (i < 12 ? i : i - 1) * (2 * M_PI / 23);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 40, 80);
    }
    env->mapTSStateToGrid(12, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (24-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 30 regions and 6 robots
void createTestEnvironmentRegions30(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(15, "Hub");
    vector<Node*> nodes(30);
    nodes[15] = hub;
    
    for (int i = 0; i < 30; i++) {
        if (i != 15) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 30; i++) {
        if (i != 15) {
            nodes[i]->addEdge(Edge(15));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 30; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 30; i++) {
        if (i == 15) continue;
        double angle = (i < 15 ? i : i - 1) * (2 * M_PI / 29);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 35, 70);
    }
    env->mapTSStateToGrid(15, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (30-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 36 regions and 6 robots
void createTestEnvironmentRegions36(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(18, "Hub");
    vector<Node*> nodes(36);
    nodes[18] = hub;
    
    for (int i = 0; i < 36; i++) {
        if (i != 18) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 36; i++) {
        if (i != 18) {
            nodes[i]->addEdge(Edge(18));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 36; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 36; i++) {
        if (i == 18) continue;
        double angle = (i < 18 ? i : i - 1) * (2 * M_PI / 35);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 30, 60);
    }
    env->mapTSStateToGrid(18, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (36-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 38 regions and 6 robots
void createTestEnvironmentRegions38(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(19, "Hub");
    vector<Node*> nodes(38);
    nodes[19] = hub;
    
    for (int i = 0; i < 38; i++) {
        if (i != 19) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 38; i++) {
        if (i != 19) {
            nodes[i]->addEdge(Edge(19));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 38; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 38; i++) {
        if (i == 19) continue;
        double angle = (i < 19 ? i : i - 1) * (2 * M_PI / 37);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 28, 56);
    }
    env->mapTSStateToGrid(19, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (38-region TS)" << endl;
}

// ============================================================================
// Create test environment with TS and GridWorld with 40 regions and 6 robots
void createTestEnvironmentRegions40(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(420, 420);
    ts = new TS();
    
    Node* hub = new Node(20, "Hub");
    vector<Node*> nodes(40);
    nodes[20] = hub;
    
    for (int i = 0; i < 40; i++) {
        if (i != 20) {
            nodes[i] = new Node(i, "R" + to_string(i));
        }
    }
    
    for (int i = 0; i < 40; i++) {
        if (i != 20) {
            nodes[i]->addEdge(Edge(20));
            hub->addEdge(Edge(i));
        }
    }
    
    for (int i = 0; i < 40; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    for (int i = 0; i < 40; i++) {
        if (i == 20) continue;
        double angle = (i < 20 ? i : i - 1) * (2 * M_PI / 39);
        int px = 210 + 150 * cos(angle);
        int py = 210 + 150 * sin(angle);
        env->mapTSStateToGrid(i, Point(px, py), 26, 52);
    }
    env->mapTSStateToGrid(20, Point(210, 210), 100, 420);
    
    mrs = new MultiRobotSystem();
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;
        int row = (i - 1) / 3;
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(380 + col, 340 + row));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots (40-region TS)" << endl;
}
