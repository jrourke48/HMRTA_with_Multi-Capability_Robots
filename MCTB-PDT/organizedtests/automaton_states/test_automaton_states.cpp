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
// Test: Number of Automaton States: 16 total automatons with states ranging from (3-256)
//=================================================================================
//Environments: 3-robot, 6-robot, 15-robot, and 45-robot all with 6 TS regions each with only one capability per robot
//

// Forward declarations
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment15(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment45(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestInfiniteBuchiAutomaton1();
BuchiAutomaton* createTestInfiniteBuchiAutomaton2();
BuchiAutomaton* createTestInfiniteBuchiAutomaton3();
BuchiAutomaton* createTestInfiniteBuchiAutomaton4();
BuchiAutomaton* createTestInfiniteBuchiAutomaton5();
BuchiAutomaton* createTestInfiniteBuchiAutomaton6();
BuchiAutomaton* createTestInfiniteBuchiAutomaton7();
BuchiAutomaton* createTestInfiniteBuchiAutomaton8();
BuchiAutomaton* createTestInfiniteBuchiAutomaton9();
BuchiAutomaton* createTestInfiniteBuchiAutomaton10();
BuchiAutomaton* createTestInfiniteBuchiAutomaton11();
BuchiAutomaton* createTestInfiniteBuchiAutomaton12();
BuchiAutomaton* createTestInfiniteBuchiAutomaton13();
BuchiAutomaton* createTestInfiniteBuchiAutomaton14();
BuchiAutomaton* createTestInfiniteBuchiAutomaton15();
BuchiAutomaton* createTestInfiniteBuchiAutomaton16();


// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   AUTOMATON SCALING TEST SUITE" << endl;
    cout << "   16 Büchi Automata" << endl;
    cout << "   4 Environments: 3-robot, 6-robot, 15-robot, and 45-robot" << endl;
    cout << "   Total Tests: 64 (16 automata × 4 environments)" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Initialize TestRunManager for AUTOMATON_STATES category
    TestRunManager manager(TestRunManager::TestCategory::AUTOMATON_STATES, ".");
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
        createTestInfiniteBuchiAutomaton8,
        createTestInfiniteBuchiAutomaton9,
        createTestInfiniteBuchiAutomaton10,
        createTestInfiniteBuchiAutomaton11,
        createTestInfiniteBuchiAutomaton12,
        createTestInfiniteBuchiAutomaton13,
        createTestInfiniteBuchiAutomaton14,
        createTestInfiniteBuchiAutomaton15,
        createTestInfiniteBuchiAutomaton16
    };
    
    vector<int> robotCounts = {3, 6, 15, 45};
    int testNum = 1;
    
    // Run tests for each environment
    for (int robotCount : robotCounts) {
        cout << "\n" << string(80, '=') << endl;
        cout << "   TESTING WITH " << robotCount << "-ROBOT ENVIRONMENT" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        // Create test environment
        TS* ts = nullptr;
        GridWorld* grid = nullptr;
        Environment* env = nullptr;
        MultiRobotSystem* mrs = nullptr;
        
        if (robotCount == 3) {
            createTestEnvironment3(ts, grid, env, mrs);
        } else if (robotCount == 6) {
            createTestEnvironment6(ts, grid, env, mrs);
        } else if (robotCount == 15) {
            createTestEnvironment15(ts, grid, env, mrs);
        } else {
            createTestEnvironment45(ts, grid, env, mrs);
        }
        
        cout << "\n" << string(80, '-') << endl;
        cout << "   RUNNING TESTS" << endl;
        cout << string(80, '-') << "\n" << endl;
        
        // For each of the 16 automata
        for (int automatonId = 1; automatonId <= 16; ++automatonId) {
            cout << "\n  Test " << testNum << " (Automaton " << automatonId << ")... ";
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
                    std::cout << "DEBUG: Creating ProductAutomaton for automaton " << automatonId << "..." << std::endl;
                    ProductAutomaton productAutomaton(*env, *mrs, *buchi);
                    std::cout << "DEBUG: ProductAutomaton created successfully. Calling OptimalAcceptingPath()..." << std::endl;
                    std::tuple<std::vector<uint16_t>, uint32_t> optimalPath = productAutomaton.OptimalAcceptingPath();
                    std::cout << "DEBUG: OptimalAcceptingPath() completed successfully" << std::endl;
                    double memAfterProduct = getMemoryUsageMB();
                    double endTimeProduct = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    double memUsedProduct = memAfterProduct - memBeforeProduct;
                    
                    //add the full product automaton metrics to the algorithm metrics
                    allocAlg->getMetrics().setFullProductAutomatonMetrics(
                        productAutomaton.getNumStates(),
                        productAutomaton.getNumEdges(),
                        std::get<1>(optimalPath), // makespan for product
                        (endTimeProduct - startTimeProduct) / 1e6,  // convert from nanoseconds to milliseconds
                        memUsedProduct
                    );
                    // Compute derived metrics after setting full product automaton metrics
                    allocAlg->getMetrics().computeDerivedMetrics();
                }

                allocAlg->getMetrics().printSummary();
                
                // Store run in TestRunManager
                map<string, string> parameters;
                parameters["automaton_id"] = to_string(automatonId);
                parameters["num_robots"] = to_string(robotCount);
                
                manager.storeRun(
                    allocAlg->getMetrics(),
                    parameters,
                    to_string(automatonId),  // independent variable: group by automaton
                    1  // trial number
                );
                
                delete allocAlg;
                delete buchi;
                
            } catch (const exception& e) {
                cout << "ERROR: " << e.what() << endl;
            }
            
            testNum++;
        }
        //export the csv for the test run manager

        
        cout << "\n" << string(80, '=') << endl;
        cout << "   ENVIRONMENT TESTING COMPLETE" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        // Export results for this configuration
        manager.exportByConfiguration();
        
        // Cleanup
        delete mrs;
        delete env;
        delete grid;
        delete ts;
    }

    cout << "\n" << string(80, '=') << "\n" << endl;
    cout << "✓ All tests completed!" << endl;
    cout << "   - 16 automata tested with current robot configuration(s)" << endl;
    cout << "   - When all 4 configs enabled (3, 6, 15, 45 robots): 64 total tests" << endl;
    
    // Export final statistics
    cout << "\n✓ Exporting final statistics..." << endl;
    manager.exportStatisticsToCSV("data/statistics.csv");
    manager.exportSummaryReport("data/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ CSV Results stored in data/ folder:" << endl;
    cout << "   - automaton_states_num_robots_3.csv" << endl;
    cout << "   - automaton_states_num_robots_6.csv" << endl;
    cout << "   - automaton_states_num_robots_15.csv" << endl;
    cout << "   - automaton_states_num_robots_45.csv" << endl;
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
    
    // Store LTLFormula in static variable to keep it alive for the entire program
    static LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
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

    static LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
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
 * Test 5: Multiple Sequential Until Conditions
 * Deep nesting of until operators with complex boolean combinations
 * Complexity: 10 APs, 20 Automaton States
 * G((F(!"p0" U ("p1" & F("p2"))) & G(F("p0")) & G(F("p3")) & F(!"p3" U ("p4" & F("p5"))) & F("p3") & F("p6" & X("p7")) & G(F("p8")) & G(F(!"p8" U "p9"))))
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

/**
 * Test 7: Extended Formula with Infinitely-Often and Next Operators
 * Enhances Test 6 pattern with additional temporal constraints (p8, p9)
 * Complexity: 10 APs, 27 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) & G(F("p8")) & X("p9") | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
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

/**
 * Test 8: Standardized High-Complexity Formula (Disjunctive Pattern)
 * 18 APs, 38 Automaton States, until-based liveness properties, variant of Test 7 with OR instead of AND
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern throughout, disjunctive top-level
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton7() {
    string ltl_str = "(G(F(!\"p0\" U \"p1\")) & G(F(\"p0\")) & G(F(\"p2\")) & G(F(!\"p2\" U \"p3\")) & G(F(!\"p4\" U \"p5\")) & G(F(!\"p6\" U \"p7\")) & G(F(!\"p8\" U \"p9\")) & G(F(!\"p10\" U \"p11\") | F(!\"p12\" U \"p13\")) & G(F(\"p14\" & X(\"p15\" & X(\"p16\" & X(\"p17\")))))))";
    
    vector<BatchAtomicProposition> batchAPs;
    for (int i = 0; i < 18; i++) {
        uint16_t tsState = i % 6;
        vector<bool> caps(13, false);
        caps[5] = true;  // All have GPS
        if (i % 3 == 0) caps[0] = true;  // Some have movement
        if (i % 4 == 0) caps[3] = true;  // Some have camera
        
        batchAPs.push_back(BatchAtomicProposition(i, tsState, caps, 0));
    }

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_9.dot");
    return buchi;
}

/**
 * Test 6: Conjunctive-Disjunctive Mixed Operators
 * Combines multiple conjunctions with disjunction, nested until and next
 * Complexity: 8 APs, 39 Automaton States
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton8() {
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
 * Test 9: Standardized High-Complexity Formula (Conjunctive Pattern)
 * 11 APs, 40 Automaton States, until-based liveness properties, variant of Test 8 with AND instead of OR
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton9() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & (!\"p9\" U \"p10\")";
    
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
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_9.dot");
    return buchi;
}
/**
 * Test 10: Standardized High-Complexity Formula (Variant 1)
 * Complexity: 12 APs, 43 Automaton States, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton10() {
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
 * Test 11: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 14 APs, 55 Automaton States, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton11() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p12\") & X(\"p13\"))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & (!\"p9\" U \"p10\")";
    
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
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 12: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 15 APs, 61 Automaton States, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton12() {
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

/**
 * Test 12: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 7 APs, 96 Automaton States
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton13() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (!(\"p1\") U (\"p2\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 14: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 7 APs, 128 Automaton States
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton14() { 
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 15: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 8 APs, 192 Automaton States
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton15() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (F(\"p8\")) & (!(\"p1\") U (\"p2\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(8, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 16: Standardized High-Complexity Formula (Variant 2)
 * Complexity: 8 APs, 256 Automaton States
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton16() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (F(\"p8\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(8, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// ============================================================================
//Create test environment with TS and GridWorld with 6 robots and 6 regions
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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
    
    // Position 15 robots in a 3x2 grid, directly adjacent (1-unit spacing)
    // Grid starts at (160, 80) in room 0
    for (int i = 1; i <= 6; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4 vertical
        int x = 180 + col;
        int y = 140 + row;
        
        // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
    cout << "✓ MultiRobotSystem created with 6 robots in a 3x2 grid" << endl;
}

// ============================================================================
//Create test environment with TS and GridWorld with 3 robots and 6 regions
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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
        int x = 180 + col;
        int y = 140 + row;
        
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
void createTestEnvironment15(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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
    // Grid starts at (180, 140) in room 0
    for (int i = 1; i <= 15; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4 vertical
        int x = 180 + col;
        int y = 140 + row;
        
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
    // Grid starts at (180, 140) in room 0
    for (int i = 1; i <= 45; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-14 vertical
        int x = 180 + col;
        int y = 140 + row;
        
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

