#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/product.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twa/twagraph.hh>
#include <spot/twa/twaproduct.hh>
#include <spot/twaalgos/emptiness.hh>
#include <bddx.h>
#include "TS.h"
#include "BuchiAutomaton.h"
#include "ProductAutomaton.h"
#include "Environment/Environment.h"
#include "Environment/Point.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/Robot.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include "LTLFormula/LTLFormula.h"
#include "LTLFormula/BatchAtomicProposition.h"
#include "../MCTB-PDT/include/TaskAllocationAlgorithms.h"

int main()
{
    try {
        std::cout << "=== Product Automaton Scaling Analysis (1-6 Robots) ===" << std::endl;
        
        // Open CSV file for results
        std::ofstream csvFile("output/product_automaton_scaling.csv");
        csvFile << "num_robots,product_states,product_edges,accepting_states,state_ratio\n";
        
        // Constant setup
        GridWorld* grid = new GridWorld(210, 210);
        std::cout << "✓ GridWorld created (210x210)" << std::endl;
        
        // Allocate Transition System (same for all iterations)
        TS* ts = new TS();
        
        // Add 6 states 
        Node* node0 = new Node(0, "Room0");
        Node* node1 = new Node(1, "Room1");
        Node* node2 = new Node(2, "Room2");
        Node* node3 = new Node(3, "Room3");
        Node* node4 = new Node(4, "Room4");
        Node* node5 = new Node(5, "Room5");

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
        
        std::cout << "✓ Transition System created" << std::endl;
        std::cout << "  - States: " << ts->getNumStates() << std::endl;
        std::cout << "  - Initial state: 0" << std::endl;

        MultiRobotSystem* mrs1 = new MultiRobotSystem();
    
        // Position 6 robots in a 3x2 grid, directly adjacent (1-unit spacing)
        // Grid starts at (160, 80) in room 0
        for (int i = 0; i < 6; i++) {
            int col = (i) % 3;  // 0-2 horizontal
            int row = (i) / 3;  // 0-4 vertical
            int x = 160 + col;
            int y = 80 + row;
            
             //Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
             RobotCapability cap = (i % 3 == 0) ?   RobotCapability::MOVEMENT_GROUND: 
                                 (i % 3 == 1) ? RobotCapability::SENSOR_CAMERA: 
                                 RobotCapability::SENSOR_GPS;
            
            Robot* r = new Robot(i, "Rover_" + std::to_string(i), Point(x, y));
            r->initializeCapabilities(13);
            r->enableCapability(cap);
            std::cout << "Robot " << i << " initialized with capability " << capabilityToString(cap) << std::endl;
            mrs1->addRobot(r);
        }
        // Allocate Environment
        Environment* env = new Environment(ts, grid);
        std::cout << "✓ Environment created" << std::endl;
        
        // Map states to grid regions
        env->mapTSStateToGrid(0, Point(180, 140), 60, 140);    // State 0 centered at (180,140)
        env->mapTSStateToGrid(1, Point(180, 35), 60, 70);   // State 1 centered at (180,40)
        env->mapTSStateToGrid(2, Point(120, 105), 60, 210);   // State 2 centered at (100,100)
        env->mapTSStateToGrid(3, Point(45, 35), 90, 70);   // State 3 centered at (50,30)
        env->mapTSStateToGrid(4, Point(45, 105), 90, 70);   // State 4 centered at (50,100)
        env->mapTSStateToGrid(5, Point(45, 175), 90, 70);   // State 5 centered at (50,150)
        std::cout << "✓ Mapped 6 states to grid regions" << std::endl;
        
        // Add random obstacles for interesting pathfinding
        std::cout << "Adding random obstacles..." << std::endl;
        srand(time(0));
        int numObstacles = 15;
        for (int i = 0; i < numObstacles; i++) {
            int x = rand() % 20;
            int y = rand() % 20;
            env->addObstacle(Point(x, y));
        }
        std::cout << "✓ Added " << numObstacles << " random obstacles" << std::endl;
        
        // Create LTL formula (same for all iterations)
        std::string ltl_str = "(G(F(\"p1\")) & G(F(\"p2\")) & G(F(\"p3\")))";
        std::vector<BatchAtomicProposition> batchAPs;
        batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
        //batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        
        LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
        BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
        buchi->visualize("output/testing_buchi_automaton.png");
        std::cout << "✓ BuchiAutomaton created" << std::endl;
        std::cout << "  - States: " << buchi->getNumStates() << std::endl;
        std::cout << "  - Edges: " << buchi->getNumEdges() << "\n" << std::endl;
        
        // Loop through 1-6 robots
        std::cout << "=== Product Automaton Scaling Test ===" << std::endl;
        std::cout << "Testing with 1-6 robots...\n" << std::endl;

        // Create fresh MultiRobotSystem for this iteration
        MultiRobotSystem* mrs2 = new MultiRobotSystem();
        if (!mrs2) {
                std::cerr << "Failed to create MultiRobotSystem" << std::endl;
                csvFile.close();
                return 1;
            }
        
        // Create ProductAutomaton
        std::cout << "Creating product automaton...\n" << std::endl;
        //ProductAutomaton productAutomaton(*env, *mrs1, *buchi);
        // std::cout << "  - Nodes: " << productAutomaton.getNumStates() << std::endl;
        // std::cout << "  - Edges: " << productAutomaton.getNumEdges() << std::endl;
        // std::cout << "  - Accepting states: " << productAutomaton.getAcceptingStates().size() << std::endl;
            
        // // Visualize the Spot product automaton
        // std::ofstream dotFile("output/testing_product_automaton.dot");
        // spot::print_dot(dotFile, productAutomaton.getSpotAutomaton());
        // dotFile.close();
        // std::cout << "✓ Product automaton visualization saved to output/testing_product_automaton.dot\n" << std::endl;

        for (int numRobots = 1; numRobots <= 6; numRobots++) {
            std::cout << "Test " << numRobots << ": Creating " << numRobots << " robot(s)... ";
            std::cout.flush();
            
            Robot* robot = mrs1->getRobot(numRobots-1);
            mrs2->addRobot(robot);

          
            

            // Create ProductAutomaton
            ProductAutomaton productAutomaton(*env, *mrs2, *buchi);
            
            unsigned long numStates = productAutomaton.getNumStates();
            unsigned long numEdges = productAutomaton.getNumEdges();
            unsigned long acceptingStates = productAutomaton.getAcceptingStates().size();
            
            double stateRatio = (double)acceptingStates / numStates * 100.0;
            std::cout << numStates << " states, " << acceptingStates << " accepting ("
                    << std::fixed << std::setprecision(1) << stateRatio << "%)" << std::endl;
        
            // Write to CSV
            csvFile << numRobots << ","
                    << numStates << "," 
                    << numEdges << "," 
                    << acceptingStates << "," 
                    << stateRatio << "\n";
                
            if (true) {
                // Visualize the Spot product automaton
                std::ofstream dotFile("output/testing_product_automaton_2robots.dot");
                spot::print_dot(dotFile, productAutomaton.getSpotAutomaton());
                dotFile.close();
                std::cout << "  ✓ Product automaton visualization saved to output/testing_product_automaton_2robots.dot\n";
            }
            // Test OptimalAcceptingPath algorithm
            std::cout << "  Testing OptimalAcceptingPath... ";
            std::tuple<std::vector<uint16_t>, uint32_t> result = productAutomaton.OptimalAcceptingPath();
            std::vector<uint16_t> path = std::get<0>(result);
            
            if (!path.empty()) {
                std::cout << "✓ Found path of length " << path.size() << ": ";
                
                // Print first few states
                for (size_t i = 0; i < std::min(size_t(5), path.size()); i++) {
                    std::cout << path[i];
                    if (i < std::min(size_t(4), path.size() - 1)) std::cout << " → ";
                }
                if (path.size() > 5) std::cout << " → ...";
                
                // Verify path visits accepting states
                const auto& accepting = productAutomaton.getAcceptingStates();
                bool visitsAccepting = false;
                for (uint16_t state : path) {
                    if (std::find(accepting.begin(), accepting.end(), state) != accepting.end()) {
                        visitsAccepting = true;
                        break;
                    }
                }
                if (numRobots == 4) {
                    std::cout << "  Note: Testing with 4 robots." << std::endl;
                    // Create TaskAllocationAlgorithms
                    TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs2);
                    
                    //build the planning decision tree
                    allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs2);
                    allocAlg->visualizeTree("planning_decision_tree.png");
                    allocAlg->visualizeOptimalPath("optimal_path.png");
                    //add the full product automaton metrics to the algorithm metrics
                    allocAlg->getMetrics().setFullProductAutomatonMetrics(
                        productAutomaton.getNumStates(),
                        productAutomaton.getNumEdges(),
                        std::get<1>(result),
                        1000,
                        10
                    );
                    allocAlg->getMetrics().computeDerivedMetrics();
                    allocAlg->getMetrics().printSummary();
                    delete allocAlg;
                }
                if (visitsAccepting) {
                    std::cout << " ✓ (Total weight: " << std::get<1>(result) << ")" << std::endl;
                } else {
                    std::cout << " (WARNING: no accepting state in path)" << std::endl;
                }
            } else {
                std::cout << "✗ No accepting path found" << std::endl;
            }
        }
        
        csvFile.close();
        std::cout << "\n✓ Results saved to output/product_automaton_scaling.csv" << std::endl;
    
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
}