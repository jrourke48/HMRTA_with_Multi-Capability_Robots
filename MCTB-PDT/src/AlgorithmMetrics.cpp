#include "AlgorithmMetrics.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>
#include <limits>

// ==================== INITIALIZATION ====================

void AlgorithmMetrics::clearMetrics() {
    // Explicitly clear independent variables
    iv_.num_automaton_states = 0;
    iv_.num_automaton_edges = 0;
    iv_.total_required_capabilities = 0;
    iv_.independent_required_capabilities = 0;
    iv_.num_atomic_propositions = 0;
    iv_.num_robots = 0;
    iv_.total_robot_capabilities = 0;
    iv_.independent_robot_capabilities = 0;
    iv_.num_ts_regions = 0;
    iv_.avg_capabilities_per_robot = 0.0;
    iv_.capability_homogeneity = 0.0;
    iv_.num_inter_task_constraints = 0;
    
    // Clear runtime 
    total_computation_time_ms = 0.0;
    
    // Clear subtree efficiency metrics
    subtree_efficiency_.total_nodes_planning = 0;
    subtree_efficiency_.total_nodes_traversed = 0;
    subtree_efficiency_.total_nodes_pruned = 0;
    subtree_efficiency_.nodes_satisfying_ltl = 0;
    subtree_efficiency_.task_allocation_algorithm_memory_MB = 0;
    subtree_efficiency_.full_product_automaton_nodes = 0;
    subtree_efficiency_.full_product_automaton_edges = 0;
    subtree_efficiency_.full_product_automaton_memory_MB = 0;
    subtree_efficiency_.pruning_ratio = 0.0;
    subtree_efficiency_.explored_product_ratio = 0.0;
    subtree_efficiency_.tree_product_ratio = 0.0;
    subtree_efficiency_.memory_reduction_ratio = 0.0;
    subtree_efficiency_.optimality_gap_percent = 0.0;
    subtree_efficiency_.runtime_speedup = 0.0;
    subtree_efficiency_.percent_nodes_in_tree = 0.0;
    subtree_efficiency_.state_space_reduction = 0;
    
    // Clear solution quality metrics
    solution_quality_.tree_makespan_seconds = 0.0;
    solution_quality_.product_makespan_seconds = 0.0;
    solution_quality_.random_sampling_makespan_seconds = 0.0;
    subtree_efficiency_.random_sampling_iterations = 0;
    subtree_efficiency_.random_sampling_memory_MB = 0.0;
    subtree_efficiency_.random_sampling_budget_multiplier = 0.0;
    subtree_efficiency_.tree_computation_time_stddev_ms = 0.0;
    subtree_efficiency_.random_sampling_computation_time_stddev = 0.0;
    solution_quality_.random_sampling_makespan_stddev = 0.0;
    random_sampling_computation_time_seconds = 0.0;
    solution_quality_.sum_of_travel_times_seconds = 0.0;
    solution_quality_.total_travel_distance = 0.0;
    solution_quality_.max_individual_travel_distance = 0.0;
    solution_quality_.robots_utilized = 0;
    solution_quality_.robot_utilization_ratio = 0.0;
    solution_quality_.load_balance_variance = 0.0;
    solution_quality_.individual_travel_times.clear();
    solution_quality_.individual_travel_distances.clear();
    solution_quality_.tasks_per_robot.clear();
}

void AlgorithmMetrics::setIndependentVariables(const IndependentVariables& vars) {
    iv_ = vars;
    
    // Compute derived independent variables
    if (vars.num_robots > 0) {
        iv_.avg_capabilities_per_robot = 
            static_cast<double>(vars.total_robot_capabilities) / vars.num_robots;
        iv_.capability_homogeneity = 
            static_cast<double>(vars.independent_robot_capabilities) / vars.num_robots;
    }
}

// ==================== TIMER MANAGEMENT ====================

void AlgorithmMetrics::startTimer() {
    overall_start_time_ = std::chrono::high_resolution_clock::now();
}

void AlgorithmMetrics::stopTimer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - overall_start_time_);
    total_computation_time_ms = duration.count() / 1000.0;  // Convert microseconds to milliseconds
}

// ==================== SOLUTION QUALITY METRICS UPDATES ====================

void AlgorithmMetrics::setSolutionMakespan(double tree_makespan_sec, double product_makespan_sec) {
    solution_quality_.tree_makespan_seconds = tree_makespan_sec;
    solution_quality_.product_makespan_seconds = product_makespan_sec;
}

void AlgorithmMetrics::setSumOfTravelTimes(double sum_seconds) {
    solution_quality_.sum_of_travel_times_seconds = sum_seconds;
}

void AlgorithmMetrics::setTravelDistance(double total_distance, double max_individual_distance) {
    solution_quality_.total_travel_distance = total_distance;
    solution_quality_.max_individual_travel_distance = max_individual_distance;
}

void AlgorithmMetrics::setRobotsUtilized(int count) {
    solution_quality_.robots_utilized = count;
    // Immediately recompute the ratio
    solution_quality_.robot_utilization_ratio = computeRobotUtilizationRatio();
}

void AlgorithmMetrics::setIndividualTravelTime(int robot_id, double time_seconds) {
    if (robot_id >= 0 && robot_id < static_cast<int>(solution_quality_.individual_travel_times.size())) {
        solution_quality_.individual_travel_times[robot_id] = time_seconds;
    } else if (robot_id == static_cast<int>(solution_quality_.individual_travel_times.size())) {
        solution_quality_.individual_travel_times.push_back(time_seconds);
    }
}

void AlgorithmMetrics::setIndividualTravelDistance(int robot_id, double distance) {
    if (robot_id >= 0 && robot_id < static_cast<int>(solution_quality_.individual_travel_distances.size())) {
        solution_quality_.individual_travel_distances[robot_id] = distance;
    } else if (robot_id == static_cast<int>(solution_quality_.individual_travel_distances.size())) {
        solution_quality_.individual_travel_distances.push_back(distance);
    }
}

void AlgorithmMetrics::setTasksPerRobot(int robot_id, int task_count) {
    if (robot_id >= 0 && robot_id < static_cast<int>(solution_quality_.tasks_per_robot.size())) {
        solution_quality_.tasks_per_robot[robot_id] = task_count;
    } else if (robot_id == static_cast<int>(solution_quality_.tasks_per_robot.size())) {
        solution_quality_.tasks_per_robot.push_back(task_count);
    }
}

// ==================== DERIVED METRICS COMPUTATION ====================

double AlgorithmMetrics::computePruningRatio() const {
    if (subtree_efficiency_.total_nodes_planning == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_pruned) / 
           subtree_efficiency_.total_nodes_traversed;
}

double AlgorithmMetrics::computePercentNodesInTree() const {
    if (subtree_efficiency_.total_nodes_traversed == 0) return 0.0;
    return (static_cast<double>(subtree_efficiency_.total_nodes_planning) / 
           subtree_efficiency_.total_nodes_traversed)*100.0;
}

double AlgorithmMetrics::computeExploredProductRatio() const {
    if (subtree_efficiency_.full_product_automaton_nodes == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_traversed) / 
           subtree_efficiency_.full_product_automaton_nodes;
}

double AlgorithmMetrics::computeTreeProductRatio() const {
    if (subtree_efficiency_.full_product_automaton_nodes == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_planning) / 
           subtree_efficiency_.full_product_automaton_nodes;
}

double AlgorithmMetrics::computeMemoryReductionRatio() const {
    if (subtree_efficiency_.full_product_automaton_memory_MB == 0) return 0.0;
    // Percent reduction: (full - subtree) / full
    return (subtree_efficiency_.full_product_automaton_memory_MB -
            subtree_efficiency_.task_allocation_algorithm_memory_MB) /
           subtree_efficiency_.full_product_automaton_memory_MB;
}
double AlgorithmMetrics::computeOptimalityGap() const {
    if (solution_quality_.tree_makespan_seconds == 0) return 0.0;

    // Measure against the product automaton when one was built. On instances too large for it the
    // random sampler is the only other baseline, so fall back to that rather than report nothing
    const double baseline = (subtree_efficiency_.full_product_automaton_nodes > 0 && solution_quality_.product_makespan_seconds < std::numeric_limits<uint32_t>::max() / 2)
        ? solution_quality_.product_makespan_seconds
        : solution_quality_.random_sampling_makespan_seconds;

    // Neither baseline ran, so there is nothing to compare against
    if (baseline == 0) return 0.0;
    // Return NaN if no path was found (sentinel: baseline makespan is max uint32)
    if (baseline == std::numeric_limits<uint32_t>::max()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return 100.0 * (solution_quality_.tree_makespan_seconds - baseline) /
           baseline;
}
double AlgorithmMetrics::computeRuntimeSpeedup() const {
    if (total_computation_time_ms == 0) return 0.0;

    // Measure against the product automaton when it ran, otherwise against the random sampler.
    // The sampler records seconds while the tree and the product record milliseconds
    const double baseline_ms = (product_computation_time_ms > 0)
        ? product_computation_time_ms
        : random_sampling_computation_time_seconds * 1000.0;

    return baseline_ms / total_computation_time_ms;
}

double AlgorithmMetrics::computeRobotUtilizationRatio() const {
    if (iv_.num_robots == 0) return 0.0;
    return static_cast<double>(solution_quality_.robots_utilized) / iv_.num_robots;
}

double AlgorithmMetrics::computeLoadBalanceVariance() const {
    if (solution_quality_.tasks_per_robot.empty()) return 0.0;
    
    double mean = std::accumulate(solution_quality_.tasks_per_robot.begin(),
                                   solution_quality_.tasks_per_robot.end(), 0.0) / 
                  solution_quality_.tasks_per_robot.size();
    
    double sq_sum = 0.0;
    for (int tasks : solution_quality_.tasks_per_robot) {
        sq_sum += (tasks - mean) * (tasks - mean);
    }
    return sq_sum / solution_quality_.tasks_per_robot.size();
}

void AlgorithmMetrics::computeDerivedMetrics() {
    // Subtree efficiency derived metrics
    subtree_efficiency_.pruning_ratio = computePruningRatio();
    subtree_efficiency_.explored_product_ratio = computeExploredProductRatio();
    subtree_efficiency_.tree_product_ratio = computeTreeProductRatio();
    subtree_efficiency_.memory_reduction_ratio = computeMemoryReductionRatio();
    subtree_efficiency_.percent_nodes_in_tree = computePercentNodesInTree();

    subtree_efficiency_.state_space_reduction = 
        subtree_efficiency_.full_product_automaton_nodes - 
        subtree_efficiency_.total_nodes_planning;
    
    // Solution quality derived metrics
    solution_quality_.robot_utilization_ratio = computeRobotUtilizationRatio();
    solution_quality_.load_balance_variance = computeLoadBalanceVariance();
    subtree_efficiency_.optimality_gap_percent = computeOptimalityGap();
    subtree_efficiency_.runtime_speedup = computeRuntimeSpeedup();
}

// ==================== REPORTING ====================

void AlgorithmMetrics::printSummary() const {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "ALGORITHM METRICS SUMMARY" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    // Independent Variables
    std::cout << "INDEPENDENT VARIABLES:" << std::endl;
    std::cout << "  Automaton States: " << iv_.num_automaton_states << std::endl;
    std::cout << "  Automaton Edges: " << iv_.num_automaton_edges << std::endl;
    std::cout << "  Total Required Capabilities: " << iv_.total_required_capabilities << std::endl;
    std::cout << "  Independent Required Capabilities: " << iv_.independent_required_capabilities << std::endl;
    std::cout << "  Atomic Propositions: " << iv_.num_atomic_propositions << std::endl;
    std::cout << "  Number of Robots: " << iv_.num_robots << std::endl;
    std::cout << "  Total Robot Capabilities: " << iv_.total_robot_capabilities << std::endl;
    std::cout << "  Independent Robot Capabilities: " << iv_.independent_robot_capabilities << std::endl;
    std::cout << "  Transition System Regions: " << iv_.num_ts_regions << std::endl;
    std::cout << "  Avg Capabilities/Robot: " << std::fixed << std::setprecision(2) 
              << iv_.avg_capabilities_per_robot << std::endl;
    std::cout << "  Capability Homogeneity: " << iv_.capability_homogeneity << std::endl;
    std::cout << "  Inter-task Constraints: " << iv_.num_inter_task_constraints << "\n" << std::endl;
    
    // Runtime Metrics
    std::cout << "RUNTIME METRICS:" << std::endl;
    std::cout << "  Total Computation Time: " << std::fixed << std::setprecision(2) 
              << total_computation_time_ms << " ms" << "\n" << std::endl;
    
    // Product Automaton Metrics
    std::cout << "PRODUCT AUTOMATON METRICS:" << std::endl;
    std::cout << "  Full Product Nodes: " << subtree_efficiency_.full_product_automaton_nodes << std::endl;
    std::cout << "  Full Product Edges: " << subtree_efficiency_.full_product_automaton_edges << std::endl;
    std::cout << "  Full Product Computation Time: " << std::fixed << std::setprecision(2) 
              << product_computation_time_ms << " ms" << std::endl;
    std::cout << "  Full Product Makespan: " << std::fixed << std::setprecision(2) 
              << solution_quality_.product_makespan_seconds << " s" << std::endl;
    std::cout << "  Full Product Memory Usage: " << std::fixed << std::setprecision(2) 
              << subtree_efficiency_.full_product_automaton_memory_MB << " MB" << "\n" << std::endl;

    // Random Sampling Metrics
    std::cout << "RANDOM SAMPLING METRICS:" << std::endl;
    std::cout << "  Random Sampling Iterations: " << subtree_efficiency_.random_sampling_iterations << std::endl;
    std::cout << "  Random Sampling Computation Time: " << std::fixed << std::setprecision(4)
              << random_sampling_computation_time_seconds << " s" << std::endl;
    std::cout << "  Random Sampling Makespan: " << std::fixed << std::setprecision(2)
              << solution_quality_.random_sampling_makespan_seconds << " s" << std::endl;
    std::cout << "  Random Sampling Memory Usage: " << std::fixed << std::setprecision(2)
              << subtree_efficiency_.random_sampling_memory_MB << " MB" << std::endl;
    std::cout << "  Random Sampling Budget Multiple: " << std::fixed << std::setprecision(0)
              << subtree_efficiency_.random_sampling_budget_multiplier << "x tree time" << std::endl;
    std::cout << "  Spread over trials (stddev): tree time "
              << std::fixed << std::setprecision(4) << subtree_efficiency_.tree_computation_time_stddev_ms << " ms, "
              << "sampler time " << subtree_efficiency_.random_sampling_computation_time_stddev << " s, "
              << "sampler makespan " << std::setprecision(2)
              << solution_quality_.random_sampling_makespan_stddev << " s" << "\n" << std::endl;
    
    // Subtree Efficiency
    std::cout << "SUBTREE EFFICIENCY METRICS:" << std::endl;
    std::cout << "  Total Nodes Planning: " << subtree_efficiency_.total_nodes_planning << std::endl;
    std::cout << "  Total Nodes Traversed: " << subtree_efficiency_.total_nodes_traversed << std::endl;
    std::cout << "  Total Nodes Pruned: " << subtree_efficiency_.total_nodes_pruned << std::endl;
    std::cout << "  Pruning Ratio: " << std::fixed << std::setprecision(4) 
              << subtree_efficiency_.pruning_ratio << std::endl;
    std::cout << "  Percent Nodes in Tree: " << std::fixed << std::setprecision(2) 
              << subtree_efficiency_.percent_nodes_in_tree << "%" << std::endl;
    std::cout << "  Explored Product Ratio: " << std::fixed << std::setprecision(4) 
              << subtree_efficiency_.explored_product_ratio << std::endl;
    std::cout << "  Tree Product Ratio: " << std::fixed << std::setprecision(4) 
              << subtree_efficiency_.tree_product_ratio << std::endl;
    std::cout << "  Memory Reduction Ratio: " << std::fixed << std::setprecision(4) 
              << subtree_efficiency_.memory_reduction_ratio << std::endl;
    std::cout << "  Optimality Gap: " << std::fixed << std::setprecision(2) 
              << subtree_efficiency_.optimality_gap_percent << "%" << std::endl;
    std::cout << "  Runtime Speedup: " << std::fixed << std::setprecision(2) 
              << subtree_efficiency_.runtime_speedup << std::endl;
    std::cout << "  State-Space Reduction: " << subtree_efficiency_.state_space_reduction << "\n" << std::endl;
    
    // Solution Quality
    std::cout << "SOLUTION QUALITY METRICS:" << std::endl;
    std::cout << "  Tree Makespan: " << std::fixed << std::setprecision(2) 
              << solution_quality_.tree_makespan_seconds << " sec" << std::endl;
    std::cout << "  Sum of Travel Times: " << solution_quality_.sum_of_travel_times_seconds << " sec" << std::endl;
    std::cout << "  Total Travel Distance: " << solution_quality_.total_travel_distance << std::endl;
    std::cout << "  Max Individual Distance: " << solution_quality_.max_individual_travel_distance << std::endl;
    std::cout << "  Robots Utilized: " << solution_quality_.robots_utilized << " / " 
              << iv_.num_robots << std::endl;
    std::cout << "  Robot Utilization Ratio: " << std::fixed << std::setprecision(4) 
              << solution_quality_.robot_utilization_ratio << std::endl;
    std::cout << "  Load Balance Variance: " << solution_quality_.load_balance_variance << std::endl;
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
}

void AlgorithmMetrics::exportToCSV(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << std::endl;
        return;
    }
    
    // Header
    outfile << "Metric,Value,Unit\n";
    
    // Independent Variables
    outfile << "Automaton States," << iv_.num_automaton_states << ",count\n";
    outfile << "Automaton Edges," << iv_.num_automaton_edges << ",count\n";
    outfile << "Atomic Propositions," << iv_.num_atomic_propositions << ",count\n";
    outfile << "Number of Robots," << iv_.num_robots << ",count\n";
    outfile << "Total Robot Capabilities," << iv_.total_robot_capabilities << ",count\n";
    outfile << "Transition System Regions," << iv_.num_ts_regions << ",count\n";
    outfile << "Avg Capabilities/Robot," << std::fixed << std::setprecision(4) 
            << iv_.avg_capabilities_per_robot << ",ratio\n";
    outfile << "Capability Homogeneity," << iv_.capability_homogeneity << ",ratio\n";
    
    // Runtime Metrics
    outfile << "Total Computation Time," << std::fixed << std::setprecision(2) 
            << total_computation_time_ms << ",ms\n";
    
    // Product Automaton Metrics
    outfile << "Full Product Nodes," << subtree_efficiency_.full_product_automaton_nodes << ",count\n";
    outfile << "Full Product Edges," << subtree_efficiency_.full_product_automaton_edges << ",count\n";
    outfile << "Full Product Computation Time," << std::fixed << std::setprecision(2) 
            << product_computation_time_ms << ",ms\n";
    outfile << "Full Product Makespan," << std::fixed << std::setprecision(2) 
            << solution_quality_.product_makespan_seconds << ",seconds\n";
    outfile << "Full Product Memory Usage," << std::fixed << std::setprecision(2) 
            << subtree_efficiency_.full_product_automaton_memory_MB << ",MB\n";

    // Random Sampling Metrics
    outfile << "Random Sampling Iterations," << subtree_efficiency_.random_sampling_iterations << ",count\n";
    outfile << "Random Sampling Computation Time," << std::fixed << std::setprecision(4)
            << random_sampling_computation_time_seconds << ",seconds\n";
    outfile << "Random Sampling Makespan," << std::fixed << std::setprecision(2)
            << solution_quality_.random_sampling_makespan_seconds << ",seconds\n";
    outfile << "Random Sampling Memory Usage," << std::fixed << std::setprecision(2)
            << subtree_efficiency_.random_sampling_memory_MB << ",MB\n";
    outfile << "Random Sampling Budget Multiple," << std::fixed << std::setprecision(0)
            << subtree_efficiency_.random_sampling_budget_multiplier << ",x tree time\n";
    outfile << "Tree Computation Time StdDev," << std::fixed << std::setprecision(4)
            << subtree_efficiency_.tree_computation_time_stddev_ms << ",ms\n";
    outfile << "Random Sampling Computation Time StdDev," << std::setprecision(4)
            << subtree_efficiency_.random_sampling_computation_time_stddev << ",seconds\n";
    outfile << "Random Sampling Makespan StdDev," << std::setprecision(2)
            << solution_quality_.random_sampling_makespan_stddev << ",seconds\n";
    
    // Subtree Efficiency
    outfile << "Total Nodes Pruned," << subtree_efficiency_.total_nodes_pruned << ",count\n";
    outfile << "Pruning Ratio," << std::fixed << std::setprecision(4) 
            << subtree_efficiency_.pruning_ratio << ",ratio\n";
    outfile << "Memory Reduction Ratio," << subtree_efficiency_.memory_reduction_ratio << ",ratio\n";
    outfile << "State-Space Reduction," << subtree_efficiency_.state_space_reduction << ",count\n";
    
    // Solution Quality
    outfile << "Tree Makespan," << std::fixed << std::setprecision(2) 
            << solution_quality_.tree_makespan_seconds << ",seconds\n";
    outfile << "Sum of Travel Times," << solution_quality_.sum_of_travel_times_seconds << ",seconds\n";
    outfile << "Total Travel Distance," << solution_quality_.total_travel_distance << ",units\n";
    outfile << "Max Individual Distance," << solution_quality_.max_individual_travel_distance << ",units\n";
    outfile << "Robots Utilized," << solution_quality_.robots_utilized << ",count\n";
    outfile << "Robot Utilization Ratio," << std::fixed << std::setprecision(4) 
            << solution_quality_.robot_utilization_ratio << ",ratio\n";
    
    outfile.close();
    std::cout << "Metrics exported to: " << filename << std::endl;
}

void AlgorithmMetrics::exportToJSON(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << std::endl;
        return;
    }
    
    outfile << "{\n";
    outfile << "  \"independent_variables\": {\n";
    outfile << "    \"automaton_states\": " << iv_.num_automaton_states << ",\n";
    outfile << "    \"automaton_edges\": " << iv_.num_automaton_edges << ",\n";
    outfile << "    \"atomic_propositions\": " << iv_.num_atomic_propositions << ",\n";
    outfile << "    \"num_robots\": " << iv_.num_robots << ",\n";
    outfile << "    \"total_robot_capabilities\": " << iv_.total_robot_capabilities << ",\n";
    outfile << "    \"ts_regions\": " << iv_.num_ts_regions << ",\n";
    outfile << "    \"avg_capabilities_per_robot\": " << std::fixed << std::setprecision(4) 
            << iv_.avg_capabilities_per_robot << ",\n";
    outfile << "    \"inter_task_constraints\": " << iv_.num_inter_task_constraints << "\n";
    outfile << "  },\n";
    
    outfile << "  \"runtime_metrics\": {\n";
    outfile << "    \"total_computation_time_ms\": " << std::fixed << std::setprecision(2) 
            << total_computation_time_ms << "\n";
    outfile << "  },\n";
    
    outfile << "  \"product_automaton_metrics\": {\n";
    outfile << "    \"full_product_nodes\": " << subtree_efficiency_.full_product_automaton_nodes << ",\n";
    outfile << "    \"full_product_edges\": " << subtree_efficiency_.full_product_automaton_edges << ",\n";
    outfile << "    \"full_product_computation_time_ms\": " << std::fixed << std::setprecision(2) 
            << product_computation_time_ms << ",\n";
    outfile << "    \"full_product_makespan_seconds\": " << std::fixed << std::setprecision(2) 
            << solution_quality_.product_makespan_seconds << ",\n";
    outfile << "    \"full_product_memory_MB\": " << std::fixed << std::setprecision(2) 
            << subtree_efficiency_.full_product_automaton_memory_MB << "\n";
    outfile << "  },\n";

    outfile << "  \"random_sampling_metrics\": {\n";
    outfile << "    \"random_sampling_iterations\": " << subtree_efficiency_.random_sampling_iterations << ",\n";
    outfile << "    \"random_sampling_computation_time_seconds\": " << std::fixed << std::setprecision(4)
            << random_sampling_computation_time_seconds << ",\n";
    outfile << "    \"random_sampling_makespan_seconds\": " << std::fixed << std::setprecision(2)
            << solution_quality_.random_sampling_makespan_seconds << ",\n";
    outfile << "    \"random_sampling_memory_MB\": " << std::fixed << std::setprecision(2)
            << subtree_efficiency_.random_sampling_memory_MB << ",\n";
    outfile << "    \"random_sampling_budget_multiplier\": " << std::fixed << std::setprecision(0)
            << subtree_efficiency_.random_sampling_budget_multiplier << ",\n";
    outfile << "    \"tree_computation_time_stddev_ms\": " << std::setprecision(4)
            << subtree_efficiency_.tree_computation_time_stddev_ms << ",\n";
    outfile << "    \"random_sampling_computation_time_stddev\": " << std::setprecision(4)
            << subtree_efficiency_.random_sampling_computation_time_stddev << ",\n";
    outfile << "    \"random_sampling_makespan_stddev\": " << std::setprecision(2)
            << solution_quality_.random_sampling_makespan_stddev << "\n";
    outfile << "  },\n";

    outfile << "  \"subtree_efficiency_metrics\": {\n";
    outfile << "    \"total_nodes_traversed\": " << subtree_efficiency_.total_nodes_traversed << ",\n";
    outfile << "    \"total_nodes_pruned\": " << subtree_efficiency_.total_nodes_pruned << ",\n";
    outfile << "    \"pruning_ratio\": " << std::fixed << std::setprecision(4) 
            << subtree_efficiency_.pruning_ratio << ",\n";
    outfile << "    \"memory_reduction_ratio\": " 
            << subtree_efficiency_.memory_reduction_ratio << ",\n";
    outfile << "    \"state_space_reduction\": " 
            << subtree_efficiency_.state_space_reduction << "\n";
    outfile << "  },\n";
    
    outfile << "  \"solution_quality_metrics\": {\n";
    outfile << "    \"tree_makespan_seconds\": " << std::fixed << std::setprecision(2) 
            << solution_quality_.tree_makespan_seconds << ",\n";
    outfile << "    \"sum_of_travel_times\": " 
            << solution_quality_.sum_of_travel_times_seconds << ",\n";
    outfile << "    \"total_travel_distance\": " 
            << solution_quality_.total_travel_distance << ",\n";
    outfile << "    \"max_individual_travel_distance\": " 
            << solution_quality_.max_individual_travel_distance << ",\n";
    outfile << "    \"robots_utilized\": " << solution_quality_.robots_utilized << ",\n";
    outfile << "    \"robot_utilization_ratio\": " << std::fixed << std::setprecision(4) 
            << solution_quality_.robot_utilization_ratio << "\n";
    outfile << "  }\n";
    outfile << "}\n";
    
    outfile.close();
    std::cout << "Metrics exported to: " << filename << std::endl;
}
void AlgorithmMetrics::setTaskMemoryUsageMB(double MB) {
    subtree_efficiency_.task_allocation_algorithm_memory_MB = MB;
}
void AlgorithmMetrics::setFullProductAutomatonMetrics(long long nodes, long long edges, double makespan_seconds, double computation_time_ms, double MB) {
    subtree_efficiency_.full_product_automaton_nodes = nodes;
    subtree_efficiency_.full_product_automaton_edges = edges;
    subtree_efficiency_.full_product_automaton_memory_MB = MB;
    product_computation_time_ms = computation_time_ms;
    solution_quality_.product_makespan_seconds = makespan_seconds;
}


void AlgorithmMetrics::setRandomSamplingMetrics(uint32_t iterations, double makespan_seconds,
                                               double computation_time_seconds, double MB) {
    subtree_efficiency_.random_sampling_iterations = iterations;
    subtree_efficiency_.random_sampling_memory_MB = MB;
    random_sampling_computation_time_seconds = computation_time_seconds;
    solution_quality_.random_sampling_makespan_seconds = makespan_seconds;
}

/**
 * @brief Average a set of trials field by field.
 * @param trials Metrics from repeated runs of one configuration.
 * @return The averaged metrics, with derived fields recomputed rather than averaged.
 *
 * Every primitive field is averaged, including ones that never vary. That is deliberate: the mean
 * of identical values is that value, so deterministic fields pass through untouched and there is
 * no hand-written list of "fields that vary" to fall out of date when a metric is added.
 */
AlgorithmMetrics AlgorithmMetrics::mean(const std::vector<AlgorithmMetrics>& trials) {
    if (trials.empty()) return AlgorithmMetrics();
    if (trials.size() == 1) return trials.front();

    AlgorithmMetrics out = trials.front();
    const double n = static_cast<double>(trials.size());

    auto avgD = [&](double AlgorithmMetrics::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.*field;
        out.*field = sum / n;
    };

    // Timings live directly on the object
    avgD(&AlgorithmMetrics::total_computation_time_ms);
    avgD(&AlgorithmMetrics::product_computation_time_ms);
    avgD(&AlgorithmMetrics::random_sampling_computation_time_seconds);

    auto avgIv = [&](double IndependentVariables::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.iv_.*field;
        out.iv_.*field = sum / n;
    };
    auto avgIvInt = [&](int IndependentVariables::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.iv_.*field;
        out.iv_.*field = static_cast<int>(std::llround(sum / n));
    };
    avgIvInt(&IndependentVariables::num_automaton_states);
    avgIvInt(&IndependentVariables::num_automaton_edges);
    avgIvInt(&IndependentVariables::total_required_capabilities);
    avgIvInt(&IndependentVariables::independent_required_capabilities);
    avgIvInt(&IndependentVariables::num_atomic_propositions);
    avgIvInt(&IndependentVariables::num_robots);
    avgIvInt(&IndependentVariables::total_robot_capabilities);
    avgIvInt(&IndependentVariables::independent_robot_capabilities);
    avgIvInt(&IndependentVariables::num_ts_regions);
    avgIvInt(&IndependentVariables::num_inter_task_constraints);
    avgIv(&IndependentVariables::avg_capabilities_per_robot);
    avgIv(&IndependentVariables::capability_homogeneity);

    auto avgEffLL = [&](long long SubtreeEfficiencyMetrics::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += static_cast<double>(t.subtree_efficiency_.*field);
        out.subtree_efficiency_.*field = std::llround(sum / n);
    };
    auto avgEffD = [&](double SubtreeEfficiencyMetrics::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.subtree_efficiency_.*field;
        out.subtree_efficiency_.*field = sum / n;
    };
    avgEffLL(&SubtreeEfficiencyMetrics::total_nodes_planning);
    avgEffLL(&SubtreeEfficiencyMetrics::total_nodes_traversed);
    avgEffLL(&SubtreeEfficiencyMetrics::total_nodes_pruned);
    avgEffLL(&SubtreeEfficiencyMetrics::nodes_satisfying_ltl);
    avgEffLL(&SubtreeEfficiencyMetrics::full_product_automaton_nodes);
    avgEffLL(&SubtreeEfficiencyMetrics::full_product_automaton_edges);
    avgEffD(&SubtreeEfficiencyMetrics::task_allocation_algorithm_memory_MB);
    avgEffD(&SubtreeEfficiencyMetrics::full_product_automaton_memory_MB);
    avgEffD(&SubtreeEfficiencyMetrics::random_sampling_memory_MB);
    {
        double sum = 0.0;
        for (const auto& t : trials) sum += static_cast<double>(t.subtree_efficiency_.random_sampling_iterations);
        out.subtree_efficiency_.random_sampling_iterations = static_cast<uint32_t>(std::llround(sum / n));
    }

    auto avgQualD = [&](double SolutionQualityMetrics::*field) {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.solution_quality_.*field;
        out.solution_quality_.*field = sum / n;
    };
    avgQualD(&SolutionQualityMetrics::tree_makespan_seconds);
    avgQualD(&SolutionQualityMetrics::product_makespan_seconds);
    avgQualD(&SolutionQualityMetrics::random_sampling_makespan_seconds);
    avgQualD(&SolutionQualityMetrics::sum_of_travel_times_seconds);
    avgQualD(&SolutionQualityMetrics::total_travel_distance);
    avgQualD(&SolutionQualityMetrics::max_individual_travel_distance);
    avgQualD(&SolutionQualityMetrics::load_balance_variance);
    avgQualD(&SolutionQualityMetrics::robot_utilization_ratio);
    {
        double sum = 0.0;
        for (const auto& t : trials) sum += t.solution_quality_.robots_utilized;
        out.solution_quality_.robots_utilized = static_cast<int>(std::llround(sum / n));
    }

    // Ratios and gaps are functions of the fields above, so they are recomputed from the averaged
    // values rather than averaged themselves
    out.computeDerivedMetrics();
    return out;
}

/**
 * @brief Population standard deviation of one sampled quantity across trials.
 * @param values One value per trial.
 * @return 0 when fewer than two values were supplied.
 */
double AlgorithmMetrics::stddev(const std::vector<double>& values) {
    if (values.size() < 2) return 0.0;
    double sum = 0.0;
    for (double v : values) sum += v;
    const double mean_value = sum / static_cast<double>(values.size());
    double variance = 0.0;
    for (double v : values) variance += (v - mean_value) * (v - mean_value);
    return std::sqrt(variance / static_cast<double>(values.size()));
}
