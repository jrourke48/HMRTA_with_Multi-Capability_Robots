#include "TestRunManager.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <set>

namespace fs = std::filesystem;

// ==================== CONSTRUCTOR & INITIALIZATION ====================

TestRunManager::TestRunManager(TestCategory category, const std::string& data_dir)
    : category_(category), data_dir_(data_dir) {
    // Single category per instance
}

TestRunManager::~TestRunManager() {
    // Cleanup if needed
}

void TestRunManager::initialize() {
    // Create main data directory
    if (!fs::exists(data_dir_)) {
        fs::create_directories(data_dir_);
        std::cout << "✓ Created results directory: " << data_dir_ << std::endl;
    }
    
    // Create category subdirectory
    createCategoryDirectory();
    
    // Create metadata file
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::ofstream meta_file(getCategoryDirPath() + "/metadata.txt");
    meta_file << "Test Manager Initialized\n";
    meta_file << "Category: " << getCategoryName() << "\n";
    meta_file << "Created: " << oss.str() << "\n";
    meta_file << "Data Version: 1.0\n";
    meta_file.close();
    
    std::cout << "✓ Test manager initialized for category: " << getCategoryName() << std::endl;
}

// ==================== STORE & RETRIEVE ====================

void TestRunManager::storeRun(const AlgorithmMetrics& metrics,
                              const std::map<std::string, std::string>& parameters,
                              const std::string& independent_variable,
                              int trial_number,
                              const std::string& notes) {
    // Create TestRun object
    TestRun run;
    run.run_id = runs_.size() + 1;
    run.trial_number = trial_number;
    run.metrics = metrics;
    run.parameters = parameters;
    run.independent_variable = independent_variable;
    run.notes = notes;
    
    // Get current timestamp
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    run.timestamp = oss.str();
    
    // Store in memory
    runs_.push_back(run);
    
    // Persist to file
    saveCategoryData();
    
    std::cout << "✓ Stored run " << run.run_id << " in category \"" 
              << getCategoryName() << "\" (trial " << trial_number << ")"
              << std::endl;
}

std::vector<TestRunManager::TestRun> TestRunManager::getRuns() const {
    return runs_;
}

std::vector<TestRunManager::TestRun> TestRunManager::getRuns(
    const std::map<std::string, std::string>& filters) const {
    
    std::vector<TestRun> filtered;
    
    for (const auto& run : runs_) {
        bool matches = true;
        for (const auto& [key, value] : filters) {
            auto it = run.parameters.find(key);
            if (it == run.parameters.end() || it->second != value) {
                matches = false;
                break;
            }
        }
        if (matches) {
            filtered.push_back(run);
        }
    }
    
    return filtered;
}

// ==================== STATISTICS & AGGREGATION ====================

std::vector<TestRunManager::RunStatistics> TestRunManager::getStatistics() const {
    std::map<std::map<std::string, std::string>, std::vector<TestRun>> grouped;
    
    // Group runs by parameter configuration
    for (const auto& run : runs_) {
        grouped[run.parameters].push_back(run);
    }
    
    std::vector<RunStatistics> stats;
    
    for (const auto& [params, group_runs] : grouped) {
        RunStatistics stat;
        stat.parameter_config = params;
        stat.num_runs = group_runs.size();
        
        // Calculate time statistics only (other metrics don't change run-to-run)
        std::vector<double> times;
        
        for (const auto& run : group_runs) {
            times.push_back(run.metrics.total_computation_time_ms);
        }
        
        // Calculate means
        stat.mean_time_ms = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        
        // Calculate standard deviation
        double time_sum_sq = 0.0;
        for (size_t i = 0; i < times.size(); ++i) {
            time_sum_sq += std::pow(times[i] - stat.mean_time_ms, 2);
        }
        
        stat.stddev_time_ms = std::sqrt(time_sum_sq / times.size());
        
        // Min/Max times
        stat.min_time_ms = *std::min_element(times.begin(), times.end());
        stat.max_time_ms = *std::max_element(times.begin(), times.end());
        
        stats.push_back(stat);
    }
    
    return stats;
}

// ==================== EXPORT FOR ANALYSIS ====================

void TestRunManager::exportByConfiguration(const std::string& output_dir, const std::string& independent_var) const {
    if (runs_.empty()) {
        std::cerr << "⚠ No runs to export for category: " << getCategoryName() << std::endl;
        return;
    }
    
    // Group runs by all parameters EXCEPT the independent variable
    // (independent_variable is the primary variable being tested, so all its values go in same CSV)
    std::map<std::map<std::string, std::string>, std::vector<TestRun>> grouped;
    
    for (const auto& run : runs_) {
        // Create parameter set excluding independent variable
        std::map<std::string, std::string> grouping_params;
        for (const auto& [key, value] : run.parameters) {
            if (key != independent_var) {  // Exclude the independent variable
                grouping_params[key] = value;
            }
        }
        grouped[grouping_params].push_back(run);
    }
    
    std::string export_dir = output_dir;
    fs::create_directories(export_dir);
    
    // Create a CSV for each secondary parameter combination
    int config_id = 0;
    for (const auto& [group_params, config_runs] : grouped) {
        // Build filename from grouping parameters
        std::ostringstream filename;
        filename << export_dir << "/" << getCategoryName();
        
        for (const auto& [key, value] : group_params) {
            filename << "_" << key << "_" << value;
        }
        filename << ".csv";
        
        std::ofstream out(filename.str());
        
        // Write header
        out << "run_id,trial_number,timestamp";
        
        // Get all unique parameter keys
        std::set<std::string> all_keys;
        for (const auto& run : config_runs) {
            for (const auto& [key, value] : run.parameters) {
                all_keys.insert(key);
            }
        }
        
        for (const auto& key : all_keys) {
            out << "," << key;
        }
        
        out << ",num_automaton_states,num_automaton_edges,total_required_capabilities,"
            << "independent_required_capabilities,num_robots,total_robot_capabilities,"
            << "independent_robot_capabilities,avg_capabilities_per_robot,capability_homogeneity,"
            << "num_ts_regions,total_nodes_planning,total_nodes_traversed,total_nodes_pruned,"
            << "nodes_satisfying_ltl,pruning_ratio,full_product_automaton_nodes,"
            << "full_product_automaton_edges,explored_product_ratio,task_allocation_algorithm_memory_MB,"
            << "full_product_automaton_memory_MB,total_computation_time_ms,product_computation_time_ms,"
            << "tree_makespan_seconds,product_makespan_seconds,robots_utilized,"
            << "robot_utilization_ratio,load_balance_variance\n";
        
        // Write data for this configuration
        for (const auto& run : config_runs) {
            out << run.run_id << "," << run.trial_number << ",\"" << run.timestamp << "\"";
            
            for (const auto& key : all_keys) {
                out << ",";
                auto it = run.parameters.find(key);
                if (it != run.parameters.end()) {
                    out << it->second;
                }
            }
            
            const auto& metrics = run.metrics;
            const auto& efficiency = metrics.getSubtreeEfficiency();
            const auto& quality = metrics.getSolutionQuality();
            const auto& iv = metrics.getIndependentVariables();
            
            out << "," << iv.num_automaton_states
                << "," << iv.num_automaton_edges
                << "," << iv.total_required_capabilities
                << "," << iv.independent_required_capabilities
                << "," << iv.num_robots
                << "," << iv.total_robot_capabilities
                << "," << iv.independent_robot_capabilities
                << "," << iv.avg_capabilities_per_robot
                << "," << iv.capability_homogeneity
                << "," << iv.num_ts_regions
                << "," << efficiency.total_nodes_planning
                << "," << efficiency.total_nodes_traversed
                << "," << efficiency.total_nodes_pruned
                << "," << efficiency.nodes_satisfying_ltl
                << "," << efficiency.pruning_ratio
                << "," << efficiency.full_product_automaton_nodes
                << "," << efficiency.full_product_automaton_edges
                << "," << efficiency.explored_product_ratio
                << "," << efficiency.task_allocation_algorithm_memory_MB
                << "," << efficiency.full_product_automaton_memory_MB
                << "," << metrics.total_computation_time_ms
                << "," << metrics.product_computation_time_ms
                << "," << quality.tree_makespan_seconds
                << "," << quality.product_makespan_seconds
                << "," << quality.robots_utilized
                << "," << quality.robot_utilization_ratio
                << "," << quality.load_balance_variance
                << "\n";
        }
        
        out.close();
        std::cout << "✓ Exported " << config_runs.size() << " runs to: " << filename.str() << std::endl;
    }
}

void TestRunManager::exportStatisticsToCSV(const std::string& output_filename) const {
    auto stats = getStatistics();
    if (stats.empty()) {
        std::cerr << "⚠ No statistics to export for category: " << getCategoryName() << std::endl;
        return;
    }
    
    std::ofstream out(output_filename);
    
    // Header
    out << "config_id";
    
    // Get all unique parameter keys
    std::set<std::string> all_keys;
    for (const auto& stat : stats) {
        for (const auto& [key, value] : stat.parameter_config) {
            all_keys.insert(key);
        }
    }
    
    for (const auto& key : all_keys) {
        out << "," << key;
    }
    
    out << ",num_runs,mean_time_ms,stddev_time_ms,min_time_ms,max_time_ms\n";
    
    // Write data
    int config_id = 0;
    for (const auto& stat : stats) {
        out << config_id++;
        
        for (const auto& key : all_keys) {
            out << ",";
            auto it = stat.parameter_config.find(key);
            if (it != stat.parameter_config.end()) {
                out << it->second;
            }
        }
        
        out << "," << stat.num_runs
            << "," << stat.mean_time_ms
            << "," << stat.stddev_time_ms
            << "," << stat.min_time_ms
            << "," << stat.max_time_ms
            << "\n";
    }
    
    out.close();
    std::cout << "✓ Exported statistics to: " << output_filename << std::endl;
}

void TestRunManager::exportSummaryReport(const std::string& output_filename) const {
    std::ofstream out(output_filename);
    
    out << "# Test Run Summary Report\n\n";
    out << "## " << getCategoryName() << "\n";
    out << "**Status**: " << getCurrentNumberOfRuns() << " / " 
        << getExpectedNumberOfRuns() << " runs ("
        << std::fixed << std::setprecision(1) 
        << getCompletionPercentage() << "%)\n\n";
    
    if (!runs_.empty()) {
        out << "| Metric | Value |\n";
        out << "|--------|-------|\n";
        
        double total_time = 0.0;
        for (const auto& run : runs_) {
            total_time += run.metrics.total_computation_time_ms;
        }
        
        out << "| Average Computation Time | " 
            << (total_time / runs_.size()) << " ms |\n";
        out << "| Last Run Timestamp | " << runs_.back().timestamp << " |\n";
        out << "| Total Runs | " << runs_.size() << " |\n";
    }
    
    out << "\n";
    out.close();
    std::cout << "✓ Exported summary report to: " << output_filename << std::endl;
}

// ==================== ANALYSIS HELPERS ====================

int TestRunManager::getExpectedNumberOfRuns() const {
    switch (category_) {
        case TestCategory::AUTOMATON_STATES:   return 64;  // 16 automata × 4 robot configs
        case TestCategory::AUTOMATON_STATES_BATCH: return 64;  // 16 automata × 4 robot configs
        case TestCategory::NUM_ROBOTS:         return 64;  // 8 automata × 8 robot counts
        case TestCategory::TS_REGIONS:         return 64;  // 8 automata × 8 region counts
        case TestCategory::AVG_CAPABILITIES:   return 80;  // 1-5 avg cap
        case TestCategory::ROBOT_HOMOGENEITY:  return 80;  // 0.2-3 homogeneity
        default: return 0;
    }
}

int TestRunManager::getCurrentNumberOfRuns() const {
    return runs_.size();
}

double TestRunManager::getCompletionPercentage() const {
    int expected = getExpectedNumberOfRuns();
    if (expected == 0) return 0.0;
    return (getCurrentNumberOfRuns() / static_cast<double>(expected)) * 100.0;
}

void TestRunManager::printTestProgress() const {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TEST PROGRESS - " << getCategoryName() << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    int current = getCurrentNumberOfRuns();
    int expected = getExpectedNumberOfRuns();
    double completion = getCompletionPercentage();
    
    std::cout << std::left << std::setw(30) << getCategoryName()
              << " [" << std::setfill('=') << std::setw((int)(completion / 5)) 
              << "" << std::setfill(' ') << std::setw(20 - (int)(completion / 5))
              << "] " << current << "/" << expected 
              << " (" << std::fixed << std::setprecision(1) << completion << "%)"
              << std::endl;
    
    std::cout << std::string(70, '=') << std::endl << std::endl;
}

std::string TestRunManager::getCategoryName() const {
    switch (category_) {
        case TestCategory::AUTOMATON_STATES:        return "automaton_states";
        case TestCategory::AUTOMATON_STATES_BATCH:  return "automaton_states_batch";
        case TestCategory::NUM_ROBOTS:              return "num_robots";
        case TestCategory::TS_REGIONS:              return "ts_regions";
        case TestCategory::AVG_CAPABILITIES:        return "avg_capabilities";
        case TestCategory::ROBOT_HOMOGENEITY:       return "robot_homogeneity";
        default: return "unknown";
    }
}

// ==================== INTERNAL HELPERS ====================

std::string TestRunManager::getCategoryDirPath() const {
    return data_dir_ + "/" + getCategoryName();
}

std::string TestRunManager::getRunFilename(int run_id) const {
    std::ostringstream oss;
    oss << getCategoryDirPath() << "/run_" << std::setfill('0') 
        << std::setw(4) << run_id << ".txt";
    return oss.str();
}

void TestRunManager::createCategoryDirectory() {
    std::string dir = getCategoryDirPath();
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
        std::cout << "✓ Created category directory: " << getCategoryName() << std::endl;
    }
}

void TestRunManager::saveCategoryData() const {
    // TODO: Save runs to individual text files
}

std::string TestRunManager::serializeTestRun(const TestRun& run) const {
    (void)run;  // Unused parameter
    // TODO: Serialize TestRun to text format
    return "";
}

TestRunManager::TestRun TestRunManager::deserializeTestRun(const std::string& str) const {
    (void)str;  // Unused parameter
    // TODO: Deserialize text to TestRun
    return TestRun{};
}
