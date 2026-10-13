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
    
    // Not persisted per run: the CSV exports carry every metric, and a file per run produced
    // hundreds of them per sweep. Call saveCategoryData() directly if a dump is ever wanted.
    
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
            << "tree_makespan_seconds,product_makespan_seconds,"
            << "random_sampling_iterations,random_sampling_computation_time_seconds,"
            << "random_sampling_makespan_seconds,random_sampling_memory_MB,"
            << "random_sampling_budget_multiplier,tree_computation_time_stddev_ms,"
            << "random_sampling_computation_time_stddev,random_sampling_makespan_stddev,"
            << "tree_product_ratio,memory_reduction_ratio,optimality_gap_percent,"
            << "runtime_speedup,percent_nodes_in_tree,state_space_reduction,robots_utilized,"
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
                << "," << efficiency.random_sampling_iterations
                << "," << metrics.random_sampling_computation_time_seconds
                << "," << quality.random_sampling_makespan_seconds
                << "," << efficiency.random_sampling_memory_MB
                << "," << efficiency.random_sampling_budget_multiplier
                << "," << efficiency.tree_computation_time_stddev_ms
                << "," << efficiency.random_sampling_computation_time_stddev
                << "," << quality.random_sampling_makespan_stddev
                << "," << efficiency.tree_product_ratio
                << "," << efficiency.memory_reduction_ratio
                << "," << efficiency.optimality_gap_percent
                << "," << efficiency.runtime_speedup
                << "," << efficiency.percent_nodes_in_tree
                << "," << efficiency.state_space_reduction
                << "," << quality.robots_utilized
                << "," << quality.robot_utilization_ratio
                << "," << quality.load_balance_variance
                << "\n";
        }
        
        out.close();
        std::cout << "✓ Exported " << config_runs.size() << " runs to: " << filename.str() << std::endl;
    }
}

// One row per (run, method) rather than one per run, so the three planners can be compared
// directly. The makespan ratio is taken against the best result any method achieved on that
// same instance, which is the product when it was built and the better of the other two
// otherwise, so instances of different sizes stay comparable.
void TestRunManager::exportCombinedCSV(const std::string& output_filename) const {
    if (runs_.empty()) {
        std::cerr << "No runs to export for category: " << getCategoryName() << std::endl;
        return;
    }

    fs::path out_path(output_filename);
    if (out_path.has_parent_path()) {
        fs::create_directories(out_path.parent_path());
    }

    std::ofstream out(output_filename);
    if (!out) {
        std::cerr << "Could not write combined CSV: " << output_filename << std::endl;
        return;
    }

    out << "sweep,run_id,trial_number,independent_variable,independent_value,"
        << "num_robots,num_automaton_states,num_automaton_edges,num_ts_regions,"
        << "avg_capabilities_per_robot,capability_homogeneity,"
        << "method,computation_time_seconds,makespan,makespan_ratio,memory_MB,solved,"
        << "sampler_budget_multiplier\n";

    for (const auto& run : runs_) {
        const auto& eff = run.metrics.getSubtreeEfficiency();
        const auto& q = run.metrics.getSolutionQuality();
        const auto& iv = run.metrics.getIndependentVariables();

        // The value of whichever parameter this sweep varies
        std::string independent_value;
        auto it = run.parameters.find(run.independent_variable);
        if (it != run.parameters.end()) independent_value = it->second;

        const std::string names[3] = {"tree", "product", "sampler"};
        const double makespans[3] = {
            q.tree_makespan_seconds,
            q.product_makespan_seconds,
            q.random_sampling_makespan_seconds
        };
        const double times[3] = {
            run.metrics.total_computation_time_ms / 1000.0,
            run.metrics.product_computation_time_ms / 1000.0,
            run.metrics.random_sampling_computation_time_seconds
        };
        const double memory[3] = {
            eff.task_allocation_algorithm_memory_MB,
            eff.full_product_automaton_memory_MB,
            eff.random_sampling_memory_MB
        };
        // A skipped product leaves its makespan at zero, which would otherwise read as a
        // perfect result rather than as an instance it could not handle
        const bool solved[3] = {
            q.tree_makespan_seconds > 0.0,
            eff.full_product_automaton_nodes > 0 && q.product_makespan_seconds > 0.0,
            q.random_sampling_makespan_seconds > 0.0
        };

        double best = 0.0;
        for (int i = 0; i < 3; ++i) {
            if (solved[i] && (best == 0.0 || makespans[i] < best)) best = makespans[i];
        }

        for (int i = 0; i < 3; ++i) {
            out << getCategoryName()
                << "," << run.run_id
                << "," << run.trial_number
                << "," << run.independent_variable
                << "," << independent_value
                << "," << iv.num_robots
                << "," << iv.num_automaton_states
                << "," << iv.num_automaton_edges
                << "," << iv.num_ts_regions
                << "," << iv.avg_capabilities_per_robot
                << "," << iv.capability_homogeneity
                << "," << names[i]
                << "," << times[i]
                << "," << makespans[i]
                << ",";
            // Left blank rather than zero when there is nothing meaningful to divide
            if (solved[i] && best > 0.0) out << (makespans[i] / best);
            out << "," << memory[i]
                << "," << (solved[i] ? 1 : 0)
                << "," << (names[i] == "sampler" ? eff.random_sampling_budget_multiplier : 0.0)
                << "\n";
        }
    }

    out.close();
    std::cout << "Exported combined comparison to: " << output_filename << std::endl;
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

namespace {

// One field per line as key=value, so a file stays readable and tolerates fields being
// added or reordered later: unknown keys are ignored and missing ones keep their defaults.
void writeField(std::ostringstream& oss, const std::string& key, const std::string& value) {
    std::string flat = value;
    // A newline inside a value would be read back as the start of the next field
    std::replace(flat.begin(), flat.end(), '\n', ' ');
    std::replace(flat.begin(), flat.end(), '\r', ' ');
    oss << key << "=" << flat << "\n";
}

template <typename T>
void writeField(std::ostringstream& oss, const std::string& key, T value) {
    oss << key << "=" << value << "\n";
}

template <typename T>
void writeVector(std::ostringstream& oss, const std::string& key, const std::vector<T>& values) {
    oss << key << "=";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ",";
        oss << values[i];
    }
    oss << "\n";
}

// Readers leave the target untouched when the key is absent, so an older file still loads
void readDouble(const std::map<std::string, std::string>& f, const std::string& key, double& out) {
    auto it = f.find(key);
    if (it != f.end() && !it->second.empty()) out = std::stod(it->second);
}

void readInt(const std::map<std::string, std::string>& f, const std::string& key, int& out) {
    auto it = f.find(key);
    if (it != f.end() && !it->second.empty()) out = std::stoi(it->second);
}

void readLongLong(const std::map<std::string, std::string>& f, const std::string& key, long long& out) {
    auto it = f.find(key);
    if (it != f.end() && !it->second.empty()) out = std::stoll(it->second);
}

void readUInt32(const std::map<std::string, std::string>& f, const std::string& key, uint32_t& out) {
    auto it = f.find(key);
    if (it != f.end() && !it->second.empty()) out = static_cast<uint32_t>(std::stoul(it->second));
}

void readString(const std::map<std::string, std::string>& f, const std::string& key, std::string& out) {
    auto it = f.find(key);
    if (it != f.end()) out = it->second;
}

template <typename T>
void readVector(const std::map<std::string, std::string>& f, const std::string& key, std::vector<T>& out) {
    auto it = f.find(key);
    if (it == f.end() || it->second.empty()) return;
    out.clear();
    std::istringstream iss(it->second);
    std::string item;
    while (std::getline(iss, item, ',')) {
        if (item.empty()) continue;
        out.push_back(static_cast<T>(std::stod(item)));
    }
}

}  // namespace

void TestRunManager::saveCategoryData() const {
    if (runs_.empty()) return;

    fs::create_directories(getCategoryDirPath());

    // Only the most recent run is new, so rewriting the whole category would be wasted work
    const TestRun& run = runs_.back();
    std::ofstream out(getRunFilename(run.run_id));
    if (!out) {
        std::cerr << "Could not write run file: " << getRunFilename(run.run_id) << std::endl;
        return;
    }
    out << serializeTestRun(run);
    out.close();
}

std::string TestRunManager::serializeTestRun(const TestRun& run) const {
    std::ostringstream oss;
    oss << std::setprecision(10);

    writeField(oss, "run_id", run.run_id);
    writeField(oss, "trial_number", run.trial_number);
    writeField(oss, "timestamp", run.timestamp);
    writeField(oss, "independent_variable", run.independent_variable);
    writeField(oss, "notes", run.notes);

    for (const auto& entry : run.parameters) {
        writeField(oss, "param." + entry.first, entry.second);
    }

    const AlgorithmMetrics& m = run.metrics;
    writeField(oss, "time.total_computation_time_ms", m.total_computation_time_ms);
    writeField(oss, "time.product_computation_time_ms", m.product_computation_time_ms);
    writeField(oss, "time.random_sampling_computation_time_seconds", m.random_sampling_computation_time_seconds);

    const auto& iv = m.getIndependentVariables();
    writeField(oss, "iv.num_automaton_states", iv.num_automaton_states);
    writeField(oss, "iv.num_automaton_edges", iv.num_automaton_edges);
    writeField(oss, "iv.total_required_capabilities", iv.total_required_capabilities);
    writeField(oss, "iv.independent_required_capabilities", iv.independent_required_capabilities);
    writeField(oss, "iv.num_atomic_propositions", iv.num_atomic_propositions);
    writeField(oss, "iv.num_robots", iv.num_robots);
    writeField(oss, "iv.total_robot_capabilities", iv.total_robot_capabilities);
    writeField(oss, "iv.independent_robot_capabilities", iv.independent_robot_capabilities);
    writeField(oss, "iv.num_ts_regions", iv.num_ts_regions);
    writeField(oss, "iv.avg_capabilities_per_robot", iv.avg_capabilities_per_robot);
    writeField(oss, "iv.capability_homogeneity", iv.capability_homogeneity);
    writeField(oss, "iv.num_inter_task_constraints", iv.num_inter_task_constraints);

    const auto& eff = m.getSubtreeEfficiency();
    writeField(oss, "eff.total_nodes_planning", eff.total_nodes_planning);
    writeField(oss, "eff.total_nodes_traversed", eff.total_nodes_traversed);
    writeField(oss, "eff.total_nodes_pruned", eff.total_nodes_pruned);
    writeField(oss, "eff.nodes_satisfying_ltl", eff.nodes_satisfying_ltl);
    writeField(oss, "eff.task_allocation_algorithm_memory_MB", eff.task_allocation_algorithm_memory_MB);
    writeField(oss, "eff.full_product_automaton_nodes", eff.full_product_automaton_nodes);
    writeField(oss, "eff.full_product_automaton_edges", eff.full_product_automaton_edges);
    writeField(oss, "eff.full_product_automaton_memory_MB", eff.full_product_automaton_memory_MB);
    writeField(oss, "eff.random_sampling_iterations", eff.random_sampling_iterations);
    writeField(oss, "eff.random_sampling_memory_MB", eff.random_sampling_memory_MB);
    writeField(oss, "eff.random_sampling_budget_multiplier", eff.random_sampling_budget_multiplier);
    writeField(oss, "eff.tree_computation_time_stddev_ms", eff.tree_computation_time_stddev_ms);
    writeField(oss, "eff.random_sampling_computation_time_stddev", eff.random_sampling_computation_time_stddev);
    writeField(oss, "eff.pruning_ratio", eff.pruning_ratio);
    writeField(oss, "eff.explored_product_ratio", eff.explored_product_ratio);
    writeField(oss, "eff.tree_product_ratio", eff.tree_product_ratio);
    writeField(oss, "eff.memory_reduction_ratio", eff.memory_reduction_ratio);
    writeField(oss, "eff.optimality_gap_percent", eff.optimality_gap_percent);
    writeField(oss, "eff.runtime_speedup", eff.runtime_speedup);
    writeField(oss, "eff.percent_nodes_in_tree", eff.percent_nodes_in_tree);
    writeField(oss, "eff.state_space_reduction", eff.state_space_reduction);

    const auto& q = m.getSolutionQuality();
    writeField(oss, "qual.tree_makespan_seconds", q.tree_makespan_seconds);
    writeField(oss, "qual.product_makespan_seconds", q.product_makespan_seconds);
    writeField(oss, "qual.random_sampling_makespan_seconds", q.random_sampling_makespan_seconds);
    writeField(oss, "qual.random_sampling_makespan_stddev", q.random_sampling_makespan_stddev);
    writeField(oss, "qual.sum_of_travel_times_seconds", q.sum_of_travel_times_seconds);
    writeField(oss, "qual.total_travel_distance", q.total_travel_distance);
    writeField(oss, "qual.max_individual_travel_distance", q.max_individual_travel_distance);
    writeField(oss, "qual.load_balance_variance", q.load_balance_variance);
    writeField(oss, "qual.robots_utilized", q.robots_utilized);
    writeField(oss, "qual.robot_utilization_ratio", q.robot_utilization_ratio);
    writeVector(oss, "qual.individual_travel_times", q.individual_travel_times);
    writeVector(oss, "qual.individual_travel_distances", q.individual_travel_distances);
    writeVector(oss, "qual.tasks_per_robot", q.tasks_per_robot);

    return oss.str();
}

TestRunManager::TestRun TestRunManager::deserializeTestRun(const std::string& str) const {
    TestRun run{};

    // Split into key=value pairs on the first '=' of each line, so values may contain '='
    std::map<std::string, std::string> fields;
    std::istringstream iss(str);
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        fields[line.substr(0, eq)] = line.substr(eq + 1);
    }

    readInt(fields, "run_id", run.run_id);
    readInt(fields, "trial_number", run.trial_number);
    readString(fields, "timestamp", run.timestamp);
    readString(fields, "independent_variable", run.independent_variable);
    readString(fields, "notes", run.notes);

    const std::string prefix = "param.";
    for (const auto& entry : fields) {
        if (entry.first.compare(0, prefix.size(), prefix) == 0) {
            run.parameters[entry.first.substr(prefix.size())] = entry.second;
        }
    }

    AlgorithmMetrics& m = run.metrics;
    readDouble(fields, "time.total_computation_time_ms", m.total_computation_time_ms);
    readDouble(fields, "time.product_computation_time_ms", m.product_computation_time_ms);
    readDouble(fields, "time.random_sampling_computation_time_seconds", m.random_sampling_computation_time_seconds);

    readInt(fields, "iv.num_automaton_states", m.iv_.num_automaton_states);
    readInt(fields, "iv.num_automaton_edges", m.iv_.num_automaton_edges);
    readInt(fields, "iv.total_required_capabilities", m.iv_.total_required_capabilities);
    readInt(fields, "iv.independent_required_capabilities", m.iv_.independent_required_capabilities);
    readInt(fields, "iv.num_atomic_propositions", m.iv_.num_atomic_propositions);
    readInt(fields, "iv.num_robots", m.iv_.num_robots);
    readInt(fields, "iv.total_robot_capabilities", m.iv_.total_robot_capabilities);
    readInt(fields, "iv.independent_robot_capabilities", m.iv_.independent_robot_capabilities);
    readInt(fields, "iv.num_ts_regions", m.iv_.num_ts_regions);
    readDouble(fields, "iv.avg_capabilities_per_robot", m.iv_.avg_capabilities_per_robot);
    readDouble(fields, "iv.capability_homogeneity", m.iv_.capability_homogeneity);
    readInt(fields, "iv.num_inter_task_constraints", m.iv_.num_inter_task_constraints);

    readLongLong(fields, "eff.total_nodes_planning", m.subtree_efficiency_.total_nodes_planning);
    readLongLong(fields, "eff.total_nodes_traversed", m.subtree_efficiency_.total_nodes_traversed);
    readLongLong(fields, "eff.total_nodes_pruned", m.subtree_efficiency_.total_nodes_pruned);
    readLongLong(fields, "eff.nodes_satisfying_ltl", m.subtree_efficiency_.nodes_satisfying_ltl);
    readDouble(fields, "eff.task_allocation_algorithm_memory_MB", m.subtree_efficiency_.task_allocation_algorithm_memory_MB);
    readLongLong(fields, "eff.full_product_automaton_nodes", m.subtree_efficiency_.full_product_automaton_nodes);
    readLongLong(fields, "eff.full_product_automaton_edges", m.subtree_efficiency_.full_product_automaton_edges);
    readDouble(fields, "eff.full_product_automaton_memory_MB", m.subtree_efficiency_.full_product_automaton_memory_MB);
    readUInt32(fields, "eff.random_sampling_iterations", m.subtree_efficiency_.random_sampling_iterations);
    readDouble(fields, "eff.random_sampling_memory_MB", m.subtree_efficiency_.random_sampling_memory_MB);
    readDouble(fields, "eff.random_sampling_budget_multiplier", m.subtree_efficiency_.random_sampling_budget_multiplier);
    readDouble(fields, "eff.tree_computation_time_stddev_ms", m.subtree_efficiency_.tree_computation_time_stddev_ms);
    readDouble(fields, "eff.random_sampling_computation_time_stddev", m.subtree_efficiency_.random_sampling_computation_time_stddev);
    readDouble(fields, "eff.pruning_ratio", m.subtree_efficiency_.pruning_ratio);
    readDouble(fields, "eff.explored_product_ratio", m.subtree_efficiency_.explored_product_ratio);
    readDouble(fields, "eff.tree_product_ratio", m.subtree_efficiency_.tree_product_ratio);
    readDouble(fields, "eff.memory_reduction_ratio", m.subtree_efficiency_.memory_reduction_ratio);
    readDouble(fields, "eff.optimality_gap_percent", m.subtree_efficiency_.optimality_gap_percent);
    readDouble(fields, "eff.runtime_speedup", m.subtree_efficiency_.runtime_speedup);
    readDouble(fields, "eff.percent_nodes_in_tree", m.subtree_efficiency_.percent_nodes_in_tree);
    readLongLong(fields, "eff.state_space_reduction", m.subtree_efficiency_.state_space_reduction);

    readDouble(fields, "qual.tree_makespan_seconds", m.solution_quality_.tree_makespan_seconds);
    readDouble(fields, "qual.product_makespan_seconds", m.solution_quality_.product_makespan_seconds);
    readDouble(fields, "qual.random_sampling_makespan_seconds", m.solution_quality_.random_sampling_makespan_seconds);
    readDouble(fields, "qual.random_sampling_makespan_stddev", m.solution_quality_.random_sampling_makespan_stddev);
    readDouble(fields, "qual.sum_of_travel_times_seconds", m.solution_quality_.sum_of_travel_times_seconds);
    readDouble(fields, "qual.total_travel_distance", m.solution_quality_.total_travel_distance);
    readDouble(fields, "qual.max_individual_travel_distance", m.solution_quality_.max_individual_travel_distance);
    readDouble(fields, "qual.load_balance_variance", m.solution_quality_.load_balance_variance);
    readInt(fields, "qual.robots_utilized", m.solution_quality_.robots_utilized);
    readDouble(fields, "qual.robot_utilization_ratio", m.solution_quality_.robot_utilization_ratio);
    readVector(fields, "qual.individual_travel_times", m.solution_quality_.individual_travel_times);
    readVector(fields, "qual.individual_travel_distances", m.solution_quality_.individual_travel_distances);
    readVector(fields, "qual.tasks_per_robot", m.solution_quality_.tasks_per_robot);

    return run;
}
