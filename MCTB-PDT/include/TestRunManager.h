#ifndef TEST_RUN_MANAGER_H
#define TEST_RUN_MANAGER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "AlgorithmMetrics.h"

/**
 * @class TestRunManager
 * @brief Manages test runs for a single category with per-configuration CSV export
 * 
 * Each instance manages one test category:
 * - Automaton State Scaling
 * - Robot Count Scaling
 * - Transition System Regions Scaling
 * - Average Capabilities Variation
 * - Robot Homogeneity Variation
 * 
 * Exports data to separate CSV files for each input configuration.
 */
class TestRunManager {
public:
    // ==================== TEST CATEGORIES ====================
    enum class TestCategory {
        AUTOMATON_STATES,           // 3-256 states
        AUTOMATON_STATES_BATCH,     // 16 automata with 4 batch configurations
        NUM_ROBOTS,                 // 3-45 robots
        TS_REGIONS,                 // 6-40 regions
        AVG_CAPABILITIES,           // 1-5 avg capabilities
        ROBOT_HOMOGENEITY           // 0.2-3 homogeneity
    };
    
    // ==================== TEST RUN DATA ====================
    struct TestRun {
        int run_id;
        int trial_number;           // Multiple runs for same config
        AlgorithmMetrics metrics;
        std::map<std::string, std::string> parameters;  // All variable configs
        std::string independent_variable;               // Primary grouping key for CSVs
        std::string timestamp;
        std::string notes;          // Optional: error notes, special observations
    };
    
    // ==================== PUBLIC INTERFACE ====================
public:
    // Constructor: each instance manages ONE category
    TestRunManager(TestCategory category, const std::string& data_dir = "./test_results");
    ~TestRunManager();
    
    // Initialize directory structure for this category
    void initialize();
    
    // Store a single run (category implicit from instance)
    void storeRun(const AlgorithmMetrics& metrics,
                  const std::map<std::string, std::string>& parameters,
                  const std::string& independent_variable,
                  int trial_number = 1,
                  const std::string& notes = "");
    
    // Retrieve all runs for this category
    std::vector<TestRun> getRuns() const;
    
    // Retrieve runs filtered by parameters
    std::vector<TestRun> getRuns(const std::map<std::string, std::string>& filters) const;
    
    // ==================== STATISTICS & AGGREGATION ====================
    
    struct RunStatistics {
        std::map<std::string, std::string> parameter_config;
        int num_runs;
        
        // Time statistics only
        double mean_time_ms;
        double stddev_time_ms;
        double min_time_ms;
        double max_time_ms;
    };
    
    // Get aggregated statistics (grouped by configuration)
    std::vector<RunStatistics> getStatistics() const;
    
    // ==================== EXPORT FOR ANALYSIS ====================
    
    // Export separate CSV for each input configuration
    void exportByConfiguration(const std::string& output_dir = "data", const std::string& independent_var = "automaton_id") const;
    
    // Export aggregated statistics
    void exportStatisticsToCSV(const std::string& output_filename) const;
    
    // Export summary report
    void exportSummaryReport(const std::string& output_filename) const;
    
    // ==================== ANALYSIS HELPERS ====================
    
    // Get metadata about this category
    int getExpectedNumberOfRuns() const;
    int getCurrentNumberOfRuns() const;
    double getCompletionPercentage() const;
    
    // Print summary of test progress
    void printTestProgress() const;
    
    // Get category name as string
    std::string getCategoryName() const;
    
    // ==================== INTERNAL DATA ====================
    
private:
    TestCategory category_;
    std::string data_dir_;
    std::vector<TestRun> runs_;
    
    // Helper methods
    std::string getCategoryDirPath() const;
    std::string getRunFilename(int run_id) const;
    
    void createCategoryDirectory();
    void saveCategoryData() const;
    
    // Serialization
    std::string serializeTestRun(const TestRun& run) const;
    TestRun deserializeTestRun(const std::string& str) const;
};

#endif // TEST_RUN_MANAGER_H
