#ifndef ORGANIZEDTESTS_TREE_TRIALS_H
#define ORGANIZEDTESTS_TREE_TRIALS_H

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "../include/AlgorithmMetrics.h"
#include "../include/TaskAllocationAlgorithms.h"

/**
 * @file tree_trials.h
 * @brief Repeat the tree search and report the average, for every sweep.
 *
 * These sweeps now measure one planner. The product automaton is archived rather than rebuilt
 * (comparison/product_archive.csv) and the sampler moved to the budget study, which reports it
 * against a stated search budget and repeats it. That leaves the tree planner, and with the other
 * two gone there is room to repeat it as well.
 *
 * The repeats are there for the timing, not the plan. The search is deterministic, so every pass
 * returns the same makespan; the runtime is not, and a single sample of it is what every sampler
 * budget in the budget study is derived from. Averaging several passes and recording the spread
 * puts the tree on the same footing as the sampler, which has been averaged over trials all along.
 */
namespace treetrials {

/** How many passes per configuration. Override with the TREE_TRIALS environment variable. */
inline int count() {
    static const int value = [] {
        const char* text = std::getenv("TREE_TRIALS");
        if (!text) return 6;                       // matches the budget study's six sampler trials
        const int parsed = std::atoi(text);
        return parsed < 1 ? 1 : parsed;
    }();
    return value;
}

/**
 * @brief Run the tree search @p count() times, keeping the last pass's tree.
 *
 * A fresh TaskAllocationAlgorithms per pass, because the search is not written to be run twice on
 * one instance - it would accumulate the previous pass's planning tree. The caller owns the
 * returned object.
 *
 * @param times Filled with each pass's wall-clock milliseconds.
 */
inline TaskAllocationAlgorithms* runRepeated(BuchiAutomaton* buchi, Environment* env,
                                             MultiRobotSystem* mrs,
                                             std::vector<double>& times) {
    TaskAllocationAlgorithms* allocAlg = nullptr;
    times.clear();
    for (int trial = 0; trial < count(); ++trial) {
        delete allocAlg;
        allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
        const auto start = std::chrono::high_resolution_clock::now();
        allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
        times.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count());
    }
    return allocAlg;
}

/**
 * @brief Replace the last pass's runtime with the mean over all passes, and record the spread.
 *
 * Call after runRepeated. The search writes its own single-pass timing into the metrics, so
 * without this the CSV would carry whichever pass happened to run last.
 */
inline void recordTiming(AlgorithmMetrics& metrics, const std::vector<double>& times) {
    if (times.empty()) return;

    double sum = 0.0;
    for (double value : times) sum += value;
    const double mean = sum / times.size();

    double variance = 0.0;
    for (double value : times) variance += (value - mean) * (value - mean);
    // Population spread over the trials actually run, not an estimate of a wider population
    const double stddev = times.size() > 1 ? std::sqrt(variance / times.size()) : 0.0;

    metrics.total_computation_time_ms = mean;
    metrics.subtree_efficiency_.tree_computation_time_stddev_ms = stddev;
}

}  // namespace treetrials

#endif  // ORGANIZEDTESTS_TREE_TRIALS_H
