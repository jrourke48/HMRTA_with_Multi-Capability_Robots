#ifndef GRID_VIS_H
#define GRID_VIS_H

#include "Environment.h"
#include "GridWorld.h"
#include "../MultiRobotSystem/MultiRobotSystem.h"
#include <vector>
#include <map>
#include <string>
#include "../Tree/Tree_Node.h"

/**
 * Stores concrete paths for each robot through the task tree
 * Maps: robotId -> vector of points representing D* path
 */
using RobotPathMap = std::map<uint32_t, std::vector<Point>>;

/**
 * Stores the synchronization barriers along each robot's concrete path
 * Maps: robotId -> vector of (optimal-path step index, index into that robot's path
 * where the step's segment ends)
 *
 * The planner already holds every robot allocated to a step to one common arrival time,
 * so a robot that reaches a task location waits there until every other robot allocated
 * to the same step has arrived before any of them move on.
 */
using RobotBarrierMap = std::map<uint32_t, std::vector<std::pair<size_t, size_t>>>;

/**
 * Visualizes the environment grid with TS state regions, obstacles, robots, and optimal path
 * Uses raylib for graphics rendering
 * 
 * Features:
 * - Display grid cells with obstacles
 * - Overlay TS state regions with different colors
 * - Render multiple robots with distinct colors
 * - Highlight concrete D* paths for each robot through the task tree
 */

/**
 * compute_dstar_paths - Compute concrete paths for each robot through the task tree using D*
 * 
 * @param env Reference to the Environment containing grid and TS
 * @param mrs Reference to the MultiRobotSystem containing all robots
 * @param optimalPath The computed optimal task allocation path (sequence of Tree_Node*)
 * @return RobotPathMap mapping robotId -> concrete path (sequence of grid points)
 * 
 * For each node in the path and each robot allocated to that node:
 * - Runs D* Lite from robot's current position to the TS state goal
 * - Concatenates all paths to build complete trajectories
 *
 * If barriers is non-null it is filled with the per-step synchronization points for each
 * robot, which visualize_environment uses to keep an allocated group moving together.
 */
RobotPathMap compute_dstar_paths(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath,
    RobotBarrierMap* barriers = nullptr
);

/**
 * visualize_environment - Displays grid, TS regions, robots, and D* paths
 * 
 * @param env Reference to the Environment containing grid and TS
 * @param mrs Reference to the MultiRobotSystem containing all robots
 * @param optimalPath The computed optimal path (sequence of Tree_Node*)
 * @param robotPaths Pre-computed D* paths for each robot (optional)
 * @param windowTitle Title for the window
 * 
 * Displays a visualization of:
 * - Grid cells colored by TS state
 * - Obstacles in dark gray
 * - Each robot's concrete D* path in a distinct color
 * - Each robot as a colored circle at its current position
 * 
 * If robotPaths is empty, computes them automatically.
 * Window closes when user clicks the close button.
 */
void visualize_environment(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath,
    const RobotPathMap& robotPaths = RobotPathMap(),
    const char* windowTitle = "Multi-Robot Task Planner",
    const std::string& snapshotDirectory = "",
    const RobotBarrierMap& robotBarriers = RobotBarrierMap()
);

#endif // GRID_VIS_H
