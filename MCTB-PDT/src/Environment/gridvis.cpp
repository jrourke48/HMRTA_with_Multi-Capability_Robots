#include "Environment/gridvis.h"
#include "Environment/DStarGridWorld.h"
#include <raylib.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

// Helper: Convert grid point to screen pixel coordinates
static void gridToScreen(
    Point gridPos,
    int cellSize,
    int margin,
    int& screenX,
    int& screenY
) {
    screenX = margin + gridPos.getX() * cellSize;
    screenY = margin + gridPos.getY() * cellSize;
}

// Helper: Compute a single D* path from start to goal
static std::vector<Point> compute_dstar_path_single(
    const Environment& env,
    const Point& startPos,
    const Point& goalPos
) {
    std::vector<Point> path;
    
    // Get the GridWorld from environment
    const GridWorld* gridWorld = env.getGridWorld();
    if (!gridWorld) {
        std::cerr << "compute_dstar_path_single: GridWorld is null" << std::endl;
        return path;
    }
    
    // Create D* Lite planner for this grid
    DStarGridWorldPlanner planner(const_cast<GridWorld*>(gridWorld));
    
    // Compute path from start to goal
    path = planner.plan(startPos, goalPos);
    
    return path;
}

/**
 * compute_dstar_paths - Compute D* paths for each robot through the task tree
 * 
 * For each Tree_Node in the path:
 *   - Extract goal position from TS state
 *   - For each allocated robot:
 *     - Run D* from robot's current position to goal
 *     - Append to robot's path
 */
RobotPathMap compute_dstar_paths(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath,
    RobotBarrierMap* barriers
) {
    RobotPathMap robotPaths;

    // Initialize empty paths for all robots
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        robotPaths[i] = std::vector<Point>();
    }

    if (barriers) {
        barriers->clear();
    }

    // Record where a robot has to wait for the rest of its group. This runs for every robot
    // allocated to the step, including ones whose D* segment could not be computed: skipping
    // them would leave the rest of the group waiting on a robot that never reports arrival.
    auto recordBarrier = [&](uint32_t robotId, size_t stepIndex) {
        if (!barriers) {
            return;
        }
        const std::vector<Point>& path = robotPaths[robotId];
        (*barriers)[robotId].emplace_back(stepIndex, path.empty() ? 0 : path.size() - 1);
    };
    
    // Track the last position of each robot for continuous path segments.
    // Seed from the configured start positions, not the current ones: the planner moves the
    // shared robot objects as it explores and leaves them wherever the search ended, so
    // getCurrentPosition() here would start the animation from the wrong place.
    std::map<uint32_t, Point> robotLastPositions;
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        const std::vector<Robot*>& robots = mrs.getRobots();
        if (i < robots.size()) {
            robotLastPositions[i] = robots[i]->getStartPosition();
        }
    }
    
    // For each node in the optimal path
    for (size_t stepIndex = 0; stepIndex < optimalPath.size(); ++stepIndex) {
        const Tree_Node* node = optimalPath[stepIndex];
        if (!node) {
            continue;
        }
        
        // Get which robots are allocated to this node
        const std::vector<bool>& allocation = node->getRoboTaskAllocation();
        const std::vector<Point>& nodePositions = node->getRobotPositions();
        
        // For each allocated robot
        for (uint32_t robotId = 0; robotId < allocation.size(); ++robotId) {
            if (!allocation[robotId]) {
                continue;  // Robot not allocated to this node
            }
            
            // Start from last position in our tracking
            Point currentPos;
            if (robotLastPositions.count(robotId)) {
                currentPos = robotLastPositions[robotId];
            } else {
                const std::vector<Robot*>& robots = mrs.getRobots();
                if (robotId < robots.size()) {
                    currentPos = robots[robotId]->getStartPosition();
                } else {
                    recordBarrier(robotId, stepIndex);
                    continue;
                }
            }

            // Get goal position from node (updated with circular arrangement)
            if (robotId >= nodePositions.size()) {
                recordBarrier(robotId, stepIndex);
                continue;
            }
            Point goalPos = nodePositions[robotId];

            // Compute D* path from current position to goal
            std::vector<Point> segmentPath = compute_dstar_path_single(env, currentPos, goalPos);

            // Skip if path is empty
            if (segmentPath.empty()) {
                recordBarrier(robotId, stepIndex);
                continue;
            }
            
            // Append path, avoiding duplicate endpoints
            for (size_t i = 0; i < segmentPath.size(); ++i) {
                if (i == 0 && !robotPaths[robotId].empty()) {
                    continue;  // Skip first point if it's a duplicate of last point
                }
                robotPaths[robotId].push_back(segmentPath[i]);
            }

            // The end of this segment is where the robot waits for the rest of its group
            recordBarrier(robotId, stepIndex);

            // Update robot's last position to where it ends up from task allocation
            if (robotId < nodePositions.size()) {
                robotLastPositions[robotId] = nodePositions[robotId];
            } else {
                // Fallback to end of D* path
                robotLastPositions[robotId] = segmentPath.back();
            }
        }
    }
    
    return robotPaths;
}

/**
 * visualize_environment - Renders grid, TS regions, obstacles, robots, and D* paths
 */
void visualize_environment(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath,
    const RobotPathMap& robotPaths,
    const char* windowTitle,
    const std::string& snapshotDirectory,
    const RobotBarrierMap& robotBarriers
) {
    const GridWorld* gridWorld = env.getGridWorld();
    if (!gridWorld) {
        std::cerr << "visualize_environment: GridWorld is null" << std::endl;
        return;
    }
    
    if (mrs.getNumRobots() == 0) {
        return;
    }

    const uint32_t gridW = gridWorld->getWidth();
    const uint32_t gridH = gridWorld->getHeight();

    const int margin = 40;
    const int maxScreenW = 1400;
    const int maxScreenH = 900;
    
    // Calculate cell size to fit the entire grid within max screen dimensions
    int cellSize = std::min(
        (maxScreenW - 2 * margin) / (int)gridW,
        (maxScreenH - 2 * margin) / (int)gridH
    );
    cellSize = std::max(cellSize, 10);  // minimum cell size of 10 pixels
    
    const int screenW = margin * 2 + gridW * cellSize;
    const int screenH = margin * 2 + gridH * cellSize;

    InitWindow(screenW, screenH, windowTitle);
    SetTargetFPS(60);

    // Compute paths if not provided
    RobotPathMap paths = robotPaths;
    RobotBarrierMap barriers = robotBarriers;
    if (paths.empty() && !optimalPath.empty()) {
        barriers.clear();
        paths = compute_dstar_paths(env, mrs, optimalPath, &barriers);
    }

    // Animation state: track current position index for each robot on its path
    std::map<uint32_t, size_t> robotPathIndices;
    std::map<uint32_t, float> robotPathProgress;  // Fractional progress along current path segment
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        robotPathIndices[i] = 0;
        robotPathProgress[i] = 0.0f;
    }

    // Synchronization state. barrierCursor[r] is the barrier robot r is currently working
    // toward; stepCohort[s] is every robot allocated to step s. A robot that has reached its
    // barrier holds there until all of stepCohort[s] have reached theirs. With no barriers
    // supplied this stays empty and every robot runs freely, as it did before.
    std::map<uint32_t, size_t> barrierCursor;
    std::map<size_t, std::vector<uint32_t>> stepCohort;
    for (const auto& [robotId, robotBarrierList] : barriers) {
        barrierCursor[robotId] = 0;
        for (const auto& [stepIndex, pathIndex] : robotBarrierList) {
            (void)pathIndex;
            stepCohort[stepIndex].push_back(robotId);
        }
    }

    // Where robot r has to be to count as arrived at step s, or npos if r has no barrier there
    auto barrierIndexFor = [&barriers](uint32_t robotId, size_t stepIndex) -> size_t {
        auto it = barriers.find(robotId);
        if (it == barriers.end()) {
            return std::string::npos;
        }
        for (const auto& [step, pathIndex] : it->second) {
            if (step == stepIndex) {
                return pathIndex;
            }
        }
        return std::string::npos;
    };

    // A step releases once every robot allocated to it has reached its own barrier
    auto stepComplete = [&](size_t stepIndex) {
        auto cohort = stepCohort.find(stepIndex);
        if (cohort == stepCohort.end()) {
            return true;
        }
        for (uint32_t robotId : cohort->second) {
            const size_t needed = barrierIndexFor(robotId, stepIndex);
            if (needed != std::string::npos && robotPathIndices[robotId] < needed) {
                return false;
            }
        }
        return true;
    };

    // The barrier robot r is held at right now, or npos if it is free to keep moving
    auto activeBarrierFor = [&](uint32_t robotId) -> size_t {
        auto it = barriers.find(robotId);
        if (it == barriers.end()) {
            return std::string::npos;
        }
        const size_t cursor = barrierCursor[robotId];
        if (cursor >= it->second.size()) {
            return std::string::npos;  // past its last barrier
        }
        return it->second[cursor].second;
    };
    float animationSpeed = 0.02f;  // Fraction of waypoint to move per frame (0.0-1.0)
    constexpr uint32_t snapshotIntervals = 10;
    uint32_t nextSnapshot = 0;
    if (!snapshotDirectory.empty()) {
        std::filesystem::create_directories(snapshotDirectory);
    }

    // TS regions share one flat beige fill; the bold black borders below separate them,
    // which keeps the robot path overlays readable
    const Color beige = Color{238, 230, 209, 255};
    constexpr unsigned char pathAlpha = 110;  // translucent, so region labels show through

    // Flatten a robot color against the beige background at the path's alpha. Robots and
    // legend swatches use this so they read as the same color as the trail they leave.
    auto soften = [&beige](Color c) {
        auto mix = [](unsigned char fg, unsigned char bg) {
            return (unsigned char)((fg * pathAlpha + bg * (255 - pathAlpha)) / 255);
        };
        return Color{mix(c.r, beige.r), mix(c.g, beige.g), mix(c.b, beige.b), 255};
    };

    // Color palette for robots (distinct and visible)
    Color robotColors[] = {
        Color{255, 0, 0, 255},      // red
        Color{0, 0, 255, 255},      // blue
        Color{0, 255, 0, 255},      // green
        Color{255, 255, 0, 255},    // yellow
        Color{255, 0, 255, 255},    // magenta
        Color{0, 255, 255, 255},    // cyan
        Color{255, 127, 0, 255},    // orange
        Color{127, 0, 255, 255},    // purple
    };
    const int numRobotColors = sizeof(robotColors) / sizeof(robotColors[0]);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(beige);

        // Update robot positions along their paths, holding each one at its barrier until
        // the rest of the robots allocated to that step have arrived
        for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
            if (!paths.count(i) || paths[i].size() <= 1 || robotPathIndices[i] >= paths[i].size() - 1) {
                continue;
            }

            const size_t holdAt = activeBarrierFor(i);
            if (holdAt != std::string::npos && robotPathIndices[i] >= holdAt) {
                robotPathProgress[i] = 0.0f;  // parked at the task location, waiting
                continue;
            }

            robotPathProgress[i] += animationSpeed;
            if (robotPathProgress[i] >= 1.0f) {
                robotPathIndices[i]++;  // Move to next waypoint
                robotPathProgress[i] -= 1.0f;  // Reset progress
            }
        }

        // Release any step whose whole group has now arrived
        for (auto& [robotId, cursor] : barrierCursor) {
            auto it = barriers.find(robotId);
            if (it == barriers.end() || cursor >= it->second.size()) {
                continue;
            }
            const auto& [stepIndex, pathIndex] = it->second[cursor];
            if (robotPathIndices[robotId] >= pathIndex && stepComplete(stepIndex)) {
                ++cursor;
            }
        }

        // Draw grid cells with TS state coloring
        for (uint32_t y = 0; y < gridH; ++y) {
            for (uint32_t x = 0; x < gridW; ++x) {
                Point cellPos(x, y);
                int screenX, screenY;
                gridToScreen(cellPos, cellSize, margin, screenX, screenY);

                // Fill cell based on obstacle status and TS state
                if (env.isObstacle(cellPos)) {
                    DrawRectangle(screenX, screenY, cellSize, cellSize, BLACK);
                } else {
                    // All TS regions share one flat beige fill; regions are told apart by
                    // the bold borders and labels rather than by color
                    DrawRectangle(screenX, screenY, cellSize, cellSize, beige);
                    DrawRectangleLines(screenX, screenY, cellSize, cellSize, LIGHTGRAY);
                }
            }
        }

        // Draw bold borders between different TS state regions
        for (uint32_t y = 0; y < gridH; ++y) {
            for (uint32_t x = 0; x < gridW; ++x) {
                Point cellPos(x, y);
                uint32_t stateId = env.gridToTSStateId(cellPos);
                int screenX, screenY;
                gridToScreen(cellPos, cellSize, margin, screenX, screenY);

                // Check neighbors for state changes and draw bold borders
                // Right edge
                if (x + 1 < gridW) {
                    Point rightPos(x + 1, y);
                    if (env.gridToTSStateId(rightPos) != stateId) {
                        DrawLine(screenX + cellSize, screenY, screenX + cellSize, screenY + cellSize, BLACK);
                        DrawLine(screenX + cellSize + 1, screenY, screenX + cellSize + 1, screenY + cellSize, BLACK);
                    }
                }
                // Bottom edge
                if (y + 1 < gridH) {
                    Point bottomPos(x, y + 1);
                    if (env.gridToTSStateId(bottomPos) != stateId) {
                        DrawLine(screenX, screenY + cellSize, screenX + cellSize, screenY + cellSize, BLACK);
                        DrawLine(screenX, screenY + cellSize + 1, screenX + cellSize, screenY + cellSize + 1, BLACK);
                    }
                }
            }
        }

        // Label TS state regions with their IDs
        std::map<uint32_t, std::vector<Point>> stateRegions;
        for (uint32_t y = 0; y < gridH; ++y) {
            for (uint32_t x = 0; x < gridW; ++x) {
                Point cellPos(x, y);
                uint32_t stateId = env.gridToTSStateId(cellPos);
                if (!env.isObstacle(cellPos)) {
                    stateRegions[stateId].push_back(cellPos);
                }
            }
        }

        // Draw labels at the center of each region
        for (const auto& [stateId, cells] : stateRegions) {
            if (cells.empty()) continue;

            // Find center of region
            int centerX = 0, centerY = 0;
            for (const Point& p : cells) {
                centerX += p.getX();
                centerY += p.getY();
            }
            centerX /= cells.size();
            centerY /= cells.size();

            int screenX, screenY;
            gridToScreen(Point(centerX, centerY), cellSize, margin, screenX, screenY);
            screenX += cellSize / 2;
            screenY += cellSize / 2;

            std::string label = "TS" + std::to_string(stateId);
            DrawText(label.c_str(), screenX - 12, screenY - 8, 14, BLACK);
        }

        // Draw each robot's D* path
        for (const auto& [robotId, path] : paths) {
            Color pathColor = robotColors[robotId % numRobotColors];
            // Translucent so the region labels and overlapping paths stay visible underneath
            pathColor.a = pathAlpha;
            
            for (const Point& p : path) {
                int screenX, screenY;
                gridToScreen(p, cellSize, margin, screenX, screenY);
                DrawRectangle(screenX, screenY, cellSize, cellSize, pathColor);
            }
        }

        // Draw origin axes (X=red, Y=green)
        Point origin(0, 0);
        int originScreenX, originScreenY;
        gridToScreen(origin, cellSize, margin, originScreenX, originScreenY);
        int originCenterX = originScreenX + cellSize / 2;
        int originCenterY = originScreenY + cellSize / 2;
        
        // Draw X axis (horizontal, red)
        int axisLength = cellSize * 5;
        DrawLine(originCenterX, originCenterY, originCenterX + axisLength, originCenterY, RED);
        DrawText("X", originCenterX + axisLength + 5, originCenterY - 8, 14, RED);
        
        // Draw Y axis (vertical, green)
        DrawLine(originCenterX, originCenterY, originCenterX, originCenterY + axisLength, GREEN);
        DrawText("Y", originCenterX - 8, originCenterY + axisLength + 5, 14, GREEN);
        
        // Draw origin marker
        DrawCircle(originCenterX, originCenterY, 5, BLACK);
        DrawText("O", originCenterX - 4, originCenterY - 20, 14, BLACK);

        // Draw robots at their current positions along their paths
        for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
            Point robotPos;
            
            // Get robot position from animated path if available
            if (paths.count(i) && paths[i].size() > 0 && robotPathIndices[i] < paths[i].size()) {
                robotPos = paths[i][robotPathIndices[i]];
            } else {
                // Fallback to starting position if no path
                const std::vector<Robot*>& robots = mrs.getRobots();
                if (i < robots.size()) {
                    robotPos = robots[i]->getStartPosition();
                } else {
                    continue;  // Skip if no position available
                }
            }
            
            int screenX, screenY;
            gridToScreen(robotPos, cellSize, margin, screenX, screenY);

            Color robotColor = robotColors[i % numRobotColors];
            int centerX = screenX + cellSize / 2;
            int centerY = screenY + cellSize / 2;
            int radius = cellSize / 3;

            DrawCircle(centerX, centerY, radius, soften(robotColor));
            DrawCircleLines(centerX, centerY, radius + 2, BLACK);

            // Draw robot ID
            // Index by position: robot ids are caller-defined and need not equal the index
            std::string robotLabel = mrs.getRobots()[i]->getName();
            DrawText(robotLabel.c_str(), centerX - 8, centerY - 8, 14, BLACK);
        }

        // Draw legend
        int legendY = 10;
        DrawText("Robot Paths (D*):", 10, legendY, 12, BLACK);
        legendY += 20;

        // Robot legend
        for (uint32_t i = 0; i < mrs.getNumRobots() && i < 6; ++i) {
            Color robotColor = robotColors[i % numRobotColors];
            DrawCircle(17, legendY + 8, 5, soften(robotColor));
            // Index by position: robot ids are caller-defined and need not equal the index
            std::string label = mrs.getRobots()[i]->getName();
            DrawText(label.c_str(), 30, legendY, 12, BLACK);
            legendY += 20;
        }

        EndDrawing();

        if (!snapshotDirectory.empty() && nextSnapshot <= snapshotIntervals) {
            double totalProgress = 0.0;
            size_t activeRobotPaths = 0;
            for (const auto& [robotId, path] : paths) {
                if (path.size() < 2) continue;
                const double robotProgress =
                    (robotPathIndices[robotId] + robotPathProgress[robotId]) / (path.size() - 1);
                totalProgress += std::min(robotProgress, 1.0);
                ++activeRobotPaths;
            }

            if (activeRobotPaths > 0) {
                const double routeProgress = totalProgress / activeRobotPaths;
                const double targetProgress = static_cast<double>(nextSnapshot) / snapshotIntervals;
                if (routeProgress >= targetProgress) {
                    std::ostringstream filename;
                    filename << snapshotDirectory << "/allocation_frame_"
                             << std::setw(2) << std::setfill('0') << nextSnapshot << ".png";
                    TakeScreenshot(filename.str().c_str());
                    std::cout << "Saved allocation snapshot: " << filename.str() << std::endl;
                    ++nextSnapshot;
                }
            }
        }
    }

    CloseWindow();
}



