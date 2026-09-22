#include "Environment/gridvis.h"
#include "Environment/DStarGridWorld.h"
#include <raylib.h>
#include <iostream>
#include <vector>
#include <cmath>

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
    const std::vector<Tree_Node*>& optimalPath
) {
    RobotPathMap robotPaths;
    
    // Initialize empty paths for all robots
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        robotPaths[i] = std::vector<Point>();
    }
    
    // Track the last position of each robot for continuous path segments
    std::map<uint32_t, Point> robotLastPositions;
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        const std::vector<Point>& positions = mrs.getRobotPositions();
        if (i < positions.size()) {
            robotLastPositions[i] = positions[i];
        }
    }
    
    // For each node in the optimal path
    for (const Tree_Node* node : optimalPath) {
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
                const std::vector<Point>& positions = mrs.getRobotPositions();
                if (robotId < positions.size()) {
                    currentPos = positions[robotId];
                } else {
                    continue;
                }
            }
            
            // Get goal position from node (updated with circular arrangement)
            if (robotId >= nodePositions.size()) {
                continue;
            }
            Point goalPos = nodePositions[robotId];
            
            // Compute D* path from current position to goal
            std::vector<Point> segmentPath = compute_dstar_path_single(env, currentPos, goalPos);
            
            // Skip if path is empty
            if (segmentPath.empty()) {
                continue;
            }
            
            // Append path, avoiding duplicate endpoints
            for (size_t i = 0; i < segmentPath.size(); ++i) {
                if (i == 0 && !robotPaths[robotId].empty()) {
                    continue;  // Skip first point if it's a duplicate of last point
                }
                robotPaths[robotId].push_back(segmentPath[i]);
            }
            
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
    const char* windowTitle
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
    if (paths.empty() && !optimalPath.empty()) {
        paths = compute_dstar_paths(env, mrs, optimalPath);
    }

    // Animation state: track current position index for each robot on its path
    std::map<uint32_t, size_t> robotPathIndices;
    std::map<uint32_t, float> robotPathProgress;  // Fractional progress along current path segment
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        robotPathIndices[i] = 0;
        robotPathProgress[i] = 0.0f;
    }
    float animationSpeed = 0.02f;  // Fraction of waypoint to move per frame (0.0-1.0)

    // Color palette for TS states
    Color stateColors[] = {
        Color{100, 200, 100, 180},  // green
        Color{100, 150, 255, 180},  // blue
        Color{255, 200, 100, 180},  // orange
        Color{200, 100, 255, 180},  // purple
        Color{255, 100, 150, 180},  // pink
        Color{100, 255, 200, 180},  // cyan
        Color{200, 200, 100, 180},  // olive
        Color{150, 100, 200, 180},  // violet
    };
    const int numStateColors = sizeof(stateColors) / sizeof(stateColors[0]);

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
        ClearBackground(RAYWHITE);

        // Update robot positions along their paths
        for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
            if (paths.count(i) && paths[i].size() > 1 && robotPathIndices[i] < paths[i].size() - 1) {
                robotPathProgress[i] += animationSpeed;
                if (robotPathProgress[i] >= 1.0f) {
                    robotPathIndices[i]++;  // Move to next waypoint
                    robotPathProgress[i] -= 1.0f;  // Reset progress
                }
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
                    DrawRectangleLines(screenX, screenY, cellSize, cellSize, RED);  // Red outline for obstacles
                } else {
                    // Color by TS state
                    uint32_t stateId = env.gridToTSStateId(cellPos);
                    Color stateCol = stateColors[stateId % numStateColors];
                    DrawRectangle(screenX, screenY, cellSize, cellSize, stateCol);
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
            // Use full opacity to match robot color exactly
            pathColor.a = 255;
            
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
                std::vector<Point> robotPositions = mrs.getRobotPositions();
                if (i < robotPositions.size()) {
                    robotPos = robotPositions[i];
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

            DrawCircle(centerX, centerY, radius, robotColor);
            DrawCircleLines(centerX, centerY, radius + 2, BLACK);

            // Draw robot ID
            std::string robotLabel = mrs.getRobot(i + 1)->getName();
            DrawText(robotLabel.c_str(), centerX - 8, centerY - 8, 14, WHITE);
        }

        // Draw legend
        int legendY = 10;
        DrawText("Robot Paths (D*):", 10, legendY, 12, BLACK);
        legendY += 20;

        // Robot legend
        for (uint32_t i = 0; i < mrs.getNumRobots() && i < 6; ++i) {
            Color robotColor = robotColors[i % numRobotColors];
            DrawCircle(17, legendY + 8, 5, robotColor);
            std::string label = mrs.getRobot(i)->getName();
            DrawText(label.c_str(), 30, legendY, 12, BLACK);
            legendY += 20;
        }

        EndDrawing();
    }

    CloseWindow();
}



