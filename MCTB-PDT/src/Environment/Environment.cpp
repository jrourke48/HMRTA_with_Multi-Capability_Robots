#include "Environment/Environment.h"
#include "Environment/Point.h"
#include <iostream>
#include <iomanip>

/**
 * Environment - Constructor
 */
Environment::Environment(TS* ts, GridWorld* grid)
    : transitionSystem(ts), gridWorld(grid) {
}

/**
 * Environment - Destructor
 */
Environment::~Environment() {
    // don't delete the pointers as they're managed externally
}

/**
 * getInitialState - Get the initial state of the transition system
 */
uint32_t Environment::getInitialState() const {
    if (!transitionSystem || transitionSystem->getInitialStates().empty()) {
        return 0;
    }
    return transitionSystem->getInitialStates()[0];
}

/**
 * getSuccessorStates - Get successor states for a given state
 */
std::vector<uint32_t> Environment::getSuccessorStates(uint32_t stateId) const {
    if (!transitionSystem) {
        return std::vector<uint32_t>();
    }
    
    // Get adjacent states as uint16_t and convert to uint32_t
    std::vector<uint16_t> adjacentIds = transitionSystem->getAdjacent(static_cast<uint16_t>(stateId));
    std::vector<uint32_t> result;
    for (uint16_t id : adjacentIds) {
        result.push_back(static_cast<uint32_t>(id));
    }
    return result;
}

/**
 * isObstacle - Check if a grid cell is an obstacle
 */
bool Environment::isObstacle(Point p) const {
    if (!gridWorld) {
        return false;
    }

    return gridWorld->isObstacle(p);
}

/**
 * isFree - Check if a grid cell is free
 */
bool Environment::isFree(Point p) const {
    if (!gridWorld) {
        return true;
    }
    return gridWorld->isFree(p);
}

/**
 * addObstacle - Add an obstacle at a specific grid cell
 * @param p The grid cell where the obstacle should be added
 * Adds an obstacle to the grid world at the specified location.
 * Note: TS center points cannot be obstacles.
 */
void Environment::addObstacle(Point p) {
    if (!gridWorld) {
        return;
    }
    
    // Check if this point is a TS state center
    for (const auto& mapping : stateIdToGridMap) {
        if (mapping.second.center.getX() == p.getX() && mapping.second.center.getY() == p.getY()) {
            // Cannot place obstacle at TS center point
            std::cout << "Cannot add obstacle at TS center point (" << p.getX() << ", " << p.getY() << ")" << std::endl;
            return;
        }
    }
    
    gridWorld->addObstacle(p);
}
/**
 * isValidState - Check if a state is valid in the transition system
 */
bool Environment::isValidState(uint32_t stateId) const {
    if (!transitionSystem) {
        return false;
    }
    // Check if state exists in the state-to-grid mapping
    return stateIdToGridMap.find(stateId) != stateIdToGridMap.end();
}

/**
 * getNumStates - Get the number of states in the transition system
 */
uint32_t Environment::getNumStates() const {
    if (!transitionSystem) {
        return 0;
    }
    return transitionSystem->getNumStates();
}

// Bidirectional mapping between GridWorld and TransitionSystem
uint32_t Environment::gridToTSStateId(Point p) const {
    // Find which state's region contains this grid point
    for (const auto& mapping : stateIdToGridMap) {
        const stateGridMapping& region = mapping.second;
        uint16_t half_width = region.width / 2;
        uint16_t half_height = region.height / 2;
        
        if (p.getX() >= region.center.getX() - half_width && p.getX() < region.center.getX() + half_width &&
            p.getY() >= region.center.getY() - half_height && p.getY() < region.center.getY() + half_height) {
            return mapping.first;
        }
    }
    return 0; // Default to state 0 if not found
}

void Environment::mapTSStateToGrid(uint32_t stateId, Point center, uint16_t width, uint16_t height) {
    stateIdToGridMap[stateId] = {center, width, height};
}
void Environment::print_Environment() const {
    if (!gridWorld || !transitionSystem) {
        std::cout << "Cannot print environment: GridWorld or TransitionSystem is null" << std::endl;
        return;
    }
    
    uint32_t width = gridWorld->getWidth();
    uint32_t height = gridWorld->getHeight();
    
    std::cout << "\n========== ASCII Environment Visualization (TS Regions Overlay) ==========" << std::endl;
    std::cout << "Grid: " << width << "x" << height << " | States: " << transitionSystem->getNumStates() << std::endl;
    std::cout << "Legend: [#] = obstacle, [0-9] = state center, || = region boundary\n" << std::endl;
    
    // Print column headers
    std::cout << "     ";
    for (uint32_t x = 0; x < width; x++) {
        std::cout << std::setw(3) << (x % 10);
    }
    std::cout << std::endl;
    
    // Print grid with region boundaries and horizontal separators
    for (uint32_t y = 0; y < height; y++) {
        // Horizontal separator line
        std::cout << "     ";
        for (uint32_t x = 0; x < width; x++) {
            // Check for region boundary (vertical)
            Point p(x, y);
            bool isLeftBoundary = (x == 0) || (gridToTSStateId(Point(x - 1, y)) != gridToTSStateId(p));
            bool isRightBoundary = (x == width - 1) || (gridToTSStateId(Point(x + 1, y)) != gridToTSStateId(p));
            
            if (isLeftBoundary) std::cout << "||";
            else std::cout << "| ";
            std::cout << "_";
            if (isRightBoundary && x == width - 1) std::cout << "||";
        }
        std::cout << std::endl;
        
        // Cell content row
        std::cout << std::setw(3) << y << " |";
        for (uint32_t x = 0; x < width; x++) {
            Point p(x, y);
            char cell = ' ';
            
            // Check if obstacle
            if (gridWorld->isObstacle(p)) {
                cell = '#';
            } else {
                // Check if this is a region center
                // bool isCenter = false;
                for (const auto& mapping : stateIdToGridMap) {
                    if (mapping.second.center.getX() == x && mapping.second.center.getY() == y) {
                        uint32_t stateId = mapping.first;
                        if (stateId < 10) {
                            cell = '0' + stateId;
                        } else {
                            cell = 'A' + (stateId - 10);
                        }
                        // isCenter = true;
                        break;
                    }
                }
            }
            
            // Check if this cell is on a region boundary
            bool isLeftBoundary = (x == 0) || (gridToTSStateId(Point(x - 1, y)) != gridToTSStateId(p));
            bool isRightBoundary = (x == width - 1) || (gridToTSStateId(Point(x + 1, y)) != gridToTSStateId(p));
            
            if (isLeftBoundary) std::cout << "||";
            else std::cout << "| ";
            
            std::cout << cell;
            
            if (isRightBoundary && x == width - 1) std::cout << "||";
        }
        std::cout << std::endl;
    }
    
    // Final bottom separator
    std::cout << "     ";
    for (uint32_t x = 0; x < width; x++) {
        Point p(x, height - 1);
        bool isLeftBoundary = (x == 0) || (gridToTSStateId(Point(x - 1, height - 1)) != gridToTSStateId(p));
        bool isRightBoundary = (x == width - 1) || (gridToTSStateId(Point(x + 1, height - 1)) != gridToTSStateId(p));
        
        if (isLeftBoundary) std::cout << "||";
        else std::cout << "| ";
        std::cout << "_";
        if (isRightBoundary && x == width - 1) std::cout << "||";
    }
    std::cout << std::endl;
    
    // Print state region information
    std::cout << "\nState Region Mappings:" << std::endl;
    for (const auto& mapping : stateIdToGridMap) {
        uint32_t stateId = mapping.first;
        const auto& region = mapping.second;
        std::cout << "  State " << stateId << ": Center (" << region.center.getX() << ", " << region.center.getY() 
                  << "), Size " << region.width << "x" << region.height << std::endl;
    }
    
    std::cout << "\n============================================================================\n" << std::endl;
}
