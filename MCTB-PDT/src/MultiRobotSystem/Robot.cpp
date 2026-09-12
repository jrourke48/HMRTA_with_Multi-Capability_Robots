#include "MultiRobotSystem/Robot.h"

/**
 * Robot - Constructor
 */
Robot::Robot(uint32_t id, const std::string& robotName, const Point& startPos)
    : robotId(id), name(robotName), startPosition(startPos), currentPosition(startPos) {
    // Initialize capabilities vector with 17 slots (one per RobotCapability enum)
    capabilities.resize(17, false);
}

/**
 * Robot - Destructor
 */
Robot::~Robot() {
}

/**
 * initializeCapabilities - Initialize the capabilities vector with a specific size
 */
void Robot::initializeCapabilities(size_t numCapabilities) {
    capabilities.clear();
    capabilities.resize(numCapabilities, false);
}

/**
 * hasCapability - Check if robot has a specific capability (O(1) lookup)
 */
bool Robot::hasCapability(RobotCapability cap) const {
    size_t capIndex = static_cast<size_t>(cap);
    if (capIndex >= capabilities.size()) {
        return false;
    }
    return capabilities[capIndex];
}

/**
 * setCapability - Enable or disable a specific capability
 */
void Robot::setCapability(RobotCapability cap, bool enabled) {
    size_t capIndex = static_cast<size_t>(cap);
    if (capIndex >= capabilities.size()) {
        capabilities.resize(capIndex + 1, false);
    }
    capabilities[capIndex] = enabled;
}

/**
 * enableCapability - Enable a capability
 */
void Robot::enableCapability(RobotCapability cap) {
    setCapability(cap, true);
}

/**
 * disableCapability - Disable a capability
 */
void Robot::disableCapability(RobotCapability cap) {
    setCapability(cap, false);
}

/**
 * clearCapabilities - Disable all capabilities
 */
void Robot::clearCapabilities() {
    std::fill(capabilities.begin(), capabilities.end(), false);
}

/**
 * getUpdatedTime - Calculate the updated time for the robot to reach a target position
 * This is a simple heuristic based on Manhattan distance and velocity. It can be enhanced with more complex pathfinding if needed.
 */
uint16_t Robot::getUpdatedTime(uint16_t currentTime, const Point& targetPosition) const {
    // Calculate Manhattan distance from current position to target position
    uint16_t distance = currentPosition.distance(targetPosition);
    // Calculate time based on velocity
    uint16_t travelTime = static_cast<uint16_t>(distance / velo);
    return currentTime + travelTime;
}

/**
 * getTravelTime - Calculate the travel time for the robot to move from a start position to a target position
 * This is a simple heuristic based on Manhattan distance and velocity. It can be enhanced with more complex pathfinding if needed.
 */
uint16_t Robot::getTravelTime(const Point& startPosition, const Point& targetPosition) const {
    // Calculate Manhattan distance from start position to target position
    uint16_t distance = startPosition.distance(targetPosition);
    // Calculate time based on velocity
    uint16_t travelTime = static_cast<uint16_t>(distance / velo);
    return travelTime;
}
