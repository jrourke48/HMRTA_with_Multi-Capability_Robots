#ifndef ROBOT_H
#define ROBOT_H

#include "RobotCapabilities.h"
#include "../Environment/Point.h"
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>

/**
 * Robot - Represents a single robot in the multi-robot system
 */
class Robot {
private:
    uint32_t robotId;
    std::string name;
    Point startPosition;
    Point currentPosition;
    uint8_t velo = 1; // Default velocity in gridpoints/second (can be modified as needed)
    std::vector<bool> capabilities;  // Index corresponds to RobotCapability enum value
    
public:
    // Constructor
    Robot(uint32_t id, const std::string& robotName, const Point& startPos = Point());
    
    // Destructor
    ~Robot();
    
    // Getters
    uint32_t getRobotId() const { return robotId; }
    const std::string& getName() const { return name; }
    const Point& getStartPosition() const { return startPosition; }
    const Point& getCurrentPosition() const { return currentPosition; }
    uint8_t getVelocity() const { return velo; }
    const std::vector<bool>& getCapabilities() const { return capabilities; }
    
    // Setters
    void setName(const std::string& robotName) { name = robotName; }
    void setCurrentPosition(const Point& position) { currentPosition = position; }
    void setVelocity(uint8_t velocity) { velo = velocity; }
    void initializeCapabilities(size_t numCapabilities);
    
    // Capability management
    bool hasCapability(RobotCapability cap) const;
    void setCapability(RobotCapability cap, bool enabled);
    void enableCapability(RobotCapability cap);
    void disableCapability(RobotCapability cap);
    void clearCapabilities();

    //get updated time for robot
    uint16_t getUpdatedTime(uint16_t currentTime, const Point& targetPosition) const;
    uint16_t getTravelTime(const Point& startPosition, const Point& targetPosition) const;
};

#endif // ROBOT_H
