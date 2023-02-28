#pragma once

#include <glm/glm.hpp>

#include <ostream>

/**
 * @brief Class representing a concrete trajectory definition
 */
class Trajectory{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief GPS time in nanoseconds identifying the moment at which
     * trajectory is registered
     */
    double gpsTime = 0;
    /**
     * @brief Trajectory position/coordinates
     */
    glm::dvec3 position = glm::dvec3(0, 0, 0);
    /**
     * @brief Roll angle in radians
     */
    double roll = 0;
    /**
     * @brief Pitch angle in radians
     */
    double pitch = 0;
    /**
     * @brief Yaw angle in radians
     */
    double yaw = 0;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default trajectory constructor
     */
    Trajectory() = default;
    /**
     * @brief Trajectory constructor
     * @param gpsTime GPS time in nanoseconds
     * @param position 3D position
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     */
    Trajectory(
        double gpsTime,
        glm::dvec3 position,
        double roll,
        double pitch,
        double yaw
    ) :
        gpsTime(gpsTime),
        position(position),
        roll(roll),
        pitch(pitch),
        yaw(yaw)
    {}
    Trajectory(const Trajectory &t){
        gpsTime = t.gpsTime;
        position = glm::dvec3(t.position);
        roll = t.roll;
        pitch = t.pitch;
        yaw = t.yaw;
    }

    // ***  O P E R A T O R S  *** //
    // *************************** //
    friend std::ostream & operator << (std::ostream &out, Trajectory &t){
        out << t.gpsTime << "," << t.position.x << "," << t.position.y << ","
            << t.position.z << "," << t.roll << ","
            << t.pitch << "," << t.yaw;
        return out;
    }
};