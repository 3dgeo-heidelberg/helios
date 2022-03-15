#pragma once

#include <filems/write/strategies/WriteStrategy.h>
#include <scanner/Trajectory.h>

#include <fstream>
#include <sstream>

namespace helios { namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write
 *  trajectory to a file.
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFileTrajectoryWriter
 */
class DirectTrajectoryWriteStrategy :
    public WriteStrategy<Trajectory const &>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The output file stream to do the writing
     */
    std::ofstream &ofs;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for direct trajectory write strategy
     * @param DirectTrajectoryWriteStrategy::ofs
     */
    DirectTrajectoryWriteStrategy(std::ofstream &ofs) : ofs(ofs) {}
    virtual ~DirectTrajectoryWriteStrategy() = default;

    // ***  WRITE STRATEGY INTERFACE *** //
    // ********************************* //
    /**
     * @brief Write trajectory to file
     * @param t Trajectory to be written
     * @see Trajectory
     */
    void write(Trajectory const &t) override {
        ofs << trajectoryToString(t);
    }

protected:
    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Build a string from trajectory data
     * @return String with trajectory data
     */
    virtual std::string trajectoryToString(Trajectory const &t){
        std::stringstream ss;
        ss  << std::setprecision(4) << std::fixed;
        ss  << t.position.x << " "
            << t.position.y << " "
            << t.position.z << " "
            << ((double)t.gpsTime) / 1000.0 << " "
            << MathConverter::radiansToDegrees(t.roll) << " "
            << MathConverter::radiansToDegrees(t.pitch) << " "
            << MathConverter::radiansToDegrees(t.yaw) << std::endl;
        return ss.str();
    }
};

    }}