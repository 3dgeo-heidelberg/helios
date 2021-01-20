#pragma once

#include <SyncFileWriter.h>
#include <fstream>
#include "MathConverter.h"

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief SyncFileWriter implementation for simple text format
 */
class SimpleSyncFileWriter : public SyncFileWriter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Output file stream to be used by the simple synchronous file
     *  writer
     */
	std::ofstream ofs;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple synchronous file writer constructor
     * @param path Path to the output file
     * @param om Open mode for the file (append by default)
     */
	explicit SimpleSyncFileWriter(
	    const std::string& path,
	    std::ios_base::openmode om = std::ios_base::app
	    ) :
	    SyncFileWriter(path)
    {
		// Open file for writing ...
		ofs.open(path, om);
	}

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Write measurement to text file
     * @see SyncFileWriter::_write(Measurement const &, glm::dvec3 const &)
     */
    void _write(Measurement const &m, glm::dvec3 const & shift) override{
        ofs << measurementToString(m, shift);
    }

    /**
     * @brief Write fullwave to text file
     * @see SimpleSyncFileWriter::_write(
     *  std::vector<double> const &,
     *  int, double, double, glm::dvec3, glm::dvec3, long
     * )
     */
    void _write(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ) override {
        ofs << fullwaveToString(
            fullwave,
            fullwaveIndex,
            minTime,
            maxTime,
            beamOrigin,
            beamDir,
            gpsTime
        );
    }

    /**
     * @brief Write trajectory to compressed file
     * @see SimpleSyncFileWriter::_write(
     *  long,
     *  double, double, double,
     *  double, double, double
     * )
     */
    void _write(Trajectory const &t) override {
        ofs << trajectoryToString(t);
    }

    // ***  F I N I S H  *** //
    // ********************* //
    /**
     * @brief SimpleSyncFileWriter finish method does not do nothing. The
     * writing operations are guaranteed to be done after the instance has
     * been destroyed.
     */
    void finish() override {};

protected:
    /**
     * @brief Build a string from measurement data
     * @param m Measurement data itself
     * @param shift Shift for the measurement coordinates
     * @return String with measurement data
     */
    virtual std::string measurementToString(
        Measurement const &m,
        glm::dvec3 const & shift
    ){
        glm::dvec3 shifted = m.position + shift;
        std::stringstream ss;
        ss << std::setprecision(4) << std::fixed;
        ss  << shifted.x << " "
            << shifted.y << " "
            << shifted.z << " "
            << m.intensity << " "
            << m.echo_width << " "
            << m.returnNumber << " "
            << m.pulseReturnNumber << " "
            << m.fullwaveIndex << " "
            << m.hitObjectId << " "
            << m.classification << " "
            << std::setprecision(4) << std::fixed
            << ((double)m.gpsTime) / 1000.0 << std::endl;
        return ss.str();
    }

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

    /**
     * @brief Build a string from fullwave data
     * @return String with fullwave data
     */
    virtual std::string fullwaveToString(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ){
        std::stringstream ss;
        ss << std::setprecision(4) << std::fixed;
        ss << fullwaveIndex << " "
           << beamOrigin.x << " "
           << beamOrigin.y << " "
           << beamOrigin.z << " "
           << beamDir.x << " "
           << beamDir.y << " "
           << beamDir.z << " "
           << minTime << " "
           << maxTime << " "
           << ((double)gpsTime)/1000.0 << " ";

        std::copy(
            fullwave.begin(),
            fullwave.end()-1,
            std::ostream_iterator<double>(ss, " ")
        );
        ss << fullwave.back();
        ss << "\n";
        return ss.str();
    }
};