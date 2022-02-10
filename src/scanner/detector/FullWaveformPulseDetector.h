#pragma once

#include "AbstractDetector.h"

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include <limits>
#include <string>

/**
 * @brief Concrete implementation of abstract detector to compute full
 * waveform pulses
 *
 * @see AbstractDetector
 */
class FullWaveformPulseDetector : public AbstractDetector {
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Synchronous file writer used to handle output
     * @see SyncFileWriter
     */
	std::shared_ptr<SyncFileWriter> fw_sfw;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base constructor for full waveform pulse detector
     * @see AbstractDetector::AbstractDetector(std::shared_ptr<Scanner>,
     *  double, double)
     */
	FullWaveformPulseDetector(
	    std::shared_ptr<Scanner> scanner,
	    double accuracy_m,
	    double range_min,
	    double range_max=std::numeric_limits<double>::max()
    ) : AbstractDetector(scanner, accuracy_m, range_min, range_max) {}
    std::shared_ptr<AbstractDetector> clone() override;
    void _clone(std::shared_ptr<AbstractDetector> ad) override;

    // ***  M E T H O D S  *** //
    // *********************** //
	/**
	 * @see AbstractDetector::shutdown
	 */
	void shutdown() override;
	void writeFullWave(
	    std::vector<double> & fullwave,
	    int fullwave_index,
	    double min_time,
	    double max_time,
	    glm::dvec3& beamOrigin,
	    glm::dvec3& beamDir,
        double gpstime
    );
	/**
	 * @see AbstractDetector::applySettings
	 */
	void applySettings(std::shared_ptr<ScannerSettings> & settings) override;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Set the path to output file
     * @param path Path to output file
     * @param fname Name of output file
     * @param computeWaveform Flag to specify if waveform must be computed
     * (true) or not (false)
     */
    void setOutputFilePath(
        std::string path,
        std::string fname="fullwave.txt",
        bool computeWaveform=true
    );
};