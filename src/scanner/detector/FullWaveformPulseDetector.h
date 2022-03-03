#pragma once

#include <AbstractDetector.h>
#include <filems/write/SyncFileWriter.h>

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include <limits>
#include <string>

using helios::filems::SyncFileWriter;

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
	/**
	 * @see AbstractDetector::applySettings
	 */
	void applySettings(std::shared_ptr<ScannerSettings> & settings) override;
};