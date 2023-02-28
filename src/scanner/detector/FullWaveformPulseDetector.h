#pragma once

#include <AbstractDetector.h>

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