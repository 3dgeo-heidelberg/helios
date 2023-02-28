#pragma once

#include "Asset.h"

/**
 * @brief Full Waveform settings
 */
class FWFSettings : public Asset {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Bin size for discretization (nanoseconds)
     */
    double binSize_ns = 0.25;
    /**
     * @brief Minimum echo width
     */
	double minEchoWidth = 2.5;
	/**
	 * @brief Peak energy
	 */
	double peakEnergy = 500.0;
	/**
	 * @brief Aperture diameter
	 */
	double apertureDiameter = 0.15;
	/**
	 * @brief Scanner efficiency
	 */
	double scannerEfficiency = 0.9;
	/**
	 * @brief Atmospheric visibility
	 */
	double atmosphericVisibility = 0.9;
	/**
	 * @brief Scanner wave length
	 */
	double scannerWaveLength = 1550.0;
	/**
	 * @brief Beam divergence (radians)
	 */
	double beamDivergence_rad = 0.0003;
	/**
	 * @brief Pulse length (nanoseconds)
	 */
	double pulseLength_ns = 4.0;
	/**
	 * @brief Beam sample quality
	 */
	int beamSampleQuality = 3;
	/**
	 * @brief Window size to iterate over discretization (nanoseconds)
	 */
	double winSize_ns = pulseLength_ns / 4.0;
	/**
	 * @brief Max full wave range (nanoseconds)
	 */
	double maxFullwaveRange_ns = 0.0;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Default constructor for full waveform settings
	 */
	FWFSettings() {}
};