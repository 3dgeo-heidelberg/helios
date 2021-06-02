#pragma once

#include "AbstractBeamDeflector.h"

/**
 * @brief Class representing a polygon mirror beam deflector
 */
class PolygonMirrorBeamDeflector : public AbstractBeamDeflector {

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Effective scan anngle (radians)
     */
	double cfg_device_scanAngleEffective_rad = 0;
	/**
	 * @brief Maximum effective scan angle (radians)
	 */
	double cfg_device_scanAngleEffectiveMax_rad = 0;

public:
    // ***  CONSTRUCTION  / DESTRUCTION  *** //
    // ************************************* //
    /**
     * @brief Constructor for polygon mirror beam deflector
     * @see PolygonMirrorBeamDeflector::cfg_device_scanAngleEffectiveMax_rad
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	PolygonMirrorBeamDeflector(
		double _scanFreqMax_Hz,
		double _scanFreqMin_Hz,
		double _scanAngleMax_rad,
		double _scanAngleEffectiveMax_rad
    ):
    AbstractBeamDeflector(_scanAngleMax_rad, _scanFreqMax_Hz, _scanFreqMin_Hz)
{
    this->cfg_device_scanAngleEffectiveMax_rad = _scanAngleEffectiveMax_rad;
    this->cfg_device_scanAngleEffective_rad = _scanAngleEffectiveMax_rad;
}
    std::shared_ptr<AbstractBeamDeflector> clone() override;
    void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see AbstractBeamDeflector::applySettings
     */
    void applySettings(std::shared_ptr<ScannerSettings>) override;
    /**
     * @see AbstractBeamDeflector::doSimStep
     */
    void doSimStep() override;
	/**
	 * @see AbstractBeamDeflector::lastPulseLeftDevice
	 */
	bool lastPulseLeftDevice() override;
};