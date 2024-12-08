#pragma once

#include "AbstractBeamDeflector.h"
/**
 * @brief Class representing a conic beam deflector
 */
class ConicBeamDeflector : public AbstractBeamDeflector {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** /7
    /**
     * @brief Rotation used to create the radius of the cone
     */
    Rotation r1;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for conic beam deflector
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	ConicBeamDeflector(
	    double scanAngleMax_rad,
	    double scanFreqMax_Hz,
	    double scanFreqMin_Hz
    ) :
        AbstractBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz)
    {}
    std::shared_ptr<AbstractBeamDeflector> clone() override;
    void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;


    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see AbstractBeamDeflector::applySettings
     */
	void applySettings(std::shared_ptr<ScannerSettings> settings) override;
	/**
	 * @see AbstractBeamDeflector::doSimStep
	 */
	void doSimStep() override;
    
    /**
	 * @see AbstractBeamDeflector::getOpticsType
	 */
    std::string getOpticsType() const override {
        return "CONIC";
    }
};