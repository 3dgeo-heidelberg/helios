#pragma once

#include <memory>

#include "ScannerSettings.h"
#include "PlatformSettings.h"

/**
 * @brief Class representing a survey leg
 */
class Leg {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
	/**
	 * @brief Scanner settings for the leg
	 * @se ScannerSettings
	 */
	std::shared_ptr<ScannerSettings> mScannerSettings;
	/**
	 * @brief Platform settings for the leg
	 * @see PlatformSettings
	 */
	std::shared_ptr<PlatformSettings> mPlatformSettings;

private:
    /**
     * @brief Distance to the next leg
     */
	double length = 0;	// Distance to the next leg

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default leg constructor
     */
    Leg() = default;
    Leg(Leg &leg);
    virtual ~Leg() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain distance to next leg
     * @return Distance to next leg
     * @see Leg::length
     */
    double getLength() {return this->length;}
    /**
     * @brief Set distance to next leg
     * @param length New distance to next leg
     * @see Leg::length
     */
	void setLength(double length) {this->length = length;}
	/**
	 * @brief Obtain leg scanner settings by reference
	 * @return Reference to leg scanner settings
	 * @see Leg::mScannerSettings
	 */
	ScannerSettings & getScannerSettings() {return *mScannerSettings;}
	/**
	 * @brief Obtain leg platform settings by reference
	 * @return Reference to leg platform settings
	 * @see Leg::mPlatformSettings
	 */
	PlatformSettings & getPlatformSettings() {return *mPlatformSettings;}
};