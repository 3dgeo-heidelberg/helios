#pragma once

#include <memory>

#include "ScannerSettings.h"
#include "PlatformSettings.h"
#include <platform/trajectory/TrajectorySettings.h>

class ScanningStrip;

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
	/**
	 * @brief Trajectory settings for the leg
	 * @see TrajectorySettings
	 */
	std::shared_ptr<TrajectorySettings> mTrajectorySettings = nullptr;

  /**
   * @brief Boolean flag to store whether the leg was already processed.
   */
  bool wasProcessed{};
private:
    /**
     * @brief Distance to the next leg
     */
	double length = 0;	// Distance to the next leg
	/**
	 * @brief The serial non negative integer unique identifier for the leg.
	 *  If it is a negative integer, it means that the serial identifier is not
	 *  valid. It is, the serial identifier does not univocally identify the
	 *  leg
	 */
	int serialId;
	/**
	 * @brief The strip the leg belongs to. It is a nullptr if the leg does
	 *  not belong to any strip at all.
	 */
	std::shared_ptr<ScanningStrip> strip;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default leg constructor
     */
    Leg() : Leg(0, -1, nullptr) {}
    /**
     * @brief Constructor for leg with input arguments
     * @see Leg::length
     * @see Leg::serialId
     * @see Leg::strip
     */
    Leg(
        double const length,
        int const serialId,
        std::shared_ptr<ScanningStrip> strip
    );
    /**
     * @brief Copy constructor for leg
     *
     * <b><span color="red">WARNING!</span></b> using this copy constructor
     *  will copy the serialId and the strip from given leg. However, the strip
     *  itself is not updated to include the copied leg. Thus, it is
     *  necessarily to either update the strip so the serial id points to the
     *  new leg or to update serial id of copy and include it in strip if
     *  desired.
     *
     * @param leg Leg to be copied
     */
    Leg(Leg &leg);
    virtual ~Leg() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain distance to next leg
     * @return Distance to next leg
     * @see Leg::length
     */
    inline double getLength() const {return this->length;}
    /**
     * @brief Set distance to next leg
     * @param length New distance to next leg
     * @see Leg::length
     */
	inline void setLength(double const length) {this->length = length;}
	/**
	 * @brief Obtain leg scanner settings by reference
	 * @return Reference to leg scanner settings
	 * @see Leg::mScannerSettings
	 */
	inline ScannerSettings & getScannerSettings() const
	{return *mScannerSettings;}
	/**
	 * @brief Obtain leg platform settings by reference
	 * @return Reference to leg platform settings
	 * @see Leg::mPlatformSettings
	 */
	PlatformSettings & getPlatformSettings() const {return *mPlatformSettings;}
	/**
	 * @brief Obtain the serial identifier of the leg
	 * @return Leg serial identifier
	 * @see Leg::serialId
	 */
	inline int getSerialId() const {return serialId;}
	/**
	 * @brief Set the leg serial identifier
	 * @param serialId New serial identifier for the leg
	 * @see Leg::serialId
	 */
	inline void setSerialId(int const serialId) {this->serialId = serialId;}
	/**
	 * @brief Obtain the scanning strip of the leg
	 * @return Leg scanning strip
	 * @see Leg::strip
	 */
	inline std::shared_ptr<ScanningStrip> getStrip() const {return strip;}
	/**
	 * @brief Set the leg scanning strip
	 * @param strip New scanning strip for the leg
	 * @see Leg::strip
	 */
	inline void setStrip(std::shared_ptr<ScanningStrip> strip)
	{this->strip = strip;}
	/**
	 * @brief Check whether the leg belongs to a strip (true) or not (false)
	 * @return True if the leg belongs to a strip, false otherwise
	 */
	inline bool isContainedInAStrip() const {return strip!=nullptr;}
};
