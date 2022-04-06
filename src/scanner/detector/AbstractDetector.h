#pragma once

#include <string>
#include <unordered_map>

#include <Scanner.h>
#include <ScannerSettings.h>
#include <Measurement.h>
#include <util/PointcloudYielder.h>

namespace helios { namespace filems { class FMSFacade; }}
using helios::filems::FMSFacade;


/**
 * @brief Base abstract class for detectors
 */
class AbstractDetector {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Scanner which the detector belongs to
     */
	std::shared_ptr<Scanner> scanner = nullptr;

protected:
	/**
	 * @brief Main facade to file management system
	 */
	std::shared_ptr<FMSFacade> fms = nullptr;

public:
	/**
	 * @brief The point cloud yielder which handles point cloud building from
	 *  measurements
	 */
	std::shared_ptr<PointcloudYielder> pcloudYielder = nullptr;

	/**
	 * @brief Detector accuracy in meters
	 */
	double cfg_device_accuracy_m = 0;
	/**
	 * @brief Minimum range for detector in meters
	 */
	double cfg_device_rangeMin_m = 0;
	/**
	 * @brief Maximum range for detector in meters
	 */
	double cfg_device_rangeMax_m;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Base constructor for abstract detector
	 * @see AbstractDetector::scanner
	 * @see AbstractDetector::accuracy_m
	 * @see AbstractDetector::rangeMin_m
	 */
	AbstractDetector(
	    std::shared_ptr<Scanner> scanner,
	    double accuracy_m,
	    double rangeMin_m,
	    double rangeMax_m=std::numeric_limits<double>::max()
    ){
        this->cfg_device_accuracy_m = accuracy_m;
        this->cfg_device_rangeMin_m = rangeMin_m;
        this->cfg_device_rangeMax_m = rangeMax_m;
        this->scanner   = std::move(scanner);
	}
	virtual ~AbstractDetector() {}
	virtual std::shared_ptr<AbstractDetector> clone() = 0;
	virtual void _clone(std::shared_ptr<AbstractDetector> ad);

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Shutdown the detector when simulation has finished
	 */
	virtual void shutdown();
    /**
     * @brief Apply scanner settings to the detector
     * @param settings Settings to be applied to de detector
     */
    virtual void applySettings(std::shared_ptr<ScannerSettings> & settings) {};
    /**
     * @brief Handle detector behavior when leg has been completed.
     *
     * It mainly forces yielded point cloud to be flushed
     */
    virtual void onLegComplete();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the main facade to file management system
     * @return The main facade to file management system
     */
    inline std::shared_ptr<FMSFacade> getFMS() const {return fms;}
    /**
     * @brief Set the main facade to file management system that will be used
     *  by the detector
     * @param fms The new main facade to file management system for the
     *  detector
     */
    void setFMS(std::shared_ptr<FMSFacade> fms);
};