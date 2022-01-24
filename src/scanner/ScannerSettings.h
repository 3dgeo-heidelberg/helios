#pragma once

#include "Asset.h"

#include <string>
#include <memory>
#include <sstream>
#include <ostream>
#include <unordered_set>
#include <functional>

/**
 * @brief Scanner settings class
 */
class ScannerSettings : public Asset {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The ID for this scanner settings.
     * It does not make sense for all scanner settings, but it is specially
     * useful when it comes to handling XML templates
     */
    std::string id = "#nullid#";
    /**
     * @brief Template defining default values which were used to build the
     *  ScannerSettings object
     */
    std::shared_ptr<ScannerSettings> baseTemplate = nullptr;

    /**
     * @brief Flag to specify if scanner is active (true) or not (false)
     *
     * When a scanner is not active, no points will be captured
     */
	bool active = true;
	/**
	 * @brief Amount of rotation (radians) per second for the scanner head
	 */
	double headRotatePerSec_rad = 0;
	/**
	 * @brief Starting angle (radians) for the scanner head
	 */
	double headRotateStart_rad = 0;
	/**
	 * @brief Ending angle (radians) for the scanner head
	 */
	double headRotateStop_rad = 0;
	/**
	 * @brief Pulse frequency (hertz)
	 */
	int pulseFreq_Hz = 0;
	/**
	 * @brief Scan angle (radians)
	 */
	double scanAngle_rad = 0;
	/**
	 * @brief Minimum vertical angle (radians)
	 */
    double verticalAngleMin_rad = 0;
    /**
     * @brief Maximum vertical angle (radians)
     */
    double verticalAngleMax_rad = 0;
    /**
     * @brief Scanning frequency (hertz)
     */
	double scanFreq_Hz = 0;
	/**
	 * @brief Beam divergence angle (radians)
	 */
	double beamDivAngle = 0.003;
	/**
	 * @brief Time interval between trajectory recollections (seconds)
	 */
	double trajectoryTimeInterval = 0.0; // In seconds

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Scanner settings default constructor
	 */
	ScannerSettings() = default;

    /**
     * @brief Copy from pointer constructor
     * @param other Scanner settings to be copied
     */
	ScannerSettings(ScannerSettings* other) {
		if (other == nullptr) return;

		this->id = other->id;
		this->baseTemplate = other->baseTemplate;
		this->active = other->active;
		this->headRotatePerSec_rad = other->headRotatePerSec_rad;
		this->headRotateStart_rad = other->headRotateStart_rad;
		this->headRotateStop_rad = other->headRotateStop_rad;
		this->pulseFreq_Hz = other->pulseFreq_Hz;
		this->scanAngle_rad = other->scanAngle_rad;
        this->verticalAngleMin_rad = other->verticalAngleMin_rad;
        this->verticalAngleMax_rad = other->verticalAngleMax_rad;
		this->scanFreq_Hz = other->scanFreq_Hz;
		this->beamDivAngle = other->beamDivAngle;
		this->trajectoryTimeInterval = other->trajectoryTimeInterval;
	}

	// ***  CHERRY PICKING  *** //
	// ************************ //
	/**
	 * @brief Build a new scanner settings which by default has the same values
	 *  than caller scanner settings (this). Any field specified through fields
	 *  set will be overloaded from cherries scanner settings.
	 * @param cherries From where overloaded values are taken
	 * @param[in] fields Which fields must be overloaded
	 * @param[in] templateFields Which fields must be overloaded for the
	 *  template. Notice it can be nullptr in case there is no cherry template
	 * @return New scanner settings from cherry picking
	 */
	std::shared_ptr<ScannerSettings> cherryPick(
	    std::shared_ptr<ScannerSettings> cherries,
	    std::unordered_set<std::string> const &fields,
	    std::unordered_set<std::string> const *templateFields=nullptr
    ){
	    // Prepare for cherry picking
        std::shared_ptr<ScannerSettings> settings = \
            std::make_shared<ScannerSettings>(this);
        std::function<bool(std::string const&)> hasCherry = [&] (
            std::string const &fieldName
        ) -> bool {
            return fields.find(fieldName) != fields.end();
        };
        // The cherry picking itself
        if(hasCherry("baseTemplate") && cherries->baseTemplate != nullptr){
            std::shared_ptr<ScannerSettings> tmpTemplate = \
                cherries->baseTemplate->baseTemplate;
            cherries->baseTemplate->baseTemplate = nullptr;
            settings = cherryPick(cherries->baseTemplate, *templateFields);
            cherries->baseTemplate->baseTemplate = tmpTemplate;
            settings->baseTemplate = cherries->baseTemplate;
        }
        if(hasCherry("active")) settings->active = cherries->active;
        if(hasCherry("headRotatePerSec_rad"))
            settings->headRotatePerSec_rad = cherries->headRotatePerSec_rad;
        if(hasCherry("headRotateStart_rad"))
            settings->headRotateStart_rad = cherries->headRotateStart_rad;
        if(hasCherry("headRotateStop_rad"))
            settings->headRotateStop_rad = cherries->headRotateStop_rad;
        if(hasCherry("pulseFreq_Hz"))
            settings->pulseFreq_Hz = cherries->pulseFreq_Hz;
        if(hasCherry("scanAngle_rad"))
            settings->scanAngle_rad = cherries->scanAngle_rad;
        if(hasCherry("verticalAngleMin_rad"))
            settings->verticalAngleMin_rad = cherries->verticalAngleMin_rad;
        if(hasCherry("verticalAngleMax_rad"))
            settings->verticalAngleMax_rad = cherries->verticalAngleMax_rad;
        if(hasCherry("scanFreq_Hz"))
            settings->scanFreq_Hz = cherries->scanFreq_Hz;
        if(hasCherry("beamDivAngle"))
            settings->beamDivAngle = cherries->beamDivAngle;
        if(hasCherry("trajectoryTimeInterval")){
            settings->trajectoryTimeInterval=cherries->trajectoryTimeInterval;
        }
        // Return settings from cherry picking
        return settings;
	}

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	/**
	 * @brief Check if this ScannerSettings has an associated template (true)
	 *  or not (false)
	 * @return True when there is an associated template, false otherwise
	 * @see ScannerSettings::baseTemplate
	 * @see ScannerSettings::hasTemplate
	 */
	bool hasTemplate() {return this->baseTemplate != nullptr;}
	/**
	 * @brief Obtain template by reference
	 * @return Reference to template associated to this ScannerSettings
	 * @see ScannerSettings::baseTemplate
	 * @see ScannerSettings::hasTemplate
	 */
	ScannerSettings & getTemplate() {return *this->baseTemplate;}

	// ***  TO STRING  *** //
	// ******************* //
	/**
	 * @brief Obtain the string representation of the scanner settings
	 * @return String representing the scanner settings
	 */
	virtual inline std::string toString() const{
        std::stringstream ss;
        ss  << "ScannerSettings \"" << id << "\":\n";
        if(baseTemplate != nullptr){
            ss  << "\ttemplate.id = \"" << baseTemplate->id << "\"\n"
                << "\ttemplate.active = " << baseTemplate->active << "\n"
                << "\ttemplate.headRotatePerSec_rad = "
                    << baseTemplate->headRotatePerSec_rad << "\n"
                << "\ttemplate.headRotateStart_rad = "
                    << baseTemplate->headRotateStart_rad << "\n"
                << "\ttemplate.headRotateStop_rad = "
                    << baseTemplate->headRotateStop_rad << "\n"
                << "\ttemplate.pulseFreq_Hz = "
                    << baseTemplate->pulseFreq_Hz << "\n"
                << "\ttemplate.scanAngle_rad = "
                    << baseTemplate->scanAngle_rad << "\n"
                << "\ttemplate.verticalAngleMin_rad = "
                    << baseTemplate->verticalAngleMin_rad << "\n"
                << "\ttemplate.verticalAngleMax_rad = "
                    << baseTemplate->verticalAngleMax_rad << "\n"
                << "\ttemplate.scanFreq_Hz = "
                    << baseTemplate->scanFreq_Hz << "\n"
                << "\ttemplate.beamDivAngle = "
                    << baseTemplate->beamDivAngle << "\n"
                << "\ttemplate.trajectoryTimeInterval = "
                    << baseTemplate->trajectoryTimeInterval << "\n"
            ;
        }
        ss  << "active = " << active << "\n"
            << "headRotatePerSec_rad = " << headRotatePerSec_rad << "\n"
            << "headRotateStart_rad = " << headRotateStart_rad << "\n"
            << "headRotateStop_rad = " << headRotateStop_rad << "\n"
            << "pulseFreq_Hz = " << pulseFreq_Hz << "\n"
            << "scanAngle_rad = " << scanAngle_rad << "\n"
            << "verticalAngleMin_rad = " << verticalAngleMin_rad << "\n"
            << "verticalAngleMax_rad = " << verticalAngleMax_rad << "\n"
            << "scanFreq_Hz = " << scanFreq_Hz << "\n"
            << "beamDivAngle = " << beamDivAngle << "\n"
            << "trajectoryTimeInterval = " << trajectoryTimeInterval << "\n"
        ;
        return ss.str();
	}
	/**
	 * @brief Overload of << operator for output streams
	 */
	friend std::ostream& operator<< (
	    std::ostream &out,
	    const ScannerSettings &settings
    ){
	    out << settings.toString();
	    return out;
	}
};