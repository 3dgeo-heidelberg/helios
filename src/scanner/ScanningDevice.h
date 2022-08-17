#pragma once

#include <maths/Rotation.h>
#include <maths/Directions.h>
#include <assetloading/Asset.h>

#include <glm/glm.hpp>

#include <string>

// TODO Rethink : Check all diamond friend classes are properly declared
class Scanner;
class SingleScanner;


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a scanning device. Any scanner needs to be
 *  composed of at least one scanning device to be computable.
 */
class ScanningDevice : public Asset {
protected:
    // ***  F R I E N D S  *** //
    // *********************** //
    friend class Scanner;
    friend class SingleScanner;

    // ***  DEVICE ATTRIBUTES  *** //
    // *************************** //
    /**
	 * @brief Device identifier
	 */
    std::string id = "";
    /**
     * @brief Head relative emitter position
     */
    glm::dvec3 headRelativeEmitterPosition = glm::dvec3(0, 0, 0);
    /**
	 * @brief Head relative emitter attitude
	 */
    Rotation headRelativeEmitterAttitude =
        Rotation(Directions::right, 0);
    /**
     * @brief Beam divergence (radians)
     */
    double beamDivergence_rad = 0;
    /**
	 * @brief Pulse length (nanoseconds)
	 */
    double pulseLength_ns = 0;
    /**
	 * @brief Average power (watts)
	 */
    double averagePower_w;
    /**
	 * @brief Beam quality
	 */
    double beamQuality;
    /**
	 * @brief Device efficiency
	 */
    double efficiency;
    /**
	 * @brief Receiver diameter (meters)
	 */
    double receiverDiameter_m;
    /**
	 * @brief Visibility (kilometers)
	 */
    double visibility_km;
    /**
	 * @brief Wave length (meters)
	 */
    double wavelength_m;
    /**
	 * @brief Atmospheric extinction
	 */
    double atmosphericExtinction;
    /**
	 * @brief Beam waist radius
	 */
    double beamWaistRadius;

    // ***  CACHED ATTRIBUTES  *** //
    // *************************** //
    /**
	 * @brief \f$D_{r2}\f$ understood as the square of receiver diameter
	 *
	 * \f[
	 *  D_{r2} = \textrm{receiverDiamater}^{2}
	 * \f]
	 *
	 * @see ScanningDevice::receiverDiameter_m
	 */
    double cached_Dr2;
    /**
	 * @brief \f$B_{t2}\f$ understood as the square of beam divergence
	 *
	 * \f[
	 *  B_{t2} = \textrm{beamDivergence}^{2}
	 * \f]
	 *
	 * @see ScanningDevice::beamDivergence_rad
	 */
    double cached_Bt2;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ScanningDevice constructor from given values
     */
    ScanningDevice(
        std::string const id,
        double const beamDiv_rad,
        glm::dvec3 const beamOrigin,
        Rotation const beamOrientation,
        double const pulseLength_ns,
        double const averagePower,
        double const bemQuality,
        double const efficiency,
        double const receiverDiameter_m,
        double const atmosphericVisibility_km,
        double const wavelength_m
    );
    /**
     * @brief Copy constructor for the ScanningDevice
     * @param scdev The scanning device to be copied
     */
    ScanningDevice(ScanningDevice &scdev);
    virtual ~ScanningDevice() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Configure beam related attributes. It is recommended to
     *  reconfigure beam attributes always that beam divergence, beam quality
     *  or wavelength are updated.
     * @see ScanningDevice::beamDivergence_rad
     * @see ScanningDevice::beamQuality
     * @see ScanningDevice::wavelength_m
     * @see ScanningDevice::beamWaistRadius
     * @see ScanningDevice::cached_Bt2
     */
    void configureBeam();
    /**
     * @brief Compute the atmospheric attenuation to be used as the
     *  atmospheric extinction
     * @return Atmospheric attenuation
     * @see Scanner::atmosphericExtinction
     */
    double calcAtmosphericAttenuation() const;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //

};