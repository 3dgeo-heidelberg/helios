#pragma once

#include <scanner/Scanner.h>
#include <scanner/ScanningDevice.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a single scanner. It is, a scanner that can only
 *  emit one pulse per time instant.
 */
class SingleScanner : public Scanner{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The only scanning device composing the SingleScanner
     */
    ScanningDevice scanDev;


public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief SingleScanner constructor from given values
     * @see Scanner::Scanner
     */
    SingleScanner(
        double const beamDiv_rad,
        glm::dvec3 const beamOrigin,
        Rotation const beamOrientation,
        std::list<int> const &pulseFreqs,
        double const pulseLength_ns,
        std::string const id,
        double const averagePower,
        double const beamQuality,
        double const efficiency,
        double const receiverDiameter,
        double const atmosphericVisibility,
        int const wavelength,
        bool const writeWaveform = false,
        bool const calcEchowidth = false,
        bool const fullWaveNoise = false,
        bool const platformNoiseDisabled = false
    );
    /**
     * @brief Copy constructor for the SingleScanner
     * @param scanner The scanner to be copied
     */
    SingleScanner(SingleScanner &scanner);
    virtual ~SingleScanner() = default;

    // ***   C L O N E   *** //
    // ********************* //
    /**
     * @see Scanner::clone
     */
    std::shared_ptr<Scanner> clone() override;
    /**
     * @see Scanner::_clone
     */
    void _clone(Scanner &sc) const override;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @see Scanner::applySettings
     */
    void applySettings(std::shared_ptr<ScannerSettings> settings) override;
    /**
     * @see Scanner::prepareDiscretization
     */
    void prepareDiscretization() override;
    /**
     * @see Scanner::calcTimePropagation
     */
    int calcTimePropagation(
        std::vector<double> & timeWave, int const numBins
    ) const override;
    /**
     * @see Scanner::calcFootprintArea
     */
    double calcFootprintArea(double const distance) const override;
    /**
     * @see Scanner::calcAbsoluteBeamAttitude
     */
    Rotation calcAbsoluteBeamAttitude() const override;
    /**
     * @see Scanner::calcAtmosphericAttenuation
     */
    double calcAtmosphericAttenuation() const override
    {return scanDev.calcAtmosphericAttenuation();}


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see Scanner::getDeviceId
     */
    std::string getDeviceId(size_t const idx) const override
    {return scanDev.id;}
    /**
     * @see Scanner::setDeviceId
     */
    void setDeviceId(std::string const deviceId, size_t const idx) override
    {scanDev.id = deviceId;}
    /**
     * @see Scanner::getNumDevices
     */
    size_t getNumDevices() const override {return 1;}
    /**
     * @see Scanner::getPulseLength_ns
	 */
    double getPulseLength_ns(size_t const idx) const override
    {return scanDev.pulseLength_ns;}
    /**
     * @see Scanner::setPulseLength_ns
     */
    void setPulseLength_ns(
        double const pulseLength_ns, size_t const idx
    ) override
    {scanDev.pulseLength_ns = pulseLength_ns;}
    /**
     * @see Scanner::getBeamDivergence
     */
    double getBeamDivergence(size_t const idx) const override
    {return scanDev.beamDivergence_rad;}
    /**
     * @see Scanner::setBeamDivergence
	 */
    void setBeamDivergence(
        double const beamDivergence, size_t const idx
    ) override
    {scanDev.beamDivergence_rad = beamDivergence;}
    /**
     * @see Scanner::getAveragePower
     */
    double getAveragePower(size_t const idx) const override
    {return scanDev.averagePower_w;}
    /**
     * @see Scanner::setAveragePower
	 */
    void setAveragePower(
        double const averagePower, size_t const idx
    ) override
    {scanDev.averagePower_w = averagePower;}
    /**
     * @see Scanner::getBeamQuality
     */
    double getBeamQuality(size_t const idx) const override
    {return scanDev.beamQuality;}
    /**
     * @see Scanner::setBeamQuality
	 */
    void setBeamQuality(double const beamQuality, size_t const idx) override
    {scanDev.beamQuality = beamQuality;}
    /**
     * @see Scanner::getEfficiency
     */
    double getEfficiency(size_t const idx) const override
    {return scanDev.efficiency;}
    /**
     * @see Scanner::setEfficiency
     */
    void setEfficiency(double const efficiency, size_t const idx) override
    {scanDev.efficiency = efficiency;}
    /**
     * @see Scanner::getReceiverDiameter
     */
    double getReceiverDiameter(size_t const idx) const override
    {return scanDev.receiverDiameter_m;}
    /**
     * @see Scanner::setReceiverDiameter
     */
    void setReceiverDiameter(
        double const receiverDiameter, size_t const idx
    ) override
    {scanDev.receiverDiameter_m = receiverDiameter;}
    /**
     * @see Scanner::getVisibility
     */
    double getVisibility(size_t const idx) const override
    {return scanDev.visibility_km;}
    /**
     * @see Scanner::setVisibility
     */
    void setVisibility(double const visibility, size_t const idx) override
    {scanDev.visibility_km = visibility;}
    /**
     * @see Scanner::getWavelength
     */
    double getWavelength(size_t const idx) const override
    {return scanDev.wavelength_m;}
    /**
     * @see Scanner::setWavelength
     */
    void setWavelength(double const wavelength, size_t const idx) override
    {scanDev.wavelength_m = wavelength;}
    /**
     * @see Scanner::getAtmosphericExtinction
     */
    double getAtmosphericExtinction(size_t const idx) const override
    {return scanDev.atmosphericExtinction;}
    /**
     * @see Scanner::setAtmosphericExtinction
     */
    void setAtmosphericExtinction(
        double const atmosphericExtinction,
        size_t const idx
    ) override
    {scanDev.atmosphericExtinction = atmosphericExtinction;}
    /**
     * @see Scanner::getBeamWaistRadius
     */
    double getBeamWaistRadius(size_t const idx) const override
    {return scanDev.beamWaistRadius;}
    /**
     * @see Scanner::setBeamWaistRadius
     */
    void setBeamWaistRadius(
        double const beamWaistRadius, size_t const idx
    ) override
    {scanDev.beamWaistRadius = beamWaistRadius;}
    /**
     * @see Scanner::getHeadRelativeEmitterPosition
     */
    glm::dvec3 getHeadRelativeEmitterPosition(
        size_t const idx
    ) const override
    {return scanDev.headRelativeEmitterPosition;}
    /**
     * @see Scanner::setHeadRelativeEmitterPosition
     */
    void setHeadRelativeEmitterPosition(
        glm::dvec3 const &pos, size_t const idx
    ) override
    {scanDev.headRelativeEmitterPosition = pos;}
    /**
     * @see Scanner::getHeadRelativeEmitterAttitude
     */
    Rotation getHeadRelativeEmitterAttitude(size_t const idx) const override
    {return scanDev.headRelativeEmitterAttitude;}
    /**
     * @see Scanner::setHeadRelativeEmitterAttitude
     */
    void setHeadRelativeEmitterAttitude(
        Rotation const &attitude, size_t const idx
    ) override
    {scanDev.headRelativeEmitterAttitude = attitude;}
    /**
     * @see Scanner::getBt2
     */
    double getBt2(size_t const idx) const override
    {return scanDev.cached_Bt2;}
    /**
     * @see Scanner::setBt2
     */
    void setBt2(double const bt2, size_t const idx) override
    {scanDev.cached_Bt2 = bt2;}
    /**
     * @see Scanner::getDr2
     */
    double getDr2(size_t const idx) const override
    {return scanDev.cached_Dr2;}
    /**
     * @see Scanner::setDr2
     */
    void setDr2(double const dr2, size_t const idx) override
    {scanDev.cached_Dr2 = dr2;}

#ifdef PYTHON_BINDING
    /**
     * @see Scanner::getRelativeAttitudeByReference
     */
    Rotation & getRelativeAttitudeByReference(
        size_t const idx
    ) override
    {return scanDev.headRelativeEmitterAttitude;}
    /**
     * @see Scanner::getRelativePosition
     */
    PythonDVec3 * getRelativePosition(size_t const idx) override
    {return new PythonDVec3(&(scanDev.headRelativeEmitterPosition));}
#endif
};