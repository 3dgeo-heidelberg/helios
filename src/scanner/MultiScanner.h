#pragma once

#include <scanner/Scanner.h>
#include <scanner/ScanningDevice.h>

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a multi scanner. It is, a scanner that can emit
 *  more than one pulse per time instant.
 */
class MultiScanner : public Scanner{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The scanning devices composing the MultiScanner
     */
    std::vector<ScanningDevice> scanDevs;


public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @biref MultiScanner constructor from given scanning devices
     * @param scanDevs The scanning devices composing the MultiScanner
     * @see Scanner::Scanner
     */
    MultiScanner(
        std::vector<ScanningDevice> && scanDevs,
        std::string const id,
        std::list<int> const &pulseFreqs,
        bool const writeWaveform=false,
        bool const calcEchowidth=false,
        bool const fullWaveNoise=false,
        bool const platformNoiseDisabled=false
    ) :
        Scanner(
            id,
            pulseFreqs,
            writeWaveform,
            calcEchowidth,
            fullWaveNoise,
            platformNoiseDisabled
        ),
        scanDevs(std::move(scanDevs))
    {}
    /**
     * @brief MultiScanner default constructor
     * @see Scanner::Scanner
     */
    MultiScanner(
        std::string const id,
        std::list<int> const &pulseFreqs,
        bool const writeWaveform=false,
        bool const calcEchowdith=false,
        bool const fullWaveNoise=false,
        bool const platformNoiseDisabled=false
    ) :
        Scanner(
            id,
            pulseFreqs,
            writeWaveform,
            calcEchowidth,
            fullWaveNoise,
            platformNoiseDisabled
        ),
        scanDevs()
    {}
    /**
     * @brief Copy constructor for the MultiScanner
     * @param scanner The scanner to be copied
     */
    MultiScanner(MultiScanner &scanner);
    virtual ~MultiScanner() = default;


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
    void applySettings(
        std::shared_ptr<ScannerSettings> settings, size_t const idx
    ) override;
    /**
     * @see Scanner::prepareDiscretization
     */
    void prepareDiscretization(size_t const idx) override;
    /**
     * @see Scanner::calcFootprintArea
     */
    double calcFootprintArea(
        double const distance, size_t const idx
    ) const override;
    /**
     * @see Scanner::calcAbsoluteBeamAttitude
     */
    Rotation calcAbsoluteBeamAttitude(size_t const idx) override;
    /**
     * @see Scanner::calcAtmosphericAttenuation
     */
    double calcAtmosphericAttenuation(size_t const idx) const override
    {return scanDevs[idx].calcAtmosphericAttenuation();}


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see Scanner::getDeviceId
     */
    std::string getDeviceId(size_t const idx) const override
    {return scanDevs[idx].id;}
    /**
     * @see Scanner::setDeviceId
     */
    void setDeviceId(std::string const deviceId, size_t const idx) override
    {scanDevs[idx].id = deviceId;}
    /**
     * @see Scanner::getNumDevices
     */
    size_t getNumDevices() const override {return scanDevs.size();}
    /**
     * @see Scanner::getPulseLength_ns
	 */
    double getPulseLength_ns(size_t const idx) const override
    {return scanDevs[idx].pulseLength_ns;}
    /**
     * @see Scanner::setPulseLength_ns
     */
    void setPulseLength_ns(
        double const pulseLength_ns, size_t const idx
    ) override
    {scanDevs[idx].pulseLength_ns = pulseLength_ns;}
    /**
     * @see Scanner::getBeamDivergence
     */
    double getBeamDivergence(size_t const idx) const override
    {return scanDevs[idx].beamDivergence_rad;}
    /**
     * @see Scanner::setBeamDivergence
     */
    void setBeamDivergence(
        double const beamDivergence, size_t const idx
    ) override
    {scanDevs[idx].beamDivergence_rad = beamDivergence;}
    /**
     * @see Scanner::getAveragePower
     */
    double getAveragePower(size_t const idx) const override
    {return scanDevs[idx].averagePower_w;}
    /**
     * @see Scanner::setAveragePower
     */
    void setAveragePower(
        double const averagePower, size_t const idx
    ) override
    {scanDevs[idx].averagePower_w = averagePower;}
    /**
     * @see Scanner::getBeamQuality
     */
    double getBeamQuality(size_t const idx) const override
    {return scanDevs[idx].beamQuality;}
    /**
     * @see Scanner::setBeamQuality
     */
    void setBeamQuality(double const beamQuality, size_t const idx) override
    {scanDevs[idx].beamQuality = beamQuality;}
    /**
     * @see Scanner::getEfficiency
     */
    double getEfficiency(size_t const idx) const override
    {return scanDevs[idx].efficiency;}
    /**
     * @see Scanner::setEfficiency
     */
    void setEfficiency(double const efficiency, size_t const idx) override
    {scanDevs[idx].efficiency = efficiency;}
    /**
     * @see Scanner::getReceiverDiameter
     */
    double getReceiverDiameter(size_t const idx) const override
    {return scanDevs[idx].receiverDiameter_m;}
    /**
     * @see Scanner::setReceiverDiameter
     */
    void setReceiverDiameter(
        double const receiverDiameter, size_t const idx
    ) override
    {scanDevs[idx].receiverDiameter_m = receiverDiameter;}
    /**
     * @see Scanner::getVisibility
     */
    double getVisibility(size_t const idx) const override
    {return scanDevs[idx].visibility_km;}
    /**
     * @see Scanner::setVisibility
     */
    void setVisibility(double const visibility, size_t const idx) override
    {scanDevs[idx].visibility_km = visibility;}
    /**
     * @see Scanner::getWavelength
     */
    double getWavelength(size_t const idx) const override
    {return scanDevs[idx].wavelength_m;}
    /**
     * @see Scanner::setWavelength
     */
    void setWavelength(double const wavelength, size_t const idx) override
    {scanDevs[idx].wavelength_m = wavelength;}
    /**
     * @see Scanner::getAtmosphericExtinction
     */
    double getAtmosphericExtinction(size_t const idx) const override
    {return scanDevs[idx].atmosphericExtinction;}
    /**
     * @see Scanner::setAtmosphericExtinction
     */
    void setAtmosphericExtinction(
        double const atmosphericExtinction,
        size_t const idx
    ) override
    {scanDevs[idx].atmosphericExtinction = atmosphericExtinction;}
    /**
     * @see Scanner::getBeamWaistRadius
     */
    double getBeamWaistRadius(size_t const idx) const override
    {return scanDevs[idx].beamWaistRadius;}
    /**
     * @see Scanner::setBeamWaistRadius
     */
    void setBeamWaistRadius(
        double const beamWaistRadius, size_t const idx
    ) override
    {scanDevs[idx].beamWaistRadius = beamWaistRadius;}
    /**
     * @see Scanner::getHeadRelativeEmitterPosition
     */
    glm::dvec3 getHeadRelativeEmitterPosition(
        size_t const idx
    ) const override
    {return scanDevs[idx].headRelativeEmitterPosition;}
    /**
     * @see Scanner::setHeadRelativeEmitterPosition
     */
    void setHeadRelativeEmitterPosition(
        glm::dvec3 const &pos, size_t const idx
    ) override
    {scanDevs[idx].headRelativeEmitterPosition = pos;}
    /**
     * @see Scanner::getHeadRelativeEmitterPositionByRef
     */
    glm::dvec3 & getHeadRelativeEmitterPositionByRef(
        size_t const idx=0
    ) override
    {return scanDevs[idx].headRelativeEmitterPosition;}
    /**
     * @see Scanner::getHeadRelativeEmitterAttitude
     */
    Rotation getHeadRelativeEmitterAttitude(size_t const idx) const override
    {return scanDevs[idx].headRelativeEmitterAttitude;}
    /**
     * @see Scanner::setHeadRelativeEmitterAttitude
     */
    void setHeadRelativeEmitterAttitude(
        Rotation const &attitude, size_t const idx
    ) override
    {scanDevs[idx].headRelativeEmitterAttitude = attitude;}
    /**
     * @see Scanner::getHeadRelativeEmitterAttitudeByRef
     */
    Rotation & getHeadRelativeEmitterAttitudeByRef(
        size_t const idx=0
    ) override
    {return scanDevs[idx].headRelativeEmitterAttitude;}
    /**
     * @see Scanner::getBt2
     */
    double getBt2(size_t const idx) const override
    {return scanDevs[idx].cached_Bt2;}
    /**
     * @see Scanner::setBt2
     */
    void setBt2(double const bt2, size_t const idx) override
    {scanDevs[idx].cached_Bt2 = bt2;}
    /**
     * @see Scanner::getDr2
     */
    double getDr2(size_t const idx) const override
    {return scanDevs[idx].cached_Dr2;}
    /**
     * @see Scanner::setDr2
     */
    void setDr2(double const dr2, size_t const idx) override
    {scanDevs[idx].cached_Dr2 = dr2;}

};