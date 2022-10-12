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
     * @see Scanner::doSimStep(unsigned int, double const)
     */
    void doSimStep(
        unsigned int legIndex, double const currentGpsTime
    ) override;
    /**
     * @see Scanner::calcRaysNumber(size_t const)
     */
    void calcRaysNumber(size_t const idx) override
    {scanDevs[idx].calcRaysNumber();}
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
     * @see Scanner::calcTargetArea
     */
    double calcTargetArea(
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
    /**
     * @see Scanner::checkMaxNOR
     */
    bool checkMaxNOR(int const nor, size_t const idx) override
    {return scanDevs[idx].maxNOR==0 || nor < scanDevs[idx].maxNOR;}
    /**
     * @see Scanner::computeSubrays
     */
    void computeSubrays(
        std::function<void(
            vector<double> const &_tMinMax,
            int const circleStep,
            double const circleStep_rad,
            Rotation &r1,
            double const divergenceAngle,
            NoiseSource<double> &intersectionHandlingNoiseSource,
            std::map<double, double> &reflections,
            vector<RaySceneIntersection> &intersects
        )> handleSubray,
        vector<double> const &tMinMax,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects,
        size_t const idx
    ) override;
    /**
     * @see Scanner::initializeFullWaveform
     */
    bool initializeFullWaveform(
        double const minHitDist_m,
        double const maxHitDist_m,
        double &minHitTime_ns,
        double &maxHitTime_ns,
        double &nsPerBin,
        double &distanceThreshold,
        int &peakIntensityIndex,
        int &numFullwaveBins,
        size_t const idx
    ) override;
    /**
     * @see Scanner::calcIntensity
     */
    double calcIntensity(
        double const incidenceAngle,
        double const targetRange,
        double const targetReflectivity,
        double const targetSpecularity,
        double const targetSpecularExponent,
        double const targetArea,
        double const radius,
        size_t const idx
    ) const override;
    /**
     * @see Scanner::calcIntensity
     */
    double calcIntensity(
        double const targetRange,
        double const radius,
        double const sigma,
        size_t const idx
    ) const override;


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see Scanner::setDeviceIndex
     */
    void setDeviceIndex(size_t const newIdx, size_t const oldIdx) override
    {scanDevs[oldIdx].devIdx = newIdx;}
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
    /**
     * @see Scanner::getCurrentPulseNumber(size_t const)
     */
    int getCurrentPulseNumber(size_t const idx) const override
    {return scanDevs[idx].state_currentPulseNumber;}
    /**
     * @see Scanner::getNumRays
     */
    int getNumRays(size_t const idx) const override
    {return scanDevs[idx].numRays;}
    /**
     * @see Scanner::setNumRays
     */
    void setNumRays(int const numRays, size_t const idx)
    {scanDevs[idx].numRays = numRays;}
    /**
     * @see Scanner::lastPulseWasHit(size_t const)
     */
    bool lastPulseWasHit(size_t const idx) const override
    {return scanDevs[idx].lastPulseWasHit();}
    /**
     * @see Scanner::setLastPulseWasHit(bool const, size_t const)
     */
    void setLastPulseWasHit(
        bool const lastPulseWasHit, size_t const idx
    ) override
    {scanDevs[idx].setLastPulseWasHit(lastPulseWasHit);}
    /**
     * @see Scanner::getScannerHead(size_t const)
     */
    std::shared_ptr<ScannerHead> getScannerHead(size_t const idx) override
    {return scanDevs[idx].scannerHead;}
    /**
     * @see Scanner::setScannerHead(shared_ptr<ScannerHead>, size_t const)
     */
    void setScannerHead(
        std::shared_ptr<ScannerHead> scannerHead,
        size_t const idx
    ) override
    {scanDevs[idx].scannerHead = scannerHead;}
    /**
     * @see Scanner::getBeamDeflector(size_t const)
     */
    std::shared_ptr<AbstractBeamDeflector> getBeamDeflector(
        size_t const idx
    ) override
    {return scanDevs[idx].beamDeflector;}
    /**
     * @see Scanner::setBeamDeflector
     */
    void setBeamDeflector(
        std::shared_ptr<AbstractBeamDeflector> beamDeflector,
        size_t const idx
    ) override
    {scanDevs[idx].beamDeflector = beamDeflector;}
    /**
     * @see Scanner::getDetector(size_t const)
     */
    std::shared_ptr<AbstractDetector> getDetector(size_t const idx) override
    {return scanDevs[idx].detector;}
    /**
     * @see Scanner::setDetector(shared_ptr<AbstractDetector>, size_t const)
     */
    void setDetector(
        std::shared_ptr<AbstractDetector> detector, size_t const idx
    ) override
    {scanDevs[idx].detector = detector;}
    /**
     * @see Scanner::getFWFSettings
     */
    FWFSettings & getFWFSettings(size_t const idx)
    {return scanDevs[idx].FWF_settings;}
    /**
     * @see Scanner::setFWFSettings(FWFSettings const &, size_t const)
     */
    void setFWFSettings(
        FWFSettings const &fwfSettings, size_t const idx
    ) override
    {scanDevs[idx].FWF_settings = fwfSettings;}
    /**
     * @see Scanner::getSupportedPulseFreqs_Hz(size_t const)
     */
    std::list<int>& getSupportedPulseFreqs_Hz(size_t const idx) override
    {return scanDevs[idx].supportedPulseFreqs_Hz;}
    /**
     * @see Scanner::setSupportedPulseFreqs_Hz(std::list<int> &, size_t const)
     */
    void setSupportedPulseFreqs_Hz(
        std::list<int> &pulseFreqs_Hz, size_t const idx
    ){
        scanDevs[idx].supportedPulseFreqs_Hz = pulseFreqs_Hz;
    }
    /**
     * @see Scanner::getMaxNOR(size_t const)
     */
    int getMaxNOR(size_t const idx) const override
    {return scanDevs[idx].maxNOR;}
    /**
     * @see Scanner::setMaxNOR(int const, size_t const)
     */
    void setMaxNOR(int const maxNOR, size_t const idx) override
    {scanDevs[idx].maxNOR = maxNOR;}
    /**
     * @see Scanner::getNumTimeBins(size_t const)
     */
    int getNumTimeBins(size_t const idx) const
    {return scanDevs[idx].numTimeBins;}
    /**
     * @see Scanner::setNumTimeBins(int const, size_t const)
     */
    void setNumTimeBins(int const numTimeBins, size_t const idx)
    {scanDevs[idx].numTimeBins = numTimeBins;}
    /**
     * @see Scanner::getPeakIntensityIndex(size_t const)
     */
    int getPeakIntensityIndex(size_t const idx) const override
    {return scanDevs[idx].peakIntensityIndex;}
    /**
     * @see Scanner::setPeakIntensityIndex(int const, size_t const)
     */
    void setPeakIntensityIndex(int const pii, size_t const idx) override
    {scanDevs[idx].peakIntensityIndex = pii;}
    /**
     * @see Scanner::getTimeWave(size_t const)
     */
    std::vector<double>& getTimeWave(size_t const idx) override
    {return scanDevs[idx].time_wave;}
    /**
     * @see Scanner::setTimeWave(std::vector<double> &, size_t const)
     */
    void setTimeWave(std::vector<double> &timewave, size_t const idx)
    {scanDevs[idx].time_wave = timewave;}
    /**
     * @see Scanner::setTimeWave(std::vector<double> &&, size_t const)
     */
    void setTimeWave(
        std::vector<double> &&timewave, size_t const idx
    ) override
    {scanDevs[idx].time_wave = timewave;}

};