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
     * @see ScanningDevice::ScanningDevice
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
        std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr = nullptr,
        bool const writeWaveform = false,
        bool const writePulse = false,
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

    // ***  SIM STEP UTILS  *** //
    // ************************ //
    /**
     * @brief Extend Scanner::onLegComplete behavior by handling onLegComplete
     *  events for the case of a single detector
     * @see Scanner::onLegComplete
     */
    void onLegComplete() override;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @see Scanner::prepareSimulation
     */
    void prepareSimulation() override;
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
    {scanDev.calcRaysNumber();}
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
    {return scanDev.calcAtmosphericAttenuation();}
    /**
     * @see Scanner::checkMaxNOR
     */
    bool checkMaxNOR(int const nor, size_t const idx) override
    {return scanDev.maxNOR==0 || nor < scanDev.maxNOR;}
    /**
     * @see Scanner::computeSubrays
     */
    void computeSubrays(
        std::function<void(
            Rotation const &subrayRotation,
            double const divergenceAngle,
            NoiseSource<double> &intersectionHandlingNoiseSource,
            std::map<double, double> &reflections,
            vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >= 2
           ,bool &subrayHit,
            std::vector<double> &subraySimRecord
#endif
        )> handleSubray,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects,
        size_t const idx
#if DATA_ANALYTICS >= 2
       ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
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
        Material const &mat,
        double const targetArea,
        double const radius,
        size_t const idx
#if DATA_ANALYTICS >= 2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
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
    {scanDev.devIdx = newIdx;}
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
     * @see Scanner::getNumRays
     */
    int getNumRays(size_t const idx) const override {return scanDev.numRays;}
    /**
     * @see Scanner::setNumRays
     */
    void setNumRays(int const numRays, size_t const idx)
    {scanDev.numRays = numRays;}
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
     * @see Scanner::getHeadRelativeEmitterPositionByRef
     */
    glm::dvec3 & getHeadRelativeEmitterPositionByRef(
        size_t const idx=0
    ) override
    {return scanDev.headRelativeEmitterPosition;}
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
     * @see Scanner::getHeadRelativeEmitterAttitudeByRef
     */
    Rotation & getHeadRelativeEmitterAttitudeByRef(
        size_t const idx=0
    ) override
    {return scanDev.headRelativeEmitterAttitude;}
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

    /**
     * @see Scanner::getScannerHead(size_t const)
     */
    std::shared_ptr<ScannerHead> getScannerHead(size_t const idx) override
    {return scanDev.scannerHead;}
    /**
     * @see Scanner::setScannerHead(shared_ptr<ScannerHead>, size_t const)
     */
    void setScannerHead(
        std::shared_ptr<ScannerHead> scannerHead,
        size_t const idx
    ) override
    {scanDev.scannerHead = scannerHead;}
    /**
     * @see Scanner::getBeamDeflector(size_t const)
     */
    std::shared_ptr<AbstractBeamDeflector> getBeamDeflector(
        size_t const idx
    ) override
    {return scanDev.beamDeflector;}
    /**
     * @see Scanner::setBeamDeflector
     */
    void setBeamDeflector(
        std::shared_ptr<AbstractBeamDeflector> beamDeflector,
        size_t const idx
    ) override
    {scanDev.beamDeflector = beamDeflector;}
    /**
     * @see Scanner::getDetector(size_t const)
     */
    std::shared_ptr<AbstractDetector> getDetector(size_t const idx) override
    {return scanDev.detector;}
    /**
     * @see Scanner::setDetector(shared_ptr<AbstractDetector>, size_t const)
     */
    void setDetector(
        std::shared_ptr<AbstractDetector> detector, size_t const idx
    ) override
    {scanDev.detector = detector;}
    /**
     * @see Scanner::getFWFSettings
     */
    FWFSettings & getFWFSettings(size_t const idx)
    {return scanDev.FWF_settings;}
    /**
     * @see Scanner::setFWFSettings(FWFSettings const &, size_t const)
     */
    void setFWFSettings(
        FWFSettings const &fwfSettings, size_t const idx
    ) override
    {scanDev.FWF_settings = fwfSettings;}
    /**
     * @see Scanner::getSupportedPulseFreqs_Hz(size_t const)
     */
    std::list<int>& getSupportedPulseFreqs_Hz(size_t const idx) override
    {return scanDev.supportedPulseFreqs_Hz;}
    /**
     * @see Scanner::setSupportedPulseFreqs_Hz(std::list<int> &, size_t const)
     */
    void setSupportedPulseFreqs_Hz(
        std::list<int> &pulseFreqs_Hz, size_t const idx
    ){
        scanDev.supportedPulseFreqs_Hz = pulseFreqs_Hz;
    }
    /**
     * @see Scanner::getMaxNOR(size_t const)
     */
    int getMaxNOR(size_t const idx) const override {return scanDev.maxNOR;}
    /**
     * @see Scanner::setMaxNOR(int const, size_t const)
     */
    void setMaxNOR(int const maxNOR, size_t const idx) override
    {scanDev.maxNOR = maxNOR;}
    /**
     * @see Scanner::getNumTimeBins(size_t const)
     */
    int getNumTimeBins(size_t const idx) const {return scanDev.numTimeBins;}
    /**
     * @see Scanner::setNumTimeBins(int const, size_t const)
     */
    void setNumTimeBins(int const numTimeBins, size_t const idx)
    {scanDev.numTimeBins = numTimeBins;}
    /**
     * @see Scanner::getPeakIntensityIndex(size_t const)
     */
    int getPeakIntensityIndex(size_t const idx) const override
    {return scanDev.peakIntensityIndex;}
    /**
     * @see Scanner::setPeakIntensityIndex(int const, size_t const)
     */
    void setPeakIntensityIndex(int const pii, size_t const idx) override
    {scanDev.peakIntensityIndex = pii;}
    /**
     * @see Scanner::getTimeWave(size_t const)
     */
    std::vector<double>& getTimeWave(size_t const idx) override
    {return scanDev.time_wave;}
    /**
     * @see Scanner::setTimeWave(std::vector<double> &, size_t const)
     */
    void setTimeWave(std::vector<double> &timewave, size_t const idx)
    {scanDev.time_wave = timewave;}
    /**
     * @see Scanner::setTimeWave(std::vector<double> &&, size_t const)
     */
    void setTimeWave(
        std::vector<double> &&timewave, size_t const idx
    ) override
    {scanDev.time_wave = timewave;}
    /**
     * @see Scanner::getCurrentPulseNumber(size_t const)
     */
    int getCurrentPulseNumber(size_t const idx) const override
    {return scanDev.state_currentPulseNumber;}
    /**
     * @see Scanner::lastPulseWasHit(size_t const)
     */
    bool lastPulseWasHit(size_t const idx) const override
    {return scanDev.lastPulseWasHit();}
    /**
     * @see Scanner::setLastPulseWasHit(bool const, size_t const)
     */
    void setLastPulseWasHit(
        bool const lastPulseWasHit, size_t const idx
    ) override
    {scanDev.setLastPulseWasHit(lastPulseWasHit);}



};