#include <scanner/Scanner.h>
#include <pybind11/pybind11.h>

class ScannerWrap : public Scanner {
public:
    using Scanner::Scanner;  // Inherit constructors

    ScannerWrap() : Scanner("", std::list<int>()) {}

    ScannerWrap(
        std::string const& id,
        std::list<int> const& pulseFreqs,
        bool writeWaveform=false,
        bool writePulse=false,
        bool calcEchowidth=false,
        bool fullWaveNoise=false,
        bool platformNoiseDisabled=false
    ) : Scanner(id, pulseFreqs, writeWaveform, writePulse, calcEchowidth, fullWaveNoise, platformNoiseDisabled) {}

    ScannerWrap(Scanner& scanner) : Scanner(scanner) {}

    std::shared_ptr<Scanner> clone() override {
        PYBIND11_OVERLOAD_PURE(
            std::shared_ptr<Scanner>,  
            Scanner,                   
            clone                      
        );
    }

    void prepareSimulation(bool const legacyEnergyModel) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            prepareSimulation,         
            legacyEnergyModel          
        );
    }

    std::shared_ptr<ScannerSettings> retrieveCurrentSettings(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::shared_ptr<ScannerSettings>, 
            Scanner,                         
            retrieveCurrentSettings,         
            idx                              
        );
    }
    

    void applySettings(std::shared_ptr<ScannerSettings> settings, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            applySettings,             
            settings, idx              
        );
    }

    void applySettingsFWF(FWFSettings fwfSettings, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            applySettingsFWF,          
            fwfSettings, idx           
        );
    }

    void doSimStep(unsigned int legIndex, double const currentGpsTime) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            doSimStep,                 
            legIndex, currentGpsTime   
        );
    }

    void calcRaysNumber(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            calcRaysNumber,            
            idx                        
        );
    }

    void prepareDiscretization(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            prepareDiscretization,     
            idx                        
        );
    }

    double calcAtmosphericAttenuation(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            calcAtmosphericAttenuation,
            idx                        
        );
    }

    Rotation calcAbsoluteBeamAttitude(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            Rotation,                  
            Scanner,                   
            calcAbsoluteBeamAttitude,  
            idx                        
        );
    }

    bool checkMaxNOR(int const nor, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            bool,                      
            Scanner,                   
            checkMaxNOR,               
            nor, idx                   
        );
    }

    void computeSubrays(
        std::function<void(
            Rotation const&,
            int const,
            NoiseSource<double>&,
            std::map<double, double>&,
            std::vector<RaySceneIntersection>&
#if DATA_ANALYTICS >= 2
            ,bool&,
            std::vector<double>&
#endif
        )> handleSubray,
        NoiseSource<double>& intersectionHandlingNoiseSource,
        std::map<double, double>& reflections,
        std::vector<RaySceneIntersection>& intersects,
        size_t const idx
#if DATA_ANALYTICS >= 2
        ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
    ) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            computeSubrays,            
            handleSubray, intersectionHandlingNoiseSource, reflections, intersects, idx
#if DATA_ANALYTICS >= 2
            ,pulseRecorder
#endif
        );
    }

    bool initializeFullWaveform(
        double const minHitDist_m,
        double const maxHitDist_m,
        double& minHitTime_ns,
        double& maxHitTime_ns,
        double& nsPerBin,
        double& distanceThreshold,
        int& peakIntensityIndex,
        int& numFullwaveBins,
        size_t const idx
    ) override {
        PYBIND11_OVERLOAD_PURE(
            bool,                      
            Scanner,                   
            initializeFullWaveform,    
            minHitDist_m, maxHitDist_m, minHitTime_ns, maxHitTime_ns, nsPerBin,
            distanceThreshold, peakIntensityIndex, numFullwaveBins, idx  
        );
    }

    double calcIntensity(
        double const incidenceAngle,
        double const targetRange,
        Material const& mat,
        int const subrayRadiusStep,
        size_t const idx
#if DATA_ANALYTICS >= 2
        ,std::vector<std::vector<double>>& calcIntensityRecords
#endif
    ) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            calcIntensity,             
            incidenceAngle, targetRange, mat, subrayRadiusStep, idx
#if DATA_ANALYTICS >= 2
            ,calcIntensityRecords
#endif
        );
    }

    double calcIntensity(
        double const targetRange,
        double const sigma,
        int const subrayRadiusStep,
        size_t const idx
    ) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            calcIntensity,             
            targetRange, sigma, subrayRadiusStep, idx  
        );
    }

    void onLegComplete() override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            onLegComplete,             
        );
    }

    ScanningDevice& getScanningDevice(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            ScanningDevice&,           
            Scanner,                   
            getScanningDevice,         
            idx                        
        );
    }

    int getCurrentPulseNumber(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            int,                       
            Scanner,                   
            getCurrentPulseNumber,     
            idx                        
        );
    }

    int getNumRays(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            int,                       
            Scanner,                   
            getNumRays,                
            idx                        
        );
    }

    void setNumRays(int const numRays, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setNumRays,                
            numRays, idx               
        );
    }

    double getPulseLength_ns(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getPulseLength_ns,         
            idx                        
        );
    }

    void setPulseLength_ns(double const pulseLength_ns, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setPulseLength_ns,         
            pulseLength_ns, idx        
        );
    }

    bool lastPulseWasHit(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            bool,                      
            Scanner,                   
            lastPulseWasHit,           
            idx                        
        );
    }

    void setLastPulseWasHit(bool const lastPulseWasHit, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setLastPulseWasHit,        
            lastPulseWasHit, idx      
        );
    }

    double getBeamDivergence(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getBeamDivergence,         
            idx                        
        );
    }

    void setBeamDivergence(double const beamDivergence, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setBeamDivergence,         
            beamDivergence, idx       
        );
    }

    double getAveragePower(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getAveragePower,           
            idx                        
        );
    }

    void setAveragePower(double const averagePower, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setAveragePower,           
            averagePower, idx         
        );
    }

    double getBeamQuality(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getBeamQuality,            
            idx                        
        );
    }

    void setBeamQuality(double const beamQuality, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setBeamQuality,            
            beamQuality, idx          
        );
    }

    double getEfficiency(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getEfficiency,             
            idx                        
        );
    }

    void setEfficiency(double const efficiency, size_t const idx = 0) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setEfficiency,             
            efficiency, idx           
        );
    }

    double getReceiverDiameter(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getReceiverDiameter,       
            idx                        
        );
    }

    void setReceiverDiameter(double const receiverDiameter, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setReceiverDiameter,       
            receiverDiameter, idx     
        );
    }

    double getVisibility(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getVisibility,             
            idx                        
        );
    }

    void setVisibility(double const visibility, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setVisibility,             
            visibility, idx           
        );
    }

    double getWavelength(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getWavelength,             
            idx                        
        );
    }

    void setWavelength(double const wavelength, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setWavelength,             
            wavelength, idx          
        );
    }

    double getAtmosphericExtinction(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getAtmosphericExtinction,  
            idx                        
        );
    }

    void setAtmosphericExtinction(double const atmosphericExtinction, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setAtmosphericExtinction,  
            atmosphericExtinction, idx
        );
    }

    double getBeamWaistRadius(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getBeamWaistRadius,        
            idx                        
        );
    }

    void setBeamWaistRadius(double const beamWaistRadius, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setBeamWaistRadius,        
            beamWaistRadius, idx      
        );
    }

    glm::dvec3 getHeadRelativeEmitterPosition(size_t const idx = 0) const override {
        PYBIND11_OVERLOAD_PURE(
            glm::dvec3,                
            Scanner,                   
            getHeadRelativeEmitterPosition, 
            idx                        
        );
    }

    void setHeadRelativeEmitterPosition(glm::dvec3 const &pos, size_t const idx = 0) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setHeadRelativeEmitterPosition, 
            pos, idx                  
        );
    }

    glm::dvec3 &getHeadRelativeEmitterPositionByRef(size_t const idx = 0) override {
        PYBIND11_OVERLOAD_PURE(
            glm::dvec3 &,              
            Scanner,                   
            getHeadRelativeEmitterPositionByRef, 
            idx                        
        );
    }

    Rotation getHeadRelativeEmitterAttitude(size_t const idx = 0) const override {
        PYBIND11_OVERLOAD_PURE(
            Rotation,                  
            Scanner,                   
            getHeadRelativeEmitterAttitude, 
            idx                        
        );
    }

    void setHeadRelativeEmitterAttitude(Rotation const &attitude, size_t const idx = 0) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setHeadRelativeEmitterAttitude, 
            attitude, idx            
        );
    }

    Rotation &getHeadRelativeEmitterAttitudeByRef(size_t const idx = 0) override {
        PYBIND11_OVERLOAD_PURE(
            Rotation &,                
            Scanner,                   
            getHeadRelativeEmitterAttitudeByRef, 
            idx                        
        );
    }

    double getBt2(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getBt2,                    
            idx                        
        );
    }

    void setBt2(double const bt2, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setBt2,                    
            bt2, idx                 
        );
    }

    double getDr2(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                    
            Scanner,                   
            getDr2,                    
            idx                        
        );
    }

    void setDr2(double const dr2, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setDr2,                    
            dr2, idx                 
        );
    }

    void setDeviceIndex(size_t const newIdx, size_t const oldIdx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setDeviceIndex,            
            newIdx, oldIdx           
        );
    }

    std::string getDeviceId(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            std::string,               
            Scanner,                   
            getDeviceId,               
            idx                        
        );
    }

    void setDeviceId(std::string const deviceId, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                      
            Scanner,                   
            setDeviceId,               
            deviceId, idx            
        );
    }

    size_t getNumDevices() const override {
        PYBIND11_OVERLOAD_PURE(
            size_t,                    
            Scanner,                   
            getNumDevices,             
        );
    }

    std::shared_ptr<ScannerHead> getScannerHead(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::shared_ptr<ScannerHead>, 
            Scanner,                      
            getScannerHead,               
            idx                           
        );
    }

    void setScannerHead(std::shared_ptr<ScannerHead> scannerHead, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                         
            Scanner,                      
            setScannerHead,               
            scannerHead, idx            
        );
    }

    std::shared_ptr<AbstractBeamDeflector> getBeamDeflector(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::shared_ptr<AbstractBeamDeflector>, 
            Scanner,                               
            getBeamDeflector,                       
            idx                                    
        );
    }

    void setBeamDeflector(std::shared_ptr<AbstractBeamDeflector> beamDeflector, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                                   
            Scanner,                                
            setBeamDeflector,                       
            beamDeflector, idx                    
        );
    }

    std::shared_ptr<AbstractDetector> getDetector(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::shared_ptr<AbstractDetector>, 
            Scanner,                          
            getDetector,                      
            idx                               
        );
    }

    void setDetector(std::shared_ptr<AbstractDetector> detector, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                              
            Scanner,                           
            setDetector,                       
            detector, idx                    
        );
    }

    FWFSettings &getFWFSettings(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            FWFSettings &,                    
            Scanner,                         
            getFWFSettings,                  
            idx                              
        );
    }

    void setFWFSettings(FWFSettings const &fwfSettings, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setFWFSettings,                   
            fwfSettings, idx                
        );
    }

    std::list<int>& getSupportedPulseFreqs_Hz(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::list<int>&,                    
            Scanner,
            getSupportedPulseFreqs_Hz,
            idx
        );
    }

    void setSupportedPulseFreqs_Hz(std::list<int> &supportedPulseFreqs_Hz, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setSupportedPulseFreqs_Hz,       
            supportedPulseFreqs_Hz, idx     
        );
    }

    int getMaxNOR(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            int,                             
            Scanner,                          
            getMaxNOR,       
            idx     
        );
    }

    void setMaxNOR(int const maxNOR, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setMaxNOR,       
            maxNOR, idx     
        );
    }

    int getNumTimeBins(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            int,                             
            Scanner,                          
            getNumTimeBins,       
            idx     
        );
    }

    void setNumTimeBins(int const numTimeBins, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setNumTimeBins,       
            numTimeBins, idx     
        );
    }

    int getPeakIntensityIndex(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            int,                             
            Scanner,                          
            getPeakIntensityIndex,       
            idx     
        );
    }

    void setPeakIntensityIndex(int const pii, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setPeakIntensityIndex,       
            pii, idx     
        );
    }

    std::vector<double>& getTimeWave(size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            std::vector<double>&,                             
            Scanner,                          
            getTimeWave,       
            idx     
        );
    }

    void setTimeWave(std::vector<double> &timewave, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setTimeWave,       
            timewave, idx     
        );
    }

    void setTimeWave(std::vector<double> &&timewave, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setTimeWave,       
            timewave, idx     
        );
    }


    double getReceivedEnergyMin(size_t const idx) const override {
        PYBIND11_OVERLOAD_PURE(
            double,                             
            Scanner,                          
            getReceivedEnetgyMin,       
            idx     
        );
    }

    void setReceivedEnergyMin(double receivedEnergyMin_W, size_t const idx) override {
        PYBIND11_OVERLOAD_PURE(
            void,                             
            Scanner,                          
            setReceivedEnetgyMin,       
            receivedEnergyMin_W, idx     
        );
    }
        
};
