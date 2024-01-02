#pragma once

#include <memory>

#include <Asset.h>
#include <ScannerHead.h>
#include <AbstractBeamDeflector.h>
#include <ScanningDevice.h>
class AbstractDetector;
#include <scanner/ScanningPulseProcess.h>
#include <scanner/detector/PulseTaskDropper.h>
#include <scanner/detector/PulseThreadPoolInterface.h>
#include <FWFSettings.h>
#include <Platform.h>
#include <maths/Directions.h>
#include <maths/Rotation.h>
#include <UniformNoiseSource.h>
#include <RandomnessGenerator.h>
#include <scanner/Trajectory.h>
#include <scanner/Measurement.h>
namespace helios { namespace filems { class FMSFacade; }}
using helios::filems::FMSFacade;



/**
 * @brief Class representing a scanner asset
 */
class Scanner : public Asset {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The scanner's identifier
     */
    std::string id = "SCANNER-ID";
    /**
     * @brief Flag specifying if write waveform (true) or not (false)
     */
    bool writeWaveform = false;
    /**
     * @brief Flag specifying if write pulse (true) or not (false)
     */
    bool writePulse = false;
    /**
     * @brief Flag specifying if calculate echo width (true) or not (false)
     */
    bool calcEchowidth = false;
    /**
     * @brief Flag specifying if compute full wave noise (true) or not (false)
     */
    bool fullWaveNoise = false;
    /**
     * @brief Flag specifying if disable platform noise (true) or not (false)
     */
    bool platformNoiseDisabled = false;
    /**
	 * @brief Flag specifying if use fixed incidence angle (true)
	 * or not (false)
	 */
    bool fixedIncidenceAngle = false;

    /**
	 * @brief Pulse frequency (hertz)
	 */
    int cfg_setting_pulseFreq_Hz = 0;

    // State variables:
    /**
	 * @brief Flag specifying if scanner is active (true) or not (false)
	 *
	 * When a scanner is not active, it is not sensing
	 */
    bool state_isActive = true;


    /**
	 * @brief The scanning pulse process used by the scanner
	 * @see ScanningPulseProcess
	 */
    std::unique_ptr<ScanningPulseProcess> spp = nullptr;


public:
    /**
	 * @brief Main facade to file management system
	 */
    std::shared_ptr<FMSFacade> fms;
    /**
	 * @brief Platform carrying the scanner
	 * @see Platform
	 */
    std::shared_ptr<Platform> platform;
    /**
	 * @brief Historical vector of all output paths where scanner measurements
	 *  were written
	 *
	 * It can be nullptr when no historical tracking of all output paths is
	 *  requested
	 */
    std::shared_ptr<std::vector<std::string>> allOutputPaths = nullptr;
    /**
	 * @brief Historical vector of all measurements performed by the scanner
	 *
	 * It can be nullptr when no historical tracking of all measurements is
	 * requested
	 */
    std::shared_ptr<std::vector<Measurement>> allMeasurements = nullptr;
    /**
	 * @brief Historical vector of all trajectory points recorded by the
	 * scanner
	 *
	 * It can be nullptr when no historical tracking of trajectory is requested
	 */
    std::shared_ptr<std::vector<Trajectory>> allTrajectories = nullptr;
    /**
	 * @brief Mutex to handle concurrent access to historical vector of all
	 * measurements and historical vector of all trajectory points
	 *
	 * @see Scanner::allMeasurements
	 * @see Scanner::alTrajectories
	 */
    std::shared_ptr<std::mutex> allMeasurementsMutex = nullptr;
    /**
	 * @brief Vector of measurements performed by the scanner at current cycle
	 *
	 * It can be nullptr when no tracking of measurements by cycle is requested
	 */
    std::shared_ptr<std::vector<Measurement>> cycleMeasurements = nullptr;
    /**
     * @brief Vector of trajectory points recorded by the scanner
     *
     * It can be nullptr when no tracking of trajectory points by cycle is
     * requested
     */
    std::shared_ptr<std::vector<Trajectory>> cycleTrajectories = nullptr;
    /**
     * @brief Mutex to handle concurrent access to vector of measurements and
     * vector of trajectory points by cycle
     *
     * @see Scanner::cycleMeasurements
     * @see Scanner::cycleTrajectories
     */
    std::shared_ptr<std::mutex> cycleMeasurementsMutex = nullptr;

    // Trajectory output processing interval
    /**
	 * @brief Time interval between record of trajectory points. When it is
	 * exactly 0, then no trajectory points will be recorded.
	 *
	 * NOTICE that it is given in nanoseconds, while the trajectoryTimeInterval
	 *  in the ScannerSettings class is given in seconds, as the user argument
	 *  itself
	 * @see ScannerSettings::trajectoryTimeInterval
	 */
    double trajectoryTimeInterval_ns = 0.0;
    /**
     * @brief GPS time (nanoseconds) corresponding to last recorded trajectory
     * point
     */
    double lastTrajectoryTime;

    // Randomness generators for single thread mode only
    /**
     * @brief First randomness generator for single thread mode
     */
    std::shared_ptr<RandomnessGenerator<double>> randGen1 = nullptr;
    /**
     * @brief Second randomness generator for single thread mode
     */
    std::shared_ptr<RandomnessGenerator<double>> randGen2 = nullptr;
    /**
     * @brief Uniform noise source for single thread mode
     */
    std::shared_ptr<UniformNoiseSource<double>>
        intersectionHandlingNoiseSource = nullptr;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Scanner constructor
     * @param pulseFreqs List of supported pulse frequencies (hertz)
     * @see Scanner::id
     * @see Scanner::writeWaveform
     * @see Scanner::calcEchowidth
     * @see Scanner::fullWaveNoise
     * @see Scanner::platformNoiseDisabled
     */
    Scanner(
        std::string const id,
        std::list<int> const &pulseFreqs,
        bool const writeWaveform=false,
        bool const writePulse=false,
        bool const calcEchowidth=false,
        bool const fullWaveNoise=false,
        bool const platformNoiseDisabled=false
    );
    /**
	 * @brief Copy constructor for the Scanner
	 * @param scanner The scanner to be copied
	 */
    Scanner(Scanner &scanner);
    virtual ~Scanner() = default;

    // ***   C L O N E   *** //
    // ********************* //
    /**
     * @brief Make a clone of this scanner
     * @return Shared pointer pointing to the clone of this scanner
     */
    virtual std::shared_ptr<Scanner> clone() = 0;
protected:
    /**
     * @brief Assist the clone method by means of handling cloning of
     *  attributes from this scanner to the new one (sc)
     * @param sc The scanner which attributes must be assigned as clones of
     *  the caller scanner (this)
     */
    virtual void _clone(Scanner &sc) const;
public:

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Prepare the scanner to deal with the simulation.
     */
    virtual void prepareSimulation(bool const legacyEnergyModel=0) = 0;
    /**
     * @brief Initialize randomness generators and noise sources that are
     *  necessary for sequential pulse computations
     */
    void initializeSequentialGenerators();
    /**
     * @brief Build the scanning pulse process to be used by the scanner
     *  during simulation
     * @param dropper Simulation's task dropper
     * @param pool Simulation's thread pool
     * @return Built scanning pulse process
     * @see Simulation::parallelizationStrategy
     * @see Simulation::taskDropper
     * @see Simulation::threadPool
     */
    void buildScanningPulseProcess(
        int const parallelizationStrategy,
        PulseTaskDropper &dropper,
        std::shared_ptr<PulseThreadPoolInterface> pool
    );
    /**
     * @brief Apply scanner settings
     * @param settings Scanner settings to be applied
     * @param idx The index of the scanning device to which scanner settings
     *  must be applied
     * @see ScannerSettings
     */
    virtual void applySettings(
        std::shared_ptr<ScannerSettings> settings, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::applySettings(std::shared_ptr<ScannerSettings>) method
     * @see Scanner::applySettings(std::shared_ptr<ScannerSettings>)
     */
    inline void applySettings(std::shared_ptr<ScannerSettings> settings)
    {applySettings(settings, 0);}
    /**
	 * @brief Retrieve current scanner settings and build a new ScannerSettings
	 *  object with them
     * @param idx The index of the scanning device which scanning settings must
     *  be retrieved
	 * @return Newly created ScannerSettings object with current scanner
	 *  settings
	 */
    virtual std::shared_ptr<ScannerSettings> retrieveCurrentSettings(
        size_t const idx
    );
    /**
     * @brief Non index version of the
     *  Scanner::retrieveCurrentSettings(size_t const) method
     * @see Scanner::retrieveCurrentSettings(size_t const)
     */
    std::shared_ptr<ScannerSettings> retrieveCurrentSettings()
    {return retrieveCurrentSettings(0);}
    /**
	 * @brief Apply full wave form settings
	 * @param settings Full wave form settings to be applied
     * @param idx The index of the scanning device which Full WaveForm settings
     *  must be updated
	 * @see FWFSettings
	 */
    virtual void applySettingsFWF(FWFSettings settings, size_t const idx);
    /**
     * @brief Non index version of the
     *  Scanner::applySettingsFWF(FWFSettings, size_t const) method
     * @see Scanner::applySettingsFWF(FWFSettings, size_t const)
     */
    inline void applySettingsFWF(FWFSettings settings)
    {applySettingsFWF(settings, 0);}
    /**
	 * @brief Perform computations for current simulation step
	 * @param legIndex Index of current leg
	 * @param currentGpsTime GPS time of current pulse
	 */
    virtual void doSimStep(
        unsigned int legIndex,
        double const currentGpsTime
    ) = 0;
    /**
	 * @brief Build a string representation of the scanner
	 * @return String representing the scanner
	 */
    std::string toString();
    /**
	 * @brief Compute the number of rays depending on beam sample quality for
     *  the scanning device
	 */
    virtual void calcRaysNumber(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::calcRaysNumber(size_t const)
     *  method
     * @see Scanner::calcRaysNumber(size_t const)
     */
    inline void calcRaysNumber() {calcRaysNumber(0);}
    /**
     * @brief Prepare wave discretization
     * @see Scanner::numTimeBins
     * @see Scanner::time_wave
     * @see Scanner::peakIntensityIndex
     */
    virtual void prepareDiscretization(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::prepareDiscretization method
     * @see Scanner::prepareDiscretization(size_t const)
     */
    inline void prepareDiscretization() {this->prepareDiscretization(0);}
    /**
     * @brief Compute the footprint radius \f$f_{r}\f$
     *
     * \f[
     *  f_{r} = \sqrt{\frac{f_{a}}{\pi}}
     * \f]
     *
     * @param distance Distance \f$d\f$
     * @return Footprint radius \f$f_{r}\f$
     * @see Scanner::calcFootprintArea
     */
    double calcFootprintRadius(double const distance, size_t const idx);
    /**
     * @see ScanningDevice::calcAtmosphericAttenuation
     */
    virtual double calcAtmosphericAttenuation(size_t const idx) const = 0;
    /**
     * @brief Non index version of Scanner::calcAtmosphericAttenuation
     * @see Scanner::calcAtmosphericAttenuation(size_t const)
     */
    inline double calcAtmosphericAttenuation() const
    {return calcAtmosphericAttenuation(0);}
    /**
     * @brief Compute the absolute beam attitude considering the mount relative
     * attitude and the deflector relative attitude
     * @see ScannerHead::getMountRelativeAttitude
     * @see AbstractBeamDeflector::getEmitterRelativeAttitude
     */
    virtual Rotation calcAbsoluteBeamAttitude(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::calcAbsoluteBeamAttitude
     *  function
     * @see Scanner::calcAbsoluteBeamAttitude(size_t const)
     */
    inline Rotation calcAbsoluteBeamAttitude()
    {return calcAbsoluteBeamAttitude(0);}

    /**
     * @brief Check if given number of return (nor) is inside
     *  expected boundaries.
     * If scanner maxNOR is 0 or nor is less than maxNOR, then the check
     *  is passed (true is returned). Otherwise, it is not passed (false is
     *  returned).
     * @param nor Current number of return
     * @return True if the check is passed, false otherwise
     * @see ScanningDevice::maxNOR
     */
    virtual bool checkMaxNOR(int const nor, size_t const idx) = 0;
    /**
     * @brief Non index version of
     *  Scanner::checkMaxNOR(int const, size_t const) method
     * @see Scanner::checkMaxNOR(int const, size_t const)
     */
    inline bool checkMaxNOR(int const nor) {return checkMaxNOR(nor, 0);}
    /**
     * @brief Perform ray casting to find intersections
     * @param[in] handleSubray The function where computed subrays must be
     *  delegated to
     * @param[out] reflections Where reflections must be stored when a hit is
     *  registered
     * @param[out] intersects Where intersections must be stored when a hit is
     *  registered
     * @param[in] idx The index of the scanning device
     * @see FullWaveformPulseRunnable::computeSubrays
     * @see FullWaveformPulseRunnable::handleSubray
     * @see ScanningDevice::computeSubrays
     */
    virtual void computeSubrays(
        std::function<void(
            Rotation const
            &subrayRotation,
            int const subrayRadiusStep,
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
    ) = 0;

    /**
     * @brief Initialize full waveform
     * While the vector is not strictly initialized in this function,
     * necessary variables are computed here.
     * @param[out] nsPerBin The size of each bin in nano seconds
     * @param[out] distanceThreshold Limit distance threshold
     * @param[out] peakIntensityIndex Index of intensity peak
     * @param[out] numFullwaveBins How many bins are necessary to discretize
     *  the full waveform
     * @param[in] idx The index of the scanning device
     * @return True if it is possible to initialize the full waveform,
     * false otherwise.
     * @see FullWaveformPulseRunnable::initializeFullWaveform
     * @see FullWaveformPulseRunnable::digestIntersections
     * @see ScanningDevice::initializeFullWaveform
     */
    virtual bool initializeFullWaveform(
        double const minHitDist_m,
        double const maxHitDist_m,
        double &minHitTime_ns,
        double &maxHitTime_ns,
        double &nsPerBin,
        double &distanceThreshold,
        int &peakIntensityIndex,
        int &numFullwaveBins,
        size_t const idx
    ) = 0;

    /**
     * @brief Handle to which scanning device request the intensity computation
     * @param idx The index of the scanning device that must compute the
     *  intensity
     * @see ScanningDevice::calcIntensity
     */
    virtual double calcIntensity(
        double const incidenceAngle,
        double const targetRange,
        Material const &mat,
        int const subrayRadiusStep,
        size_t const idx
#if DATA_ANALYTICS >= 2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) const = 0;
    /**
     * @brief Handle to which scanning device request the intensity computation
     * @param idx The index of the scanning device that must compute the
     *  intensity
     * @see ScanningDevice::calcIntensity
     */
    virtual double calcIntensity(
        double const targetRange,
        double const sigma,
        int const subrayRadiusStep,
        size_t const idx
    ) const = 0;


    // ***  SIM STEP UTILS  *** //
    // ************************ //
    /**
     * @brief Handle position and attitude noise
     * @param absoluteBeamOrigin Beam origin (position) to add noise to if
     * requested
     * @param absoluteBeamAttitude Beam attitude to add noise to if requested
     */
    void handleSimStepNoise(
        glm::dvec3 & absoluteBeamOrigin,
        Rotation & absoluteBeamAttitude
    );
    /**
     * @brief Exposes ScanningPulseProcess:onLegComplete method of the
     *  scanning pulse process defining this scanner
     */
    virtual void onLegComplete() {spp->onLegComplete();}
    /**
     * @brief Exposes ScanningPulseProcess::onSimulationFinished method of the
     *  scanning pulse process defining this scanner
     */
    void inline onSimulationFinished() {spp->onSimulationFinished();}
    /**
     * @brief Handle trajectory output whatever it is to output file, to
     * all trajectories vector or to cycle trajectories vector
     * @param currentGpsTime Current GPS time (nanoseconds)
     * @see Scanner::allTrajectories
     * @see Scanner::cycleTrajectories
     */
    void handleTrajectoryOutput(double const currentGpsTime);
    /**
     * @brief Track given output path in a thread safe way
     * @param path Output path to be tracked
     * @see Scanner::allOutputPaths
     */
    void trackOutputPath(std::string const &path);

    // *** GETTERs and SETTERs *** //
    // *************************** //
    /**
     * @brief Obtain the requested scanning device.
     * @param idx The index of the scanning device to be obtained.
     * @see ScanningDevice::getScanningDevice
     */
    virtual ScanningDevice& getScanningDevice(size_t const idx) = 0;
    /**
	 * @brief Obtain the current pulse number of the scanning device
     * @param idx The index of the scanning device which pulse number must be
     *  obtained
	 * @return The current pulse number of the scanning device
	 * @see ScanningDevice::state_currentPulseNumber
	 */
    virtual int getCurrentPulseNumber(size_t const idx) const = 0;
    /**
     * @brief Non index version of the Scanner::getCurrentPulseNumber method
     * @see Scanner::getCurrentPulseNumber(size_t const)
     */
    inline int getCurrentPulseNumber() const {return getCurrentPulseNumber(0);}
    /**
	 * @brief Obtain the number of rays of the scanning device
     * @param idx The index of the scanning device which number of rays must
     *  be obtained
	 * @return Number of rays of the scanning device
	 * @see ScanningDevice::numRays
	 */
    virtual int getNumRays(size_t const idx) const = 0;
    /**
     * @brief Non index version of the Scanner::getNumRays(size_t const)
     *  method
     * @see Scanner::getNumRays(size_t const)
     */
    inline int getNumRays() const {return getNumRays(0);}
    /**
	 * @brief Set the number of rays of the scanning device
	 * @param numRays New number of rays for the scanning device
     * @param idx The index of the scanning device which number of rays must
     *  be setted
	 * @see ScanningDevice::numRays
	 */
    virtual void setNumRays(int const numRays, size_t const idx) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setNumRays(int const, size_t const) method
     * @see Scanner::setNumRays(int const, size_t const)
     */
    inline void setNumRays(int const numRays) {setNumRays(numRays, 0);}

    /**
	 * @brief Obtain the pulse frequency
	 * @return Pulse frequency (hertz)
	 * @see Scanner::cfg_setting_pulseFreq_Hz
	 */
    inline int getPulseFreq_Hz() const {return this->cfg_setting_pulseFreq_Hz;}
    /**
     * @brief Set the pulse frequency
     * @param pulseFreq_Hz New pulse frequency (hertz)
     * @see Scanner::cfg_setting_pulseFreq_Hz
     */
    void setPulseFreq_Hz(int const pulseFreq_Hz);

    /**
	 * @brief Get the pulse length
	 * @param idx The index of the scanning device which pulse length must
	 *  be obtained (by default 0, it is the first one)
	 * @return Pulse length (nanoseconds)
	 * @see ScanningDevice::cfg_device_pulseLength_ns
	 */
    virtual double getPulseLength_ns(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getPulseLength_ns
	 *  method
	 * @see Scanner::getPulseLength_ns(size_t const)
	 */
    inline double getPulseLength_ns() const {return getPulseLength_ns(0);}
    /**
     * @brief Set the pulse length
     * @param pulseLength_ns New pulse length (nanoseconds)
     * @param idx The index of the scanning device which pulse length must
     *  be setted (by default 1, it is the first one)
	 * @see ScanningDevice::cfg_device_pulseLength_ns
     */
    virtual void setPulseLength_ns(
        double const pulseLength_ns, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setPulseLength_ns
	 *  method
	 * @see Scanner::setPulseLength_ns(double const, size_t const)
	 */
    inline void setPulseLength_ns(double const pulseLength_ns)
    {setPulseLength_ns(pulseLength_ns, 0);}

    /**
	 * @brief Check if last pulse was hit (true) or not (false) for the
     *  scanning device.
     * @param idx The index of the scanning device which last pulse must be
     *  checked.
	 * @return True if the last pulse of the scanning device was hit, false
     *  otherwise.
	 * @see ScanningDevice::state_lastPulseWasHit
     * @see ScanningDevice::lastPulseWasHit
	 */
    virtual bool lastPulseWasHit(size_t const idx) const = 0;
    /**
     * @brief Non index version of the Scanner::lastPulseWasHit method
     * @see Scanner::lastPulseWasHit method
     */
    inline bool lastPulseWasHit() const {return lastPulseWasHit(0);}
    /**
	 * @brief Specify if last pulse was hit (true) or not (false)
	 * @param lastPulseWasHit New last pulse hit specification
	 * @see Scanner::state_lastPulseWasHit
	 */
    virtual void setLastPulseWasHit(
        bool const lastPulseWasHit, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the Scanner::setLastPulseWasHit method
     * @see Scanner::setLastPulseWasHit(bool const, size_t const)
     */
    inline void setLastPulseWasHit(bool const lastPulseWasHit)
    {setLastPulseWasHit(lastPulseWasHit, 0);}

    /**
     * @brief Obtain beam divergence
     * @param idx The index of the scanning device which beam divergence must
     *  be obtained (by default 0, it is the first one)
     * @return Beam divergence (radians)
     * @see ScanningDevice::beamDivergence_rad
     */
    virtual double getBeamDivergence(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getBeamDivergence
	 *  method
	 * @see Scanner::getBeamDivergence(size_t const)
	 */
    inline double getBeamDivergence() const {return getBeamDivergence(0);}
    /**
	 * @brief Set beam divergence
	 * @param beamDivergence New beam divergence (radians)
	 * @param idx The index of the scanning device which beam divergence must
	 *  be setted (by default 0, it is the first one)
     * @see ScanningDevice::beamDivergence_rad
	 */
    virtual void setBeamDivergence(
        double const beamDivergence, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setBeamDivergence
	 *  method
	 * @see Scanner::setBeamDivergence(double const, size_t const)
	 */
    inline void setBeamDivergence(double const beamDivergence)
    {setBeamDivergence(beamDivergence, 0);}

    /**
     * @brief Obtain average power
     * @param idx The index of the scanning device which average power must be
     *  obtained (by default 0, it is the first one)
     * @return Average power (watts)
     * @see ScanningDevice::averagePower_w
     */
    virtual double getAveragePower(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getAveragePower method
	 * @see Scanner::getAveragePower(size_t const)
	 */
    inline double getAveragePower() const {return getAveragePower(0);}
    /**
	 * @brief Set average power
	 * @param averagePower New average power (watts)
	 * @param idx The index of the scanning device which average power must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::averagePower_w
	 */
    virtual void setAveragePower(
        double const averagePower, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setAveragePower method
	 * @see Scanner::setAveragePower(double const, size_t const)
	 */
    inline void setAveragePower(double const averagePower)
    {setAveragePower(averagePower, 0);}

    /**
     * @brief Get beam quality
     * @param idx The index of the scanning device which beam quality must be
     *  obtained (by default 0, it is the first one)
     * @return Beam quality
     * @see ScanningDevice::beamQuality
     */
    virtual double getBeamQuality(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getBeamQuality method
	 * @see Scanner::getBeamQuality(size_t const)
	 */
    inline double getBeamQuality() const {return getBeamQuality(0);}
    /**
	 * @brief Set beam quality
	 * @param beamQuality New beam quality
     * @param idx The index of the scanning device which beam quality must be
     *  obtained (by default 0, it is the first one)
     * @see ScanningDevice::beamQuality
	 */
    virtual void setBeamQuality(
        double const beamQuality, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setBeamQuality method
	 * @see Scanner::setBeamQuality(double const, size_t const)
	 */
    inline void setBeamQuality(double const beamQuality)
    {setBeamQuality(beamQuality, 0);}

    /**
     * @brief Obtain device efficiency
     * @param idx The index of the scanning device which efficiency must be
     *  obtained (by default 0, it is the first one)
     * @return Device efficiency
     * @see ScanningDevice::efficiency
     */
    virtual double getEfficiency(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getEfficiency method
	 * @see Scanner::getEfficiency(size_t const)
	 */
    inline double getEfficiency() const {return getEfficiency(0);}
    /**
	 * @brief Set device efficiency
	 * @param efficiency New device efficiency
	 * @param idx The index of the scanning device which efficiency must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::efficiency
	 */
    virtual void setEfficiency(double const efficiency, size_t const idx=0)=0;
    /**
	 * @brief No index argument version of the Scanner::setEfficiency method
	 * @see Scanner::setEfficiency(double const, size_t const)
	 */
    inline void setEfficiency(double const efficiency)
    {setEfficiency(efficiency, 0);}

    /**
	 * @brief Get receiver diameter
	 * @param idx The index of the scanning device which efficiency must be
	 *  obtained (by default 0, it is the first one)
	 * @return Receiver diameter
	 * @see ScanningDevice::receiverDiameter_m
	 */
    virtual double getReceiverDiameter(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getReceiverDiameter
	 * @see Scanner::getReceiverDiameter(size_t const)
	 */
    inline double getReceiverDiameter() const {return getReceiverDiameter(0);}
    /**
	 * @brief Set receiver diameter
	 * @param receiverDiameter  New receiver diameter
	 * @param idx The index of the scanning device which efficiency must be
	 *  setted (by default 0, it is the first one)
	 * @see ScanningDevice::receiverDiameter_m
	 */
    virtual void setReceiverDiameter(
        double const receiverDiameter, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setReceiverDiameter
	 *  method
	 * @see Scanner::setReceiverDiameter(double const, size_t const)
	 */
    inline void setReceiverDiameter(double const receiverDiameter)
    {setReceiverDiameter(receiverDiameter, 0);}

    /**
     * @brief Get device visibility
     * @param idx The index of the scanning device which visibility must be
     *  obtained (by default 0, it is the first one)
     * @return Device visibility (kilometers)
     * @see ScanningDevice::visibility_km
     */
    virtual double getVisibility(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getVisibility method
	 * @see Scanner::getVisibility(size_t const)
	 */
    inline double getVisibility() const{return getVisibility(0);}
    /**
	 * @brief Set device visibility
	 * @param visibility New device visibility (kilometers)
	 * @param idx The index of the scanning device which visibility must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::visibility_km
	 */
    virtual void setVisibility(double const visibility, size_t const idx)=0;
    /**
	 * @brief No index argument version of the Scanner::setVisibility method
	 * @see Scanner::setVisibility(double const, size_t const)
	 */
    inline void setVisibility(double const visibility)
    {setVisibility(visibility, 0);}

    /**
     * @brief Obtain wave length
     * @param idx The index of the scanning device which wavelength must be
     *  obtained (by default 0, it is the first one)
     * @return Wave length (meters)
     * @see ScanningDevice::wavelength_m
     */
    virtual double getWavelength(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getWavelength method
	 * @see Scanner::getWavelength(size_t const)
	 */
    inline double getWavelength() const {return getWavelength(0);}
    /**
	 * @brief Set wave length
	 * @param wavelength New wave length (meters)
	 * @param idx The index of the scanning device which wavelength must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::wavelength_m
	 */
    virtual void setWavelength(double const wavelength, size_t const idx)=0;
    /**
	 * @brief No index argument version of the Scanner::setWavelength method
	 * @see Scanner::setWavelength(double const, size_t const)
	 */
    inline void setWavelength(double const wavelength)
    {setWavelength(wavelength, 0);}

    /**
	 * @brief Obtain atmospheric extinction
	 * @param idx The index of the scanning device which atmospheric extinction
	 *  must be obtained (by default 0, it is the first one)
	 * @return Atmospheric extinction
	 * @see ScanningDevice::atmosphericExtinction
	 */
    virtual double getAtmosphericExtinction(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the
	 *  Scanner::getAtmosphericExtinction method
	 * @return Scanner::getAtmosphericExtinction(size_t const)
	 */
    inline double getAtmosphericExtinction() const
    {return getAtmosphericExtinction(0);}
    /**
     * @brief Set atmospheric extinction
     * @param atmosphericExtinction New atmospheric extinction
	 * @param idx The index of the scanning device which atmospheric extinction
	 *  must be setted (by default 0, it is the first one)
	 * @see ScanningDevice::atmosphericExtinction
     */
    virtual void setAtmosphericExtinction(
        double const atmosphericExtinction,
        size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the
	 *  Scanner::setAtmosphericExtinction method
	 * @see Scanner::setAtmosphericExtinction(double const, size_t const)
	 */
    inline void setAtmosphericExtinction(double const atmosphericExtinction)
    {setAtmosphericExtinction(atmosphericExtinction, 0);}

    /**
     * @brief Obtain beam waist radius
     * @param idx The index of the scanning device which beam waist radius must
     *  be obtained (by default 0, it is the first one)
     * @return Beam waist radius
     * @see ScanningDevice::beamWaistRadius
     */
    virtual double getBeamWaistRadius(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getBeamWaistRadius
	 *  method
	 * @see Scanner::getBeamWaistRadius(size_t const)
	 */
    inline double getBeamWaistRadius() const {return getBeamWaistRadius(0);}
    /**
	 * @brief Set beam waist radius
	 * @param beamWaistRadius New beam waist radius
	 * @param idx The index of the scanning device which beam waist radius must
	 *  be setted (by default 0, it is the first one)
     * @see ScanningDevice::beamWaistRadius
	 */
    virtual void setBeamWaistRadius(
        double const beamWaistRadius, size_t const idx
        ) = 0;
    /**
	 * @brief No index argument version of the Scanner::setBeamWaistRadius
	 *  method
	 * @see Scanner::setBeamWaistRadius(double const, size_t const)
	 */
    inline void setBeamWaistRadius(double const beamWaistRadius)
    {setBeamWaistRadius(beamWaistRadius, 0);}
    /**
	 * @brief Obtain the head's relative emitter position
	 * @param idx The index of the scanning device which head's relative
	 *  emitter position must be obtained (by default 0, it is the first one)
	 * @return The head's relative emitter position
	 * @see ScanningDevice::headRelativeEmitterPosition
	 */
    virtual glm::dvec3 getHeadRelativeEmitterPosition(
        size_t const idx=0
    ) const = 0;
    /**
	 * @brief Set the head's relative emitter position
	 * @param pos The new position for the head's relative emitter
	 * @param idx The index of the scanning device which head's relative
	 *  emitter position must be setted (by default 0, it is the first one)
	 * @see ScanningDevice::headRelativeEmitterPosition
	 */
    virtual void setHeadRelativeEmitterPosition(
        glm::dvec3 const &pos, size_t const idx=0
    ) = 0;
    /**
     * @brief Obtain the head's relative emitter position by reference (can be
     *  written)
     * @param idx The index of the scanning device which head's relative
     *  emitter position must be obtained (by default 0, it is the first one)
     * @return The head's relative emitter position by reference
     * @see ScanningDevice::headRelativeEmitterPosition
     */
    virtual glm::dvec3 & getHeadRelativeEmitterPositionByRef(
        size_t const idx=0
    ) = 0;
    /**
	 * @brief Obtain the head's relative emitter attitude
	 * @param idx The index of the scanning device which head's relative
	 *  emitter attitude must be obtained (by default 0, it is the first one)
	 * @return The head's relative emitter attitude
	 * @see ScanningDevice::headRelativeEmitterAttitude
	 */
    virtual Rotation getHeadRelativeEmitterAttitude(size_t const idx=0)const=0;
    /**
	 * @brief Obtain the head's relative emitter attitude
	 * @param attitude The new attitude for the head's relative emitter
	 * @param idx The index of the scanning device which head's relative
	 *  emitter attitude must be setted (by default 0, it is the first one)
	 */
    virtual void setHeadRelativeEmitterAttitude(
        Rotation const &attitude, size_t const idx=0
    ) = 0;
    /**
     * @brief Obtain the head's relative emitter attitude by reference (can be
     *  written)
     * @param idx The index of the scanning device which head's relative
     *  emitter attitude must be obtained (by default 0, it is the first one)
     * @return The head's relative emitter attitude by reference
     * @see ScanningDevice::headRelativeEmitterAttitude
     */
    virtual Rotation & getHeadRelativeEmitterAttitudeByRef(
        size_t const idx=0
    ) = 0;

    /**
     * @brief Obtain \f$B_{t2}\f$
     * @param idx The index of the scanning device which cached Bt2 (square of
     *  beam divergence) must be obtained (by default 0, it is the first one)
     * @return \f$B_{t2}\f$
     * @see ScanningDevice::cached_Bt2
     */
    virtual double getBt2(size_t const idx) const = 0;
    /**
     * @brief No index argument version of the Scanner::getBt2 method
     * @see Scanner::getBt2(size_t const)
     */
    inline double getBt2() const {return getBt2(0);}
    /**
     * @brief Set \f$B_{t2}\f$
     * @param bt2 New \f$B_{t2}\f$
     * @param idx The index of the scanning device which cached Bt2 (square of
     *  beam divergence) must be setted (by default 0, it is the first one)
     * @see ScanningDevice::cached_Bt2
     */
    virtual void setBt2(double const bt2, size_t const idx) = 0;
    /**
     * @brief No index argument version of the Scanner::setBt2 method
     * @Scanner::setBt2(double const, size_t const)
     */
    inline void setBt2(double const bt2) {setBt2(bt2, 0);}

    /**
	 * @brief Obtain \f$D_{r2}\f$
	 * @param idx The index of the scanning device which cached Dr2 (square of
	 *  receiver diameter) must be obtained (by default 0, it is the first one)
	 * @return \f$D_{r2}\f$
	 * @see ScanningDevice::cached_Dr2
	 */
    virtual double getDr2(size_t const idx) const = 0;
    /**
     * @brief No index argument version of the Scanner::getDr2 method
     * @see Scanner::getDr2(size_t const)
     */
    inline double getDr2() const {return getDr2(0);}
    /**
	 * @brief Set \f$D_{r2}\f$
	 * @param dr2 New \f$D_{t2}\f$
	 * @param idx The index of the scanning device which cached Dr2 (square of
	 *  receiver diameter) must be obtained (by default 0, it is the first one)
	 * @see ScanningDevice::cached_Dr2
	 */
    virtual void setDr2(double const dr2, size_t const idx) = 0;
    /**
	 * @brief No index argument version of the Scanner::setDr2 method
	 * @Scanner::setDr2(double const, size_t const)
	 */
    inline void setDr2(double const dr2) {setDr2(dr2, 0);}

    /**
	 * @brief Check if scanner is active (true) or not (false)
	 * @return True if scanner is active, false otherwise
	 * @see Scanner::state_isActive
	 */
    inline bool isActive() const {return this->state_isActive;}
    /**
	 * @brief Set scanner active status. True to make it active, false to
	 * make it inactive
	 * @param active New scanner active status
	 * @see Scanner::state_isActive
	 */
    inline void setActive(bool const active) {this->state_isActive = active;}

    /**
	 * @brief Check if scanner is configured to write wave form (true) or not
	 *  (false)
	 * @return True if scanner is configured to write wave form, false
	 *  otherwise
	 * @see Scanner::writeWaveform
	 */
    inline bool isWriteWaveform() const {return this->writeWaveform;}
    /**
     * @brief Set scanner write wave form configuration.
     * @param writeWaveform True to make scanner write wave form, false
     *  otherwise
     * @see Scanner::writeWaveform
     */
    inline void setWriteWaveform(bool const writeWaveform)
    {this->writeWaveform = writeWaveform;}
    /**
     * @brief Check if scanner is configured to write pulses (true) or not
     *  (false)
     * @return True if scanner is configured to write pulses, false
     *  otherwise
     * @see Scanner::writePulse
     */
    inline bool isWritePulse() const {return this->writePulse;}
    /**
     * @brief Set scanner write pulse configuration.
     * @param writePulse True to make scanner write pulse, false otherwise
     * @see Scanner::writePulse
     */
    inline void setWritePulse(bool const writePulse)
    {this->writePulse = writePulse;}
    /**
     * @brief Check if scanner is configured to compute echo width (true) or
     *  not (false)
     * @return True if scanner is configured to compute echo width, false
     *  otherwise
     * @see Scanner::calcEchowidth
     */
    inline bool isCalcEchowidth() const {return this->calcEchowidth;}
    /**
     * @brief Set scanner echo width configuration.
     * @param calcEchowidth True to make scanner compute echo width,
     *  false otherwise
     * @see Scanner::calcEchowidth
     */
    inline void setCalcEchowidth(bool const calcEchowidth)
    {this->calcEchowidth = calcEchowidth;}

    /**
     * @brief Check if scanner is configured to add noise to full wave (true)
     *  or not (false)
     * @return True if scanner is configured to add noise to full wave, false
     *  otherwise
     * @see Scanner::fullWaveNoise
     */
    inline bool isFullWaveNoise() const {return this->fullWaveNoise;}
    /**
	 * @brief Set scanner full wave noise policy
	 * @param fullWaveNoise True to make scanner add noise to the full wave,
	 *  false otherwise
     * @see Scanner::fullWaveNoise
	 */
    inline void setFullWaveNoise(bool const fullWaveNoise)
    {this->fullWaveNoise = fullWaveNoise;}

    /**
     * @brief Check if platform noise is disabled (true) or not (false)
     * @return True if platform noise is disabled, false otherwise
     * @see Scanner::platformNoiseDisabled
     */
    inline bool isPlatformNoiseDisabled() {return this->platformNoiseDisabled;}
    /**
	 * @brief Set platform noise disabled flag
	 * @param platformNoiseDisabled True to disable platform noise, false
	 *  to enable it
     * @see Scanner::platformNoiseDisabled
	 */
    inline void setPlatformNoiseDisabled(bool const platformNoiseDisabled)
    {this->platformNoiseDisabled = platformNoiseDisabled;}

    /**
     * @brief Check if incidence angle is fixed (true) or not (false)
     * @return True if incidence angle is fixed, false otherwise
     * @see Scanner::fixedIncidenceAngle
     */
    inline bool isFixedIncidenceAngle() const
    {return this->fixedIncidenceAngle;}
    /**
     * @brief Set fixed incidence angle flag
     * @param fixedIncidenceAngle True to enable fixed incidence angle, false
     *  to disable it
     * @see Scanner::fixedIncidenceAngle
     */
    inline void setFixedIncidenceAngle(bool const fixedIncidenceAngle)
    {this->fixedIncidenceAngle = fixedIncidenceAngle;}

    /**
     * @brief Obtain the identifier of the scanner
     * @see Scanner::id
     */
    inline std::string getScannerId() const {return id;}
    /**
     * @brief Set the identifier of the scanner
     * @param id The new identifier for the scanner
     * @see Scanner::id
     */
    inline void setScannerId(std::string const &id) {this->id = id;}
    /**
     * @brief Set the device index (newIdx) of the device at given index
     *  (oldIdx).
     *
     * NOTE this function is not safe. It usage must be avoided unless you
     *  really know what you are doing.
     *
     * @param newIdx New index for the device
     * @param oldIdx Old index of the device to be updated
     */
    virtual void setDeviceIndex(size_t const newIdx, size_t const oldIdx) = 0;
    /**
	 * @brief Obtain scanner device identifier
	 * @return Scanner device identifier
	 * @see ScanningDevice::id
	 */
    virtual std::string getDeviceId(size_t const idx) const = 0;
    /**
	 * @brief No index argument version of the Scanner::getDeviceId method
	 * @see Scanner::getDeviceId(size_t const)
	 */
    inline std::string getDeviceId() const {return getDeviceId(0);}
    /**
	 * @brief Set the scanner device identifier
	 * @param deviceId New scanner device identifier
	 * @see ScanningDevice::id
	 */
    virtual void setDeviceId(std::string const deviceId, size_t const idx)=0;
    /**
     * @brief No index argument version of the Scanner::setDeviceId method
     * @see Scanner::setDeviceId(std::string const, size_t const)
     */
    inline void setDeviceId(std::string const deviceId)
    {setDeviceId(deviceId, 0);}
    /**
	 * @brief Obtain the number of scanning devices composing the scanner
	 */
    virtual size_t getNumDevices() const = 0;

    /**
     * @brief Obtain the scanner head of the scanning device
     * @param idx The index of the scanning device which scanner head must be
     *  obtained
     * @return The scanner head of the scanning device
     * @see ScannerHead
     */
    virtual std::shared_ptr<ScannerHead> getScannerHead(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::getScannerHead(size_t const)
     *  method
     * @see Scanner::getScannerHead(size_t const)
     */
    inline std::shared_ptr<ScannerHead> getScannerHead()
    {return getScannerHead(0);}
    /**
     * @brief Set the scanner head of the scanning device
     * @param scannerHead The scanner head to be assigned to the scanning
     *  device
     * @param idx The index of the scanning device which scanner head must be
     *  setted
     */
    virtual void setScannerHead(
        std::shared_ptr<ScannerHead> scannerHead, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the Scanner::setScannerHead method
     * @see Scanner::setScannerHead(shared_ptr<ScannerHead>, size_t const)
     */
    inline void setScannerHead(std::shared_ptr<ScannerHead> scannerHead)
    {setScannerHead(scannerHead, 0);}
    /**
     * @brief Obtain the beam deflector of the scanning device
     * @param idx The index of the scanning device which beam deflector must
     *  be obtained
     * @return The beam deflector of the scanning device
     * @see AbstractBeamDeflector
     */
    virtual std::shared_ptr<AbstractBeamDeflector> getBeamDeflector(
        size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the Scanner::getBeamDeflector(size_t const)
     *  method
     * @see Scanner::getBeamDeflector(size_t const)
     */
    inline std::shared_ptr<AbstractBeamDeflector> getBeamDeflector()
    {return getBeamDeflector(0);}
    /**
     * @brief Set the beam deflector of the scanning device
     * @param deflector New beam deflector for the scanning device
     * @param idx The index of the scanning device which beam deflector must be
     *  setted
     */
    virtual void setBeamDeflector(
        std::shared_ptr<AbstractBeamDeflector> deflector, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the Scanner::setBeamDeflector method
     * @see Scanner::setBeamDeflector
     */
    inline void setBeamDeflector(
        std::shared_ptr<AbstractBeamDeflector> deflector
    ){setBeamDeflector(deflector, 0);}
    /**
     * @brief Obtain the abstract detector of the scanning device
     * @param idx The index of the scanning device which abstract detector
     *  must be obtained
     * @return The abstract detector of the scanning device
     * @see AbstractDetector
     */
    virtual std::shared_ptr<AbstractDetector> getDetector(size_t const idx)=0;
    /**
     * @brief Non index version of the Scanner::getDetector(size_t const)
     *  method
     * @see Scanner::getDetector(size_t const)
     */
    inline std::shared_ptr<AbstractDetector> getDetector()
    {return getDetector(0);}
    /**
     * @brief Set the abstract detector of the scanning device
     * @param idx The index of the scanning device which abstract detector
     *  must be setted
     * @see AbstractDetector
     */
    virtual void setDetector(
        std::shared_ptr<AbstractDetector> detector,
        size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the Scanner::setDetector method
     * @see Scanner::setDetector(shared_ptr<AbstractDetector>, size_t const)
     */
    inline void setDetector(std::shared_ptr<AbstractDetector> detector)
    {setDetector(detector, 0);}
    /**
     * @brief Set the detector of each scanning device to the given one
     * @param detector The new detector for each scanning device
     */
    inline void setAllDetectors(std::shared_ptr<AbstractDetector> detector)
    {
        for(size_t i = 0 ; i < getNumDevices() ; ++i) setDetector(detector, i);
    }

    /**
     * @brief Obtain the Full WaveForm settings of the scanning device
     * @param idx The index of the scanning device which full waveform settings
     *  must be obtained
     * @return The full waveform settings of the scanning device
     * @see FWFSettings
     */
    virtual FWFSettings & getFWFSettings(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::getFWFSettings(size_t const)
     * @see Scanner::getFWFSettings(size_t const)
     */
    inline FWFSettings & getFWFSettings() {return getFWFSettings(0);}
    /**
     * @brief Set the Full WaveForm settings of the scanning device
     * @param idx The index of the scanning device which full waveform settings
     *  must be setted
     * @see FWFSettings
     */
    virtual void setFWFSettings(
        FWFSettings const &fwfSettings, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setFWFSettings(FWFSettings &, size_t const) method
     * @see Scanner::setFWFSettings(FWFSettings const &, size_t const)
     */
    inline void setFWFSettings(FWFSettings const &fwfSettings)
    {setFWFSettings(fwfSettings, 0);}
    /**
     * @brief Obtain the list of supported pulse frequencies of the scanning
     *  device
     * @param idx The index of the scanning device which supported pulse
     *  frequencies must be obtained
     * @return The list of supported pulse frequencies of the scanning device
     */
    virtual std::list<int>& getSupportedPulseFreqs_Hz(size_t const idx) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::getSupportedPulseFreqs_Hz(size_t const) method
     * @see Scanner::getSupportedPulseFreqs_Hz
     */
    inline std::list<int>& getSupportedPulseFreqs_Hz()
    {return getSupportedPulseFreqs_Hz(0);}
    /**
     * @brief Set the list of supported pulse frequencies (in Hertz) for the
     *  scanning device
     * @param pulseFreqs_Hz Supported pulse frequencies for the scanning device
     * @param idx The index of the scanning device which supported pulse
     *  frequencies must be setted
     */
    virtual void setSupportedPulseFreqs_Hz(
        std::list<int> &pulseFreqs_Hz, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setSupportedPulseFreqs_Hz(std::list<int> &, size_t const)
     * @see Scanner::setSupportedPulseFreqs_Hz(std::list<int> &, size_t const)
     */
    inline void setSupportedPulseFreqs_Hz(std::list<int> &pulseFreqs_Hz)
    {setSupportedPulseFreqs_Hz(pulseFreqs_Hz, 0);}
    /**
     * @brief Obtain the maximum number of returns per pulse for the scanning
     *  device (0 means no maximum at all).
     * @param idx The index of the scanning device which max NOR must be
     *  obtained
     * @return The maximum number of returns per pulse for the scanning device
     */
    virtual int getMaxNOR(size_t const idx) const = 0;
    /**
     * @brief Non index version of Scanner::getMaxNOR(size_t const)
     * @see Scanner::getMaxNOR(size_t const)
     */
    inline int getMaxNOR() const {return getMaxNOR(0);}
    /**
     * @brief Set the maximum number of returns per pulse for the scanning
     *  device (0 means no maximum at all).
     * @param maxNOR New max NOR for the scanning device
     * @param idx Index of the scanning device which max NOR must be setted
     * @see ScanningDevice::maxNOR
     */
    virtual void setMaxNOR(int const maxNOR, size_t const idx) = 0;
    /**
     * @brief Non index version of Scanner::setMaxNOR(int const, size_t const)
     * @see Scanner::setMaxNOR(int const, size_t const)
     */
    inline void setMaxNOR(int const maxNOR) {setMaxNOR(maxNOR, 0);}
    /**
     * @brief Obtain the number of bins defining the size of the time
     *  discretization for the scanning device
     * @param idx Index of the scanning device which number of time bins must
     *  be obtained
     * @return The number of time bins for the scanning device
     */
    virtual int getNumTimeBins(size_t const idx) const = 0;
    /**
     * @brief Non index version of the Scanner::getNumTimeBins(size_t const)
     *  method
     * @see Scanner::getNumTimeBins(size_t const)
     */
    inline int getNumTimeBins() const {return getNumTimeBins(0);}
    /**
     * @brief Set the number of bins defining the size of the time
     *  discretization for the scanning device
     * @param numTimeBins The number of time bins for the scanning device
     * @param idx The index of the scanning device which number of time bins
     *  must be setted
     */
    virtual void setNumTimeBins(int const numTimeBins, size_t const idx) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setNumTimeBins(int const, size_t const) method
     * @see Scanner::setNumTimeBins(int const, size_t const)
     */
    inline void setNumTimeBins(int const numTimeBins)
    {setNumTimeBins(numTimeBins, 0);}
    /**
     * @brief Obtain the index of the bin containing the intensity peak for
     *  the scanning device
     * @param idx The index of the scanning device which peak intensity index
     *  must be obtained
     * @return The peak intensity index of the scanning device
     */
    virtual int getPeakIntensityIndex(size_t const idx) const = 0;
    /**
     * @brief Non index version of the
     *  Scanner::getPeakIntensityIndex(size_t const) method
     * @see Scanner::getPeakIntensityIndex(size_t const)
     */
    inline int getPeakIntensityIndex() const {return getPeakIntensityIndex(0);}
    /**
     * @brief Set the index of the bin containing the intensity peak for
     *  the scanning device
     * @param pii The new peak intensity index for the scanning device
     * @param idx The index of the scanning device which peak intensity
     *  index must be setted
     */
    virtual void setPeakIntensityIndex(int const pii, size_t const idx) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setPeakIntensityIndex(int const, size_t const) method
     * @see Scanner::setPeakIntensityIndex(int const, size_t const)
     */
    inline void setPeakIntensityIndex(int const pii)
    {setPeakIntensityIndex(pii, 0);}
    /**
     * @brief Obtain the time discretization vector of the scanning device
     * @param idx The index of the scanning device which time discretization
     *  vector must be obtained
     * @return The time discretization vector of the scanning device
     */
    virtual std::vector<double>& getTimeWave(size_t const idx) = 0;
    /**
     * @brief Non index version of the Scanner::getTimeWave(size_t const)
     *  method
     * @see Scanner::getTimeWave(size_t const)
     */
    inline std::vector<double>& getTimeWave() {return getTimeWave(0);}
    /**
     * @brief Set the time discretization vector of the scanning device
     * @param timewave The new time discretization vector for the scanning
     *  device
     * @param idx The index of scanning device which time discretization
     *  vector must be updated
     */
    virtual void setTimeWave(
        std::vector<double> &timewave, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setTimeWave(std::vector<double> &, size_t const) method
     * @see Scanner::setTimeWave(std::vector<double> &, size_t const)
     */
    inline void setTimeWave(std::vector<double> &timewave)
    {setTimeWave(timewave, 0);}
    /**
     * @brief Rvalue version of the
     *  Scanner::setTimeWave(std::vector<double> &, size_t const) method
     * @see Scanner::setTimeWave(std::vector<double> &, size_t const)
     */
    virtual void setTimeWave(
        std::vector<double> &&timewave, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setTimeWave(std::vector<double> &&, size_t const) method
     * @see Scanner::setTimeWave(std::vector<double> &&, size_t const)
     */
    inline void setTimeWave(std::vector<double> &&timewave)
    {setTimeWave(timewave, 0);}
    /**
     * @brief Obtain the minimum received energy threshold of the scanning
     *  device.
     * @param idx The index of the scanning device which minimum received
     *  energy threshold must be obtained.
     * @return The minimum received energy threshold of the scanning device.
     */
    virtual double getReceivedEnergyMin(size_t const idx) const = 0;
    /**
     * @brief Non index version of the
     * Scanner::getReceivedEnergyMin(size_t const) method.
     * @see Scanner::getReceivedEnergyMin(size_t const)
     */
    inline double getReceivedEnergyMin() const
    {return getReceivedEnergyMin(0);}
    /**
     * @brief Set the minimum threshold for the received energy of the
     *  scanning device.
     * @param receivedEnergyMin_J The new minimum threshold for the received
     *  energy of the scanning device.
     * @param idx The index of the scanning device which time discretization
     *  vector must be updated.
     */
    virtual void setReceivedEnergyMin(
        double const receivedEnergyMin_J, size_t const idx
    ) = 0;
    /**
     * @brief Non index version of the
     *  Scanner::setReceivedEnergyMin(double const, size_t const) method.
     * @see Scanner::setReceivedEnergyMin(double const, size_t const)
     */
    inline void setReceivedEnergyMin(double const receivedEnergyMin_J)
    {setReceivedEnergyMin(receivedEnergyMin_J, 0);}

#ifdef DATA_ANALYTICS
    /**
     * @brief Workaround to access the scanning pulse process when building
     *  HELIOS++ in DATA_ANALYTICS mode
     */
    inline ScanningPulseProcess * getScanningPulseProcess(){
        return spp.get();
    }
#endif

};