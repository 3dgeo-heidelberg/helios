#pragma once

#include <memory>

#include <Asset.h>
#include <ScannerHead.h>
#include <AbstractBeamDeflector.h>
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

#ifdef PYTHON_BINDING
#include <PyBeamDeflectorWrapper.h>
namespace pyhelios{ class PyDetectorWrapper;};
#include <PyIntegerList.h>
#include <PyNoiseSourceWrapper.h>
#include <PyRandomnessGeneratorWrapper.h>
#include <PyDoubleVector.h>
using pyhelios::PyBeamDeflectorWrapper;
using pyhelios::PyDetectorWrapper;
using pyhelios::PyIntegerList;
using pyhelios::PyNoiseSourceWrapper;
using pyhelios::PyRandomnessGeneratorWrapper;
using pyhelios::PyDoubleVector;
#endif



/**
 * @brief Class representing a scanner asset
 */
class Scanner : public Asset {
private:
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
     * @brief Number of rays computed by the calcRaysNumber function
     * @see Scanner::calcRaysNumber
     */
	int numRays = 0;
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
	 * @brief Current pulse number
	 */
	int state_currentPulseNumber = 0;
	/**
	 * @brief Flag specifying if last pulse was hit (true) or not (false)
	 */
	bool state_lastPulseWasHit = false;
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
     * @brief Scanner head composing the scanner
     * @see ScannerHead
     */
	std::shared_ptr<ScannerHead> scannerHead;
	/**
	 * @brief Beam deflector composing the scanner
	 * @see AbstractBeamDeflector
	 */
	std::shared_ptr<AbstractBeamDeflector> beamDeflector;
	/**
	 * @brief Platform carrying the scanner
	 * @see Platform
	 */
	std::shared_ptr<Platform> platform;
	/**
	 * @brief Detector composing the scanner
	 * @see AbstractDetector
	 */
	std::shared_ptr<AbstractDetector> detector;
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
	 * NOTICE that it is given in nanosconds, while the trajectoryTimeInterval
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

	// FWF settings
	/**
	 * @brief Full wave form settings for the scanner
	 * @see FWFSettings
	 */
	FWFSettings FWF_settings;

	// Pulse discretization
	/**
	 * @brief Number if bins defining discretization size
	 *
	 * The number of bins is computed considering full wave settings:
	 * \f[
	 *  \textrm{numTimeBins} = \frac{\textrm{pulseLength}}{binSize}
	 * \f]
	 */
	int numTimeBins = -1;
	/**
	 * @brief Index of bin containing the intensity peak. It is computed
	 * through calcTimePropagation function.
	 *
	 * @see Scanner::calcTimePropagation(vector<double> &, int)
	 */
    int peakIntensityIndex = -1;
    /**
     * @brief Discretization vector
     */
    std::vector<double> time_wave;

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

    // ########## BEGIN Emitter ###########
	/**
	 * @brief Pulse frequencies (hertz) supoported by the scanner
	 */
	std::list<int> cfg_device_supportedPulseFreqs_Hz;

    /**
     * @brief Maximum number of returns per pulse. When 0, it means there is
     *  not maximum at all
     * @see Scanner::checkMaxNOR
     */
    int maxNOR = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Scanner constructor
     * @see Scanner::cfg_device_beamDivergence_rad
     * @param beamOrigin Beam origin used to set head relative emitter position
     * @param beamOrientation Beam orientation used to set head relative
     *  emitter attitude
     * @param pulseFreqs List of supported pulse frequencies (hertz)
     * @see Scanner::cfg_device_supportedPulseFreqs_Hz
     * @see Scanner::cfg_device_pulseLength_ns
     * @see Scanner::cfg_device_id
     * @see Scanner::cfg_device_averagePower_w
     * @see Scanner::cfg_device_beamQuality
     * @see Scanner::cfg_device_efficiency
     * @see Scanner::cfg_device_receiverDiameter_m
     * @param atmosphericVisibility Atmospheric visibility understood as
     * the scanner visibility (kilometers)
     * @param wavelength Wavelength used to set scanner wave length after
     * dividing by \f$10^{9}\f$
     * @see Scanner::writeWaveform
     * @see Scanner::calcEchowidth
     * @see Scanner::fullWaveNoise
     * @see Scanner::platformNoiseDisabled
     */
	Scanner(
        std::string const id,
        std::list<int> const &pulseFreqs,
        bool const writeWaveform=false,
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

    // ***  M E T H O D S  *** //
    // *********************** //
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
     * @see ScannerSettings
     */
	void applySettings(std::shared_ptr<ScannerSettings> settings);
	/**
	 * @brief Retrieve current scanner settings and build a new ScannerSettings
	 *  object with them
	 * @return Newly created ScannerSettings object with current scanner
	 *  settings
	 */
	std::shared_ptr<ScannerSettings> retrieveCurrentSettings();
	/**
	 * @brief Apply full wave form settings
	 * @param settings Full wave form settings to be applied
	 * @see FWFSettings
	 */
	void applySettingsFWF(FWFSettings settings);
	/**
	 * @brief Perform computations for current simulation step
	 * @param legIndex Index of current leg
	 * @param currentGpsTime GPS time of current pulse
	 */
	void doSimStep(
	    unsigned int legIndex,
	    double currentGpsTime
    );
	/**
	 * @brief Build a string representation of the scanner
	 * @return String representing the scanner
	 */
	std::string toString();
	/**
	 * @brief Compute the number of rays depending on beam sample quality
	 */
    void calcRaysNumber();
    /**
     * @brief Prepare wave discretization
     * @see Scanner::numTimeBins
     * @see Scanner::time_wave
     * @see Scanner::peakIntensityIndex
     */
    void prepareDiscretization();
    /**
     * @brief Compute propagation time, which means obtaining the intensity
     * peak index
     * @see Scanner::numTimeBins
     * @see Scanner::time_wave
     * @see Scanner::peakIntensityIndex
     * @return Index of intensity peak
     */
    int calcTimePropagation(std::vector<double> & timeWave, int numBins);
    /**
     * @brief Compute the footprint area \f$f_{a}\f$
     *
     * \f[
     *  f_{a} = \frac{{\pi}d^{2}B_{t2}}{4}
     * \f]
     *
     * @param distance Distance \f$d\f$
     * @return Footprint area \f$f_{a}\f$
     * @see Scanner::cached_Bt2
     */
    double calcFootprintArea(double distance);
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
    double calcFootprintRadius(double distance);
    /**
     * @brief Compute the absolute beam attitude considering the mount relative
     * attitude and the deflector relative attitude
     * @see ScannerHead::getMountRelativeAttitude
     * @see AbstractBeamDeflector::getEmitterRelativeAttitude
     */
    Rotation calcAbsoluteBeamAttitude();

    /**
     * @brief Check if given number of return (nor) is inside
     *  expected boundaries.
     * If scanner maxNOR is 0 or nor is less than maxNOR, then the check
     *  is passed (true is returned). Otherwise, it is not passed (false is
     *  returned).
     * @param nor Current number of return
     * @return True if the check is passed, false otherwise
     * @see Scanner::maxNOR
     */
    inline bool checkMaxNOR(int nor) {return maxNOR==0 || nor < maxNOR;}

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
    inline void onLegComplete() {spp->onLegComplete();}
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
	 * @brief Obtain the current pulse number
	 * @return The current pulse number
	 * @see Scanner::state_currentPulseNumber
	 */
	inline int getCurrentPulseNumber() const {return state_currentPulseNumber;}
	/**
	 * @brief Obtain the number of rays
	 * @return Number of rays
	 * @see Scanner::numRays
	 */
	inline int getNumRays() {return this->numRays;}
	/**
	 * @brief Set the number of rays
	 * @param numRays New number of rays
	 * @see Scanner::numRays
	 */
	inline void setNumRays(int numRays) {this->numRays = numRays;}

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
	virtual double getPulseLength_ns(size_t const idx=0) const = 0;
	/**
     * @brief Set the pulse length
     * @param pulseLength_ns New pulse length (nanoseconds)
     * @param idx The index of the scanning device which pulse length must
     *  be setted (by default 1, it is the first one)
	 * @see ScanningDevice::cfg_device_pulseLength_ns
     */
	virtual void setPulseLength_ns(
	    double const pulseLength_ns, size_t const idx=0
    ) = 0;

	/**
	 * @brief Check if last pulse was hit (true) or not (false)
	 * @return True if last pulse was hit, false otherwise
	 * @see Scanner::state_lastPulseWasHit
	 */
	inline bool lastPulseWasHit() const {return this->state_lastPulseWasHit;}
	/**
	 * @brief Specify if last pulse was hit (true) or not (false)
	 * @param lastPulseWasHit New last pulse hit specification
	 * @see Scanner::state_lastPulseWasHit
	 */
    void setLastPulseWasHit(bool lastPulseWasHit);

    /**
     * @brief Obtain beam divergence
     * @param idx The index of the scanning device which beam divergence must
     *  be obtained (by default 0, it is the first one)
     * @return Beam divergence (radians)
     * @see ScanningDevice::beamDivergence_rad
     */
	virtual double getBeamDivergence(size_t const idx=0) const = 0;
	/**
	 * @brief Set beam divergence
	 * @param beamDivergence New beam divergence (radians)
	 * @param idx The index of the scanning device which beam divergence must
	 *  be setted (by default 0, it is the first one)
     * @see ScanningDevice::beamDivergence_rad
	 */
	virtual void setBeamDivergence(
	    double const beamDivergence, size_t const idx=0
    ) = 0;

    /**
     * @brief Obtain average power
     * @param idx The index of the scanning device which average power must be
     *  obtained (by default 0, it is the first one)
     * @return Average power (watts)
     * @see ScanningDevice::averagePower_w
     */
	virtual double getAveragePower(size_t const idx=0) const = 0;
	/**
	 * @brief Set average power
	 * @param averagePower New average power (watts)
	 * @param idx The index of the scanning device which average power must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::averagePower_w
	 */
	virtual void setAveragePower(
	    double const averagePower, size_t const idx=0
    ) = 0;

    /**
     * @brief Get beam quality
     * @param idx The index of the scanning device which beam quality must be
     *  obtained (by default 0, it is the first one)
     * @return Beam quality
     * @see ScanningDevice::beamQuality
     */
	virtual double getBeamQuality(size_t const idx=0) const = 0;
	/**
	 * @brief Set beam quality
	 * @param beamQuality New beam quality
     * @param idx The index of the scanning device which beam quality must be
     *  obtained (by default 0, it is the first one)
     * @see ScanningDevice::beamQuality
	 */
	virtual void setBeamQuality(
	    double const beamQuality, size_t const idx=0
    ) = 0;

    /**
     * @brief Obtain device efficiency
     * @param idx The index of the scanning device which efficiency must be
     *  obtained (by default 0, it is the first one)
     * @return Device efficiency
     * @see ScanningDevice::efficiency
     */
	virtual double getEfficiency(size_t const idx = 0) const = 0;
	/**
	 * @brief Set device efficiency
	 * @param efficiency New device efficiency
	 * @param idx The index of the scanning device which efficiency must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::efficiency
	 */
	virtual void setEfficiency(double const efficiency, size_t const idx=0)=0;

	/**
	 * @brief Get receiver diameter
	 * @param idx The index of the scanning device which efficiency must be
	 *  obtained (by default 0, it is the first one)
	 * @return Receiver diameter
	 * @see ScanningDevice::receiverDiameter_m
	 */
	virtual double getReceiverDiameter(size_t const idx = 0) const = 0;
	/**
	 * @brief Set receiver diameter
	 * @param receiverDiameter  New receiver diameter
	 * @param idx The index of the scanning device which efficiency must be
	 *  setted (by default 0, it is the first one)
	 * @see ScanningDevice::receiverDiameter_m
	 */
	virtual void setReceiverDiameter(
	    double const receiverDiameter, size_t const idx = 0
    ) = 0;

    /**
     * @brief Get device visibility
     * @param idx The index of the scanning device which visibility must be
     *  obtained (by default 0, it is the first one)
     * @return Device visibility (kilometers)
     * @see ScanningDevice::visibility_km
     */
	virtual double getVisibility(size_t const idx=0) const = 0;
	/**
	 * @brief Set device visibility
	 * @param visibility New device visibility (kilometers)
	 * @param idx The index of the scanning device which visibility must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::visibility_km
	 */
	virtual void setVisibility(double const visibility, size_t const idx=0)=0;

    /**
     * @brief Obtain wave length
     * @param idx The index of the scanning device which wavelength must be
     *  obtained (by default 0, it is the first one)
     * @return Wave length (meters)
     * @see ScanningDevice::wavelength_m
     */
	virtual double getWavelength(size_t const idx=0) const = 0;
	/**
	 * @brief Set wave length
	 * @param wavelength New wave length (meters)
	 * @param idx The index of the scanning device which wavelength must be
	 *  setted (by default 0, it is the first one)
     * @see ScanningDevice::wavelength_m
	 */
	virtual void setWavelength(double const wavelength, size_t const idx=0)=0;

	/**
	 * @brief Obtain atmospheric extinction
	 * @param idx The index of the scanning device which atmospheric extinction
	 *  must be obtained (by default 0, it is the first one)
	 * @return Atmospheric extinction
	 * @see ScanningDevice::atmosphericExtinction
	 */
	virtual double getAtmosphericExtinction(size_t const idx=0) const = 0;
	/**
     * @brief Set atmospheric extinction
     * @param atmosphericExtinction New atmospheric extinction
	 * @param idx The index of the scanning device which atmospheric extinction
	 *  must be setted (by default 0, it is the first one)
	 * @see ScanningDevice::atmosphericExtinction
     */
	virtual void setAtmosphericExtinction(
	    double const atmosphericExtinction,
	    size_t const idx=0
    ) = 0;

    /**
     * @brief Obtain beam waist radius
     * @param idx The index of the scanning device which beam waist radius must
     *  be obtained (by default 0, it is the first one)
     * @return Beam waist radius
     * @see ScanningDevice::beamWaistRadius
     */
	virtual double getBeamWaistRadius(size_t const idx=0) const = 0;
	/**
	 * @brief Set beam waist radius
	 * @param beamWaistRadius New beam waist radius
	 * @param idx The index of the scanning device which beam waist radius must
	 *  be setted (by default 0, it is the first one)
     * @see ScanningDevice::beamWaistRadius
	 */
	virtual void setBeamWaistRadius(
	    double const beamWaistRadius, size_t const idx=0
    ) = 0;
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
     * @brief Obtain \f$B_{t2}\f$
     * @param idx The index of the scanning device which cached Bt2 (square of
     *  beam divergence) must be obtained (by default 0, it is the first one)
     * @return \f$B_{t2}\f$
     * @see ScanningDevice::cached_Bt2
     */
	virtual double getBt2(size_t const idx) const = 0;
	/**
	 * @brief Set \f$B_{t2}\f$
	 * @param bt2 New \f$B_{t2}\f$
     * @param idx The index of the scanning device which cached Bt2 (square of
     *  beam divergence) must be setted (by default 0, it is the first one)
     * @see ScanningDevice::cached_Bt2
	 */
	virtual void setBt2(double const bt2, size_t const idx=0) = 0;

	/**
	 * @brief Obtain \f$D_{r2}\f$
	 * @param idx The index of the scanning device which cached Dr2 (square of
	 *  receiver diameter) must be obtained (by default 0, it is the first one)
	 * @return \f$D_{r2}\f$
	 * @see ScanningDevice::cached_Dr2
	 */
	virtual double getDr2(size_t const idx=0) const = 0;
	/**
	 * @brief Set \f$D_{r2}\f$
	 * @param dr2 New \f$D_{t2}\f$
	 * @param idx The index of the scanning device which cached Dr2 (square of
	 *  receiver diameter) must be obtained (by default 0, it is the first one)
	 * @see ScanningDevice::cached_Dr2
	 */
	virtual void setDr2(double const dr2, size_t const idx=0) = 0;

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
	 * @brief Obtain scanner device identifier
	 * @return Scanner device identifier
	 * @see ScanningDevice::id
	 */
	virtual std::string getDeviceId(size_t const idx = 0) const = 0;
	/**
	 * @brief Set the scanner device identifier
	 * @param deviceId New scanner device identifier
	 * @see ScanningDevice::id
	 */
	virtual void setDeviceId(std::string const deviceId, size_t const idx=0)=0;
	/**
	 * @brief Obtain the number of scanning devices composing the scanner
	 */
	virtual size_t getNumDevices() const = 0;

#ifdef PYTHON_BINDING
    /**
     * @brief Python wrapper for scanner head access
     * @return Reference to scanner head
     * @see Scanner::scannerHead
     */
    ScannerHead & getScannerHead(){return *scannerHead;}
    /**
     * @brief Python wrapper for beam deflector access
     * @return Wrapped beam deflector
     * @see Scanner::beamDeflector
     */
    PyBeamDeflectorWrapper * getPyBeamDeflector()
        {return new PyBeamDeflectorWrapper(beamDeflector);}
    /**
     * @brief Python wrapper for detector access
     * @return Wrapped detector
     * @see Scanner::detector
     */
    PyDetectorWrapper * getPyDetectorWrapper();
    /**
     * @brief Python wrapper for supported pulse frequencies list
     * @return Wrapped list of supported pulse frequencies
     * @see Scanner::cfg_device_supportedPulseFreqs_Hz
     */
    PyIntegerList * getSupportedPulseFrequencies()
        {return new PyIntegerList(cfg_device_supportedPulseFreqs_Hz);}
    /**
     * @brief Python wrapper for head relative emitter attitude access
     * @param idx The index of the scanning device which relative attitude
     *  must be obtained by reference (by default 0, it is the first one)
     * @return Reference to head relative emitter attitude
     * @see Scanner::cfg_device_headRelativeEmitterAttitude
     */
    virtual Rotation & getRelativeAttitudeByReference(
        size_t const idx=0
    ) const = 0;
    /**
     * @brief Python wrapper for head relative emitter position
     * @param idx The index of the scanning device which relative position
     *  must be obtained (by default 0, it is the first one)
     * @return Wrapped head relative emitter position
     * @see Scanner::cfg_device_headRelativeEmitterPosition
     */
    virtual PythonDVec3 * getRelativePosition(size_t const idx=0) const = 0;
    /**
     * @brief Python wrapper for intersection handling noise source
     * @return Wrapped intersection handling noise source
     * @see Scanner::intersectionHandlingNoiseSource
     */
	PyNoiseSourceWrapper * getIntersectionHandlingNoiseSource(){
        if(intersectionHandlingNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*intersectionHandlingNoiseSource);
    }
    /**
     * @brief Python wrapper for first randomness generator
     * @return Wrapped first randomness generator
     * @see Scanner::randGen1
     */
    PyRandomnessGeneratorWrapper * getRandGen1(){
        if(randGen1 == nullptr) return nullptr;
        return new PyRandomnessGeneratorWrapper(*randGen1);
    }
    /**
     * @brief Python wrapper for second randomness generator
     * @return Wrapped second randomness generator
     * @see Scanner::randGen2
     */
    PyRandomnessGeneratorWrapper * getRandGen2(){
        if(randGen2 == nullptr) return nullptr;
        return new PyRandomnessGeneratorWrapper(*randGen2);
    }
    /**
     * @brief Python wrapper for time wave vector
     * @return Wrapped time wave vector
     * @see Scanner::time_wave
     */
    PyDoubleVector * getTimeWave(){
        return new PyDoubleVector(time_wave);
    }
#endif
};