#pragma once

#include <memory>

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include "Asset.h"
#include "ScannerHead.h"
#include "AbstractBeamDeflector.h"
class AbstractDetector;
#include "FWFSettings.h"
#include "Platform.h"
#include "maths/Directions.h"
#include "maths/Rotation.h"
#include "ThreadPool.h"
#include "SyncFileWriter.h"
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
#include <Measurement.h>


/**
 * @brief Class representing a scanner asset
 */
class Scanner : public Asset {
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
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
     * @brief Beam divergence (radians)
     */
	double cfg_device_beamDivergence_rad = 0;
	/**
	 * @brief Pulse length (nanoseconds)
	 */
	double cfg_device_pulseLength_ns = 0;
	/**
	 * @brief Pulse frequency (hertz)
	 */
	int cfg_setting_pulseFreq_Hz = 0;
	/**
	 * @brief Device identifier
	 */
	std::string cfg_device_id = "";
	/**
	 * @brief Average power (watts)
	 */
	double cfg_device_averagePower_w;
	/**
	 * @brief Beam quality
	 */
	double cfg_device_beamQuality;
	/**
	 * @brief Device efficiency
	 */
	double cfg_device_efficiency;
	/**
	 * @brief Receiver diamater (meters)
	 */
	double cfg_device_receiverDiameter_m;
	/**
	 * @brief Visibility (kilometers)
	 */
	double cfg_device_visibility_km;
	/**
	 * @brief Wave length (meters)
	 */
	double cfg_device_wavelength_m;

	/**
	 * @brief Atmospheric extinction
	 */
	double atmosphericExtinction;
	/**
	 * @brief Beam waist radius
	 */
	double beamWaistRadius;
	// ########## END Emitter ###########

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

	// Cached variables
	/**
	 * @brief \f$D_{r2}\f$ understood as the square of receiver diameter
	 *
	 * \f[
	 *  D_{r2} = \textrm{receiverDiamater}^{2}
	 * \f]
	 *
	 * @see Scanner::cfg_device_receiverDiameter_m
	 */
	double cached_Dr2;
	/**
	 * @brief \f$B_{t2}\f$ understood as the square of beam divergence
	 *
	 * \f[
	 *  B_{t2} = \textrm{beamDivergence}^{2}
	 * \f]
	 *
	 * @see Scanner::cfg_device_beamDivergence_rad
	 */
	double cached_Bt2;

	// Trajectory writer
	/**
	 * @brief Synchronous file writer
	 */
	std::shared_ptr<SyncFileWriter> tfw = nullptr;

public:
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
	 * exactly 0, then no trajectory points will be recorded
	 */
    double trajectoryTimeInterval = 0.0;
    /**
     * @brief GPS time (milliseconds) corresponding to last recorded trajectory
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
     * @brief Head relative emitter position
     */
	glm::dvec3 cfg_device_headRelativeEmitterPosition = glm::dvec3(0, 0, 0);
	/**
	 * @brief Head relative emitter attitude
	 */
	Rotation cfg_device_headRelativeEmitterAttitude =
	    Rotation(Directions::right, 0);
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
	    double beamDiv_rad,
	    glm::dvec3 beamOrigin,
	    Rotation beamOrientation,
	    std::list<int> pulseFreqs,
	    double pulseLength_ns,
	    std::string id,
	    double averagePower,
	    double beamQuality,
	    double efficiency,
	    double receiverDiameter,
	    double atmosphericVisibility,
	    int wavelength,
	    bool writeWaveform = false,
	    bool calcEchowidth = false,
	    bool fullWaveNoise = false,
	    bool platformNoiseDisabled = false
    );
    Scanner(Scanner &scanner);

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Apply scanner settings
     * @param settings Scanner settings to be applied
     * @see ScannerSettings
     */
	void applySettings(std::shared_ptr<ScannerSettings> settings);
	/**
	 * @brief Apply full wave form settings
	 * @param settings Full wave form settings to be applied
	 * @see FWFSettings
	 */
	void applySettingsFWF(FWFSettings settings);
	/**
	 * @brief Perform computations for current simulation step
	 * @param pool Thread pool used to handle concurrent computations
	 * @param legIndex Index of current leg
	 * @param currentGpsTime GPS time of current pulse
	 */
	void doSimStep(thread_pool& pool, unsigned int legIndex, double currentGpsTime);
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
     * @brief Compute the atmospheric attenuation to be used as the
     * atmospheric attenuation
     * @return Atmospheric attenuation
     * @see Scanner::atmosphericExtinction
     */
    double calcAtmosphericAttenuation();
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
     * @return True of the check is passed, false otherwise
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
     * @brief Handle pulse computation whatever it is single thread based
     * or thread pool based
     * @param pool Thread pool to be used to handle multi threading pulse
     * computation
     * @param legIndex Index of current leg
     * @param absoluteBeamOrigin Absolute position of beam origin
     * @param absoluteBeamAttitude Beam attitude
     * @param currentGpsTime Current GPS time (milliseconds)
     */
    void handlePulseComputation(
        thread_pool& pool,
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double currentGpsTime
    );
    /**
     * @brief Handle trajectory output whatever it is to output file, to
     * all trajectories vector or to cycle trajectories vector
     * @param currentGpsTime Current GPS time (milliseconds)
     * @see Scanner::allTrajectories
     * @see Scanner::cycleTrajectories
     */
    void handleTrajectoryOutput(double currentGpsTime);

	// *** GETTERs and SETTERs *** //
	// *************************** //
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
	inline int getPulseFreq_Hz() {return this->cfg_setting_pulseFreq_Hz;}
    /**
     * @brief Set the pulse frequency
     * @param pulseFreq_Hz New pulse frequency (hertz)
     * @see Scanner::cfg_setting_pulseFreq_Hz
     */
	void setPulseFreq_Hz(int pulseFreq_Hz);

	/**
	 * @brief Get the pulse length
	 * @return Pulse length (nanoseconds)
	 * @see Scanner::cfg_device_pulseLength_ns
	 */
	inline double getPulseLength_ns()
	    {return this->cfg_device_pulseLength_ns;}
    /**
     * @brief Set the pulse length
     * @param pulseLength_ns New pulse length (nanoseconds)
	 * @see Scanner::cfg_device_pulseLength_ns
     */
	inline void setPulseLength_ns(double pulseLength_ns)
	    {this->cfg_device_pulseLength_ns = pulseLength_ns;}

	/**
	 * @brief Check if last pulse was hit (true) or not (false)
	 * @return True if last pulse was hit, false otherwise
	 * @see Scanner::state_lastPulseWasHit
	 */
	inline bool lastPulseWasHit() {return this->state_lastPulseWasHit;}
	/**
	 * @brief Specify if last pulse was hit (true) or not (false)
	 * @param lastPulseWasHit New last pulse hit specification
	 * @see Scanner::state_lastPulseWasHit
	 */
    void setLastPulseWasHit(bool lastPulseWasHit);

    /**
     * @brief Obtain beam divergence
     * @return Beam divergence (radians)
     * @see Scanner::cfg_device_beamDivergence_rad
     */
	inline double getBeamDivergence()
	    {return this->cfg_device_beamDivergence_rad;}
	/**
	 * @brief Set beam divergence
	 * @param beamDivergence New beam divergence (radians)
     * @see Scanner::cfg_device_beamDivergence_rad
	 */
	inline void setBeamDivergence(double beamDivergence)
	    {this->cfg_device_beamDivergence_rad = beamDivergence;}

    /**
     * @brief Obtain average power
     * @return Average power (watts)
     * @see Scanner::cfg_device_averagePower_w
     */
	inline double getAveragePower() {return this->cfg_device_averagePower_w;}
	/**
	 * @brief Set average power
	 * @param averagePower New average power (watts)
	 * @see Scanner::cfg_device_averagePower_w
	 */
	inline void setAveragePower(double averagePower)
	    {this->cfg_device_averagePower_w = averagePower;}

    /**
     * @brief Get beam quality
     * @return Beam quality
     * @see Scanner::cfg_device_beamQuality
     */
	inline double getBeamQuality() {return this->cfg_device_beamQuality;}
	/**
	 * @brief Set beam quality
	 * @param beamQuality New beam quality
     * @see Scanner::cfg_device_beamQuality
	 */
	inline void setBeamQuality(double beamQuality)
	    {this->cfg_device_beamQuality = beamQuality;}

    /**
     * @brief Obtain device efficiency
     * @return Device efficiency
     * @see Scanner::cfg_device_efficiency
     */
	inline double getEfficiency() {return this->cfg_device_efficiency;}
	/**
	 * @brief Set device efficiency
	 * @param efficiency New device efficiency
	 * @see Scanner::cfg_device_efficiency
	 */
	inline void setEfficiency(double efficiency)
	    {this->cfg_device_efficiency = efficiency;}

	/**
	 * @brief Get receiver diamater
	 * @return Receiver diamater
	 * @see Scanner::cfg_device_receiverDiameter_m
	 */
	inline double getReceiverDiameter()
	    {return this->cfg_device_receiverDiameter_m;}
	/**
	 * @brief Set receiver diameter
	 * @param receiverDiameter  New receiver diameter
	 * @see Scanner::cfg_device_receiverDiameter_m
	 */
	inline void setReceiverDiameter(double receiverDiameter)
        {this->cfg_device_receiverDiameter_m = receiverDiameter;}

    /**
     * @brief Get device visibility
     * @return Device visibility (kilometers)
     * @see Scanner::cfg_device_visibility_km
     */
	inline double getVisibility() {return this->cfg_device_visibility_km;}
	/**
	 * @brief Set device visibility
	 * @param visibility New device visibility (kilometers)
     * @see Scanner::cfg_device_visibility_km
	 */
	inline void setVisibility(double visibility)
        {this->cfg_device_visibility_km = visibility;}

    /**
     * @brief Obtain wave length
     * @return Wave length (meters)
     * @see Scanner::cfg_device_wavelength_m
     */
	inline double getWavelength() {return this->cfg_device_wavelength_m;}
	/**
	 * @brief Set wave length
	 * @param wavelength New wave length (meters)
     * @see Scanner::cfg_device_wavelength_m
	 */
	inline void setWavelength(double wavelength)
	    {this->cfg_device_wavelength_m = wavelength;}

	/**
	 * @brief Obtain atmospheric extinction
	 * @return Atmospheric extinction
	 * @see Scanner::atmosphericExtinction
	 */
	inline double getAtmosphericExtinction()
	    {return this->atmosphericExtinction;}
    /**
     * @brief Set atmospheric extinction
     * @param atmosphericExtinction New atmospheric extinction
	 * @see Scanner::atmosphericExtinction
     */
	inline void setAtmosphericExtinction(double atmosphericExtinction)
        {this->atmosphericExtinction = atmosphericExtinction;}

    /**
     * @brief Obtain beam waist radius
     * @return Beam waist radius
     * @see Scanner::beamWaistRadius
     */
	inline double getBeamWaistRadius() {return this->beamWaistRadius;}
	/**
	 * @brief Set beam waist radius
	 * @param beamWaistRadius New beam waist radius
	 * @see Scanner::beamWaistRadius
	 */
	inline void setBeamWaistRadius(double beamWaistRadius)
	    {this->beamWaistRadius = beamWaistRadius;}

    /**
     * @brief Obtain \f$B_{t2}\f$
     * @return \f$B_{t2}\f$
     * @see Scanner::cached_Bt2
     */
	inline double getBt2() {return this->cached_Bt2;}
	/**
	 * @brief Set \f$B_{t2}\f$
	 * @param bt2 New \f$B_{t2}\f$
     * @see Scanner::cached_Bt2
	 */
	inline void setBt2(double bt2) {this->cached_Bt2 = bt2;}

	/**
	 * @brief Obtain \f$D_{r2}\f$
	 * @return \f$D_{r2}\f$
	 * @see Scanner::cached_Dr2
	 */
	inline double getDr2() {return this->cached_Dr2;}
	/**
	 * @brief Set \f$D_{r2}\f$
	 * @param dr2 New \f$D_{t2}\f$
	 * @see Scanner::cached_Dr2
	 */
	inline void setDr2(double dr2) {this->cached_Dr2 = dr2;}

	/**
	 * @brief Check if scanner is active (true) or not (false)
	 * @return True if scanner is active, false otherwise
	 * @see Scanner::state_isActive
	 */
	inline bool isActive() {return this->state_isActive;}
	/**
	 * @brief Set scanner active status. True to make it active, false to
	 * make it inactive
	 * @param active New scanner active status
	 * @see Scanner::state_isActive
	 */
	inline void setActive(bool active) {this->state_isActive = active;}

	/**
	 * @brief Check if scanner is configured to write wave form (true) or not
	 *  (false)
	 * @return True if scanner is configured to write wave form, false
	 *  otherwise
	 * @see Scanner::writeWaveform
	 */
    inline bool isWriteWaveform() {return this->writeWaveform;}
    /**
     * @brief Set scanner write wave form configuration.
     * @param writeWaveform True to make scanner write wave form, false
     *  otherwise
     * @see Scanner::writeWaveform
     */
    inline void setWriteWaveform(bool writeWaveform)
        {this->writeWaveform = writeWaveform;}
    /**
     * @brief Check if scanner is configured to compute echo width (true) or
     *  not (false)
     * @return True if scanner is configured to compute echo width, false
     *  otherwise
     * @see Scanner::calcEchowidth
     */
    inline bool isCalcEchowidth() {return this->calcEchowidth;}
    /**
     * @brief Set scanner echo width configuration.
     * @param calcEchowidth True to make scanner compute echo width,
     *  false otherwise
     * @see Scanner::calcEchowidth
     */
    inline void setCalcEchowidth(bool calcEchowidth)
        {this->calcEchowidth = calcEchowidth;}

    /**
     * @brief Check if scanner is configured to add noise to full wave (true)
     *  or not (false)
     * @return True if scanner is configured to add noise to full wave, false
     *  otherwise
     * @see Scanner::fullWaveNoise
     */
	inline bool isFullWaveNoise() {return this->fullWaveNoise;}
	/**
	 * @brief Set scanner full wave noise policy
	 * @param fullWaveNoise True to make scanner add noise to the full wave,
	 *  false otherwise
     * @see Scanner::fullWaveNoise
	 */
	inline void setFullWaveNoise(bool fullWaveNoise)
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
	inline void setPlatformNoiseDisabled(bool platformNoiseDisabled)
        {this->platformNoiseDisabled = platformNoiseDisabled;}

    /**
     * @brief Check if incidence angle is fixed (true) or not (false)
     * @return True if incidence angle is fixed, false otherwise
     * @see Scanner::fixedIncidenceAngle
     */
    inline bool isFixedIncidenceAngle() {return this->fixedIncidenceAngle;}
    /**
     * @brief Set fixed incidence angle flag
     * @param fixedIncidenceAngle True to enable fixed incidence angle, false
     *  to disable it
     * @see Scanner::fixedIncidenceAngle
     */
    inline void setFixedIncidenceAngle(bool fixedIncidenceAngle)
        {this->fixedIncidenceAngle = fixedIncidenceAngle;}

    /**
     * @brief Set synchronous file writer for trajectory
     * @param tfw Synchronous file writer to be used to write trajectory
     * @see Scanner::tfw
     */
    inline void setTrajectoryFileWriter(std::shared_ptr<SyncFileWriter> tfw){
        this->tfw = tfw;
	}

	/**
	 * @brief Obtain scanner device identifier
	 * @return Scanner device identifier
	 * @see Scanner::cfg_device_id
	 */
	inline std::string getDeviceId() {return this->cfg_device_id;}
	/**
	 * @brief Set the scanner device identifier
	 * @param deviceId New scanner device identifier
	 * @see Scanner::cfg_device_id
	 */
	inline void setDeviceId(std::string const deviceId)
        {this->cfg_device_id = deviceId;}

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
     * @return Reference to head relative emitter attitude
     * @see Scanner::cfg_device_headRelativeEmitterAttitude
     */
    Rotation & getRelativeAttitudeByReference()
        {return cfg_device_headRelativeEmitterAttitude;}
    /**
     * @brief Python wrapper for head relative emitter position
     * @return Wrapped head relative emitter position
     * @see Scanner::cfg_device_headRelativeEmitterPosition
     */
    PythonDVec3 * getRelativePosition()
        {return new PythonDVec3(&cfg_device_headRelativeEmitterPosition);}
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