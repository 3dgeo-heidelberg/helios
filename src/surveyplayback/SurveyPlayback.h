#pragma once

#include <memory>
#include <string>

#include "Simulation.h"
#include "Survey.h"
#include "typedef.h"

/**
 * @brief Survey playback class, used to extend simulation functionalities
 *  so it can be controlled
 * @see Simulation
 */
class SurveyPlayback : public Simulation {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Flag to specify if leg has been started (true) or not (false)
     */
	bool mLegStarted = false;
	/**
	 * @brief Flag to specify if LAS format must be used for the output (true)
	 *  or not (false)
	 */
	bool lasOutput = false;
	/**
	 * @brief Flag to specify if output must be zipped (true) or not (false)
	 */
	bool zipOutput = false;

	/**
	 * @brief The survey itself
	 * @see Survey
	 */
	std::shared_ptr<Survey> mSurvey;

	/**
	 * @brief Path to output file
	 */
	std::string mOutputFilePathString = "";
	/**
	 * @brief Output format string. It is not used at the moment and might be
	 *  removed in the future.
	 *
	 * @deprecated mFormatString can be considered as deprecated
	 */
	std::string mFormatString = "%03d";

private:
    /**
     * @brief Root output path
     */
    const std::string outputPath;

    /**
     * @brief Number of effective legs
     */
	int numEffectiveLegs = 0;	// = -1 leg if survey !onGround
	/**
	 * @brief Currently elapsed length. It can be understood as the summation
	 *  of all traveled legs
	 */
	double elapsedLength = 0;	// Sum of legs length traveled
	/**
	 * @brief Survey simulation progress tracking
	 */
	double progress = 0;
	/**
	 * @brief Progress tracking for current leg
	 */
	double legProgress = 0;
	/**
	 * @brief Time (nanoseconds) when the leg started
	 */
	long legStartTime_ns = 0;
	/**
	 * @brief Elapsed time (milliseconds) since survey simulation started
	 */
	long elapsedTime_ms = 0;
	/**
	 * @brief Expected remaining time (milliseconds) for survey simulation
	 */
	long remainingTime_ms = 0;
    /**
     * @brief Elapsed time (milliseconds) since current leg started
     */
	long legElapsedTime_ms = 0;
	/**
	 * @brief Expected remaining time (milliseconds) for current leg completion
	 */
	long legRemainingTime_ms = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Survey playback constructor
     * @param survey The survey itself
     * @param outputPath Root output path
     * @param numThreads Number of threads to be used
     * @param lasOutput Flag to specify LAS format for the output (true) or not
     *  (false)
     * @param zipOutput Flag to specify if output must be zipped (true) or not
     *  (false)
     * @param exportToFile Flag to specify if output must be written to a file
     *  (true) or not (false)
     * @see SurveyPlayback::lasOutput
     * @see SurveyPlayback::zipOutput
     * @see SurveyPlayback::outputPath
     * @see Survey
     */
	SurveyPlayback(
        std::shared_ptr<Survey> survey,
        const std::string outputPath,
        size_t numThreads,
        bool lasOutput,
        bool zipOutput,
        bool exportToFile=true
    );


    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Time estimation for the entire simulation and current leg.
     *  NOTICE this function is called from trackProgress
     * @param legCurrentProgress Current leg progress
     * @param onGround Not used at the moment
     * @param legElapsedLength Elapsed length for current leg
     * @see SurveyPlayback::trackProgress
     */
	void estimateTime(
	    int legCurrentProgress,
	    bool onGround,
	    double legElapsedLength
    );
	/**
	 * @brief Progress tracking and time estimation
	 * @see SurveyPlayback::estimateTime
	 */
	void trackProgress();
	/**
	 * @brief Perform computations for current simulation step
	 */
	virtual void doSimStep();
	/**
	 * @brief Handle leg completion
	 */
    void onLegComplete();
    /**
     * @brief Start specified leg
     * @param legIndex Index of leg to start
     * @param manual Specify if leg initialization must be manual (true) or
     *  not (false)
     * @see Platform::initLeg
     * @see Platform::initLegManual
     */
    void startLeg(unsigned int legIndex, bool manual);
    /**
     * @brief Prepare output for current leg (measurements, trajectory and
     *  fullwave)
     * @see SyncFileWriter
     */
    void prepareOutput();
    /**
     * @brief Clear point cloud file for current leg
     */
    void clearPointcloudFile();
    /**
     * @brief Start next leg
     * @param manual Specify if manual leg initialization must be used (true)
     *  or not (false)
     * @see Platform::initLeg
     * @see Platform::initLegManual
     */
    void startNextLeg(bool manual);
    /**
     * @brief Handle survey playback shutdown
     * @see Simulation::shutdown
     * @see Scanner::AbstractDetector
     */
    void shutdown();
    /**
     * @brief Translate milliseconds to time stamp string
     *
     * @param millis
     * @return Time stamp string corresponding to given milliseconds. Its
     *  format is "DD HH:MM:SS"
     */
    std::string milliToString(long millis);
    /**
     * @brief Perform stop and turn operation to advance to next leg
     *
     * Notice this operation is only supported for HelicopterPlatform.
     *  Trying to use it with other platforms leads to undefined behaviors
     *  and should be avoided.
     *
     * @param legIndex Index of current leg
     * @param leg Current leg
     * @see Platform::stopAndTurn
     * @see HelicopterPlatform
     */
    void stopAndTurn(unsigned int legIndex, std::shared_ptr<Leg> leg);


    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain current leg
     * @return Current leg
     * @see Leg
     */
	std::shared_ptr<Leg> getCurrentLeg();
	/**
	 * @brief Obtain current leg index
	 * @return Current leg index
	 */
	int getCurrentLegIndex();
	/**
	 * @brief Obtain current leg output prefix
	 * @return Current leg output prefix
	 */
	std::string getLegOutputPrefix();
	/**
	 * @brief Obtain current output path
	 * @return Current output path
	 */
	std::string getCurrentOutputPath();
	/**
	 * @brief Obtain current trajectory output path
	 * @return Current trajectory output path
	 */
	std::string getTrajectoryOutputPath();

	/**
	 * @brief Obtain simulation progress
	 * @return Simulation progress
	 */
	double getProgress() {return this->progress;}

	/**
	 * @brief Obtain current leg progress
	 * @return Current leg progress
	 */
	double getLegProgress() {return this->legProgress;}

	/**
	 * @brief Obtain the number of effective legs
	 * @return Number of effective legs
	 */
	int getNumEffectiveLegs() {return this->numEffectiveLegs;}

	/**
	 * @brief Obtain elapsed time
	 * @return Elapsed time (milliseconds)
	 */
	long long getElapsedTime() {return this->elapsedTime_ms;}

	/**
	 * @brief Obtain expected remaining time
	 * @return Expected remaining time (milliseconds)
	 */
	long getRemainingTime() {return this->remainingTime_ms;}

	/**
	 * @brief Obtain current leg elapsed time
	 * @return Current leg elapsed time (milliseconds)
	 */
	long long getLegElapsedTime() {return this->legElapsedTime_ms;}
	/**
	 * @brief Obtain current leg expected remaining time
	 * @return Current leg expected remaining time (milliseconds)
	 */
	long getLegRemainingTime() {return this->legRemainingTime_ms;}
};