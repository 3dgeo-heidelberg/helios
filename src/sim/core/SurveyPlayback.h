#pragma once

#include <memory>
#include <string>

#include "Survey.h"
#include "typedef.h"
#include <Simulation.h>

namespace helios {
namespace filems {
class FMSFacade;
}
}

/**
 * @brief Survey playback class, used to extend simulation functionalities
 *  so it can be controlled
 * @see Simulation
 */
class SurveyPlayback : public Simulation
{
private:
  friend class SimulationPlayer;

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Flag to specify if leg has been started (true) or not (false)
   */
  bool mLegStarted = false;

  /**
   * @brief The survey itself
   * @see Survey
   */
  std::shared_ptr<Survey> mSurvey;
  /**
   * @brief Main facade to file management system
   */
  std::shared_ptr<helios::filems::FMSFacade> fms = nullptr;

private:
  /**
   * @brief Number of effective legs
   */
  int numEffectiveLegs = 0; // = -1 leg if survey !onGround
  /**
   * @brief Currently elapsed length. It can be understood as the summation
   *  of all traveled legs
   */
  double elapsedLength = 0; // Sum of legs length traveled
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
  std::chrono::nanoseconds legStartTime_ns;
  /**
   * @brief Elapsed time (nanoseconds) since survey simulation started
   */
  std::chrono::nanoseconds elapsedTime_ns;
  /**
   * @brief Expected remaining time (nanoseconds) for survey simulation
   */
  long long remainingTime_ns;
  /**
   * @brief Elapsed time (nanoseconds) since current leg started
   */
  std::chrono::nanoseconds legElapsedTime_ns;
  /**
   * @brief Expected remaining time (nanoseconds) for current leg completion
   */
  long long legRemainingTime_ns;
  /**
   * Flag to specify whether the shutdown process
   *  after finishing a simulation must be finished or not. It is mostly
   *  useful to run multiple simulations from PyHelios without rebuilding
   *  the survey.
   */
  bool disableShutdown;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Survey playback constructor
   * @param survey The survey itself
   * @param fms The main facade of file management system
   * @param exportToFile Flag to specify if output must be written to a file
   *  (true) or not (false)
   * @param disableShutdown Flag to specify whether the shutdown process
   *  after finishing a simulation must be finished or not. It is mostly
   *  useful to run multiple simulations from PyHelios without rebuilding
   *  the survey.
   * @see Survey
   * @see Simulation::Simulation(unsigned, double, size_t)
   */
  SurveyPlayback(
    std::shared_ptr<Survey> survey,
    std::shared_ptr<helios::filems::FMSFacade> fms,
    int const parallelizationStrategy,
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPoolInterface,
    int const chunkSize,
    std::string fixedGpsTimeStart,
    bool const legacyEnergyModel,
    bool const exportToFile = true,
    bool const disableShutdown = false);

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
  void estimateTime(int legCurrentProgress,
                    bool onGround,
                    double legElapsedLength);
  /**
   * @brief Estimate the leg progress from linear space progress.
   *
   * Let \f$l\f$ be the leg elapsed length and \f$L\f$ be the total length of
   *  the leg. But then, the leg progress \f$l_p\f$ can be estimated from
   *  linear space as follows:
   *
   * \f[
   *  l_p = 100 \frac{l}{L}
   * \f]
   *
   * @param legElapsedLength The elapsed L2 spatial distance (euclidean
   *  distance, standard vector norm) of the current leg. Noted as \f$l\f$.
   * @return Leg progress estimated from linear space progress.
   */
  int estimateSpatialLegProgress(double const legElapsedLength);
  /**
   * @brief Estimate leg progress from angular progress.
   *
   * Let \f$\theta\f$ be the elapsed angular distance in radians and
   *  \f$\Delta\f$ be the difference between end and start angles in radians
   *  too. But then, the leg progress \f$l_p\f$ can be estimated these
   *  angles as follows:
   *
   * \f[
   *  l_p = 100 \frac{\theta}{\Delta}
   * \f]
   *
   * @param legElapsedAngle The elapsed angular distance of the current leg.
   *  Noted as \f$\theta\f$.
   * @return Leg progress estimated from angular progress.
   */
  int estimateAngularLegProgress(double const legElapsedAngle);
  /**
   * @brief Estimate leg progress from temporal progress.
   *
   * Let \f$t\f$ be the current time, \f$t_a\f$ be the starting time point of
   *  the simulated time, and \f$t_b\f$ the ending time point of the
   *  simulation time.
   * Thus, the leg progress \f$l_p\f$ can be estimated from time as follows:
   *
   * \f[
   *  l_p = 100 \frac{t - t_a}{t_b - t_a}
   * \f]
   *
   * @return Leg progress estimated from temporal progress.
   */
  int estimateTemporalLegProgress();
  /**
   * @brief Progress tracking and time estimation
   * @see SurveyPlayback::estimateTime
   */
  void trackProgress();
  /**
   * @brief Perform computations for current simulation step
   */
  void doSimStep() override;
  /**
   * @brief Handle leg completion
   */
  void onLegComplete() override;
  /**
   * @brief Start specified leg
   * @param legIndex Index of leg to start
   * @param manual Specify if leg initialization must be manual (true) or
   *  not (false)
   * @see Platform::initLeg
   * @see Platform::initLegManual
   */
  void startLeg(unsigned int const legIndex, bool const manual);
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
   * @brief Handle survey playback shutdown.
   * @see Simulation::shutdown
   * @see Scanner::AbstractDetector
   */
  void shutdown() override;
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
   * @see SurveyPlayback::getPreviousLeg
   */
  std::shared_ptr<Leg> getCurrentLeg();
  /**
   * @brief Obtain the previous leg, if any
   * @return Previous leg, nullptr if there is no previous leg
   * @see Leg
   * @see SurveyPlayback::getCurrentLeg
   */
  std::shared_ptr<Leg> getPreviousLeg();
  /**
   * @brief Obtain current leg index
   * @return Current leg index
   */
  int getCurrentLegIndex();
  /**
   * @brief Obtain current leg output prefix
   * @param format The integer format string to handle how many digits use
   *  to numerate both strip and leg prefixes
   * @return Current leg output prefix
   */
  std::string getLegOutputPrefix(std::string format = "%03d");

  /**
   * @brief Obtain simulation progress
   * @return Simulation progress
   */
  double getProgress() { return this->progress; }

  /**
   * @brief Obtain current leg progress
   * @return Current leg progress
   */
  double getLegProgress() { return this->legProgress; }

  /**
   * @brief Obtain the number of effective legs
   * @return Number of effective legs
   */
  int getNumEffectiveLegs() { return this->numEffectiveLegs; }

  /**
   * @brief Obtain elapsed time
   * @return Elapsed time (nanoseconds)
   */
  std::chrono::nanoseconds getElapsedTime() { return this->elapsedTime_ns; }
  /**
   * @brief Obtain elapsed length
   * @return The elapsed length
   * @see SurveyPlayback::elapsedLength
   */
  double getElapsedLength() { return this->elapsedLength; }

  /**
   * @brief Obtain expected remaining time
   * @return Expected remaining time (nanoseconds)
   */
  long getRemainingTime() { return this->remainingTime_ns; }
  /**
   * @brief Obtain the leg start time in nanoseconds
   * @return The leg start time (nanoseconds)
   */
  std::chrono::nanoseconds getLegStartTime() { return this->legStartTime_ns; }
  /**
   * @brief Obtain current leg elapsed time
   * @return Current leg elapsed time (nanoseconds)
   */
  std::chrono::nanoseconds getLegElapsedTime()
  {
    return this->legElapsedTime_ns;
  }
  /**
   * @brief Obtain current leg expected remaining time
   * @return Current leg expected remaining time (nanoseconds)
   */
  long getLegRemainingTime() { return this->legRemainingTime_ns; }
};
