#pragma once

#include <Measurement.h>
#include <PulseTaskFactory.h>
#include <Rotation.h>
#include <scanner/SimulatedPulse.h>
class Scanner;

#include <glm/glm.hpp>

#include <mutex>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the scanning pulse process interface
 */
class ScanningPulseProcess
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The pulse task factory to build pulse tasks
   * @see PulseTaskFactory
   */
  PulseTaskFactory ptf;
  /**
   * @brief The scanner emitting the pulses
   * @see Scanner
   */
  std::shared_ptr<Scanner> scanner;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for scanning pulse process
   */
  ScanningPulseProcess(std::shared_ptr<Scanner> scanner);
  virtual ~ScanningPulseProcess() = default;

  // ***  PULSE COMPUTATION  *** //
  // *************************** //
  /**
   * @brief Handle pulse computation whatever it is single thread based
   * or thread pool based
   * @see SimulatedPulse
   */
  virtual void handlePulseComputation(SimulatedPulse const& sp) = 0;
  /**
   * @brief Handle behavior of scanning pulse process once current leg has
   *  been completed. It is useful mainly when scanning pulses are computed
   *  in a parallel way, so pending tasks can be adequately handled.
   *
   * Default implementation does nothing. Therefore, any concrete class
   *  providing an implementation of the ScanningPulseProcess interface must
   *  override this method if it needs to handle anything at on leg complete.
   */
  virtual inline void onLegComplete() {}
  /**
   * @brief Handle behavior of scanning pulse process once simulation has
   *  finished.
   *
   * Default implementation does nothing. Therefore, any concrete class
   *  providing an implementation of the ScanningPulseProcess interface must
   *  override this method if it needs to handle anything at on simulation
   *  finished.
   */
  virtual inline void onSimulationFinished() {}

  // *** GETTERs and SETTERs  *** //
  // **************************** //
  /**
   * @brief Obtain the scanner
   * @return The scanner
   * @see ScanningPulseProcess::scanner
   */
  std::shared_ptr<Scanner> getScanner() const;
  /**
   * @brief Obtain the scanner's write waveform flag
   * @return Scanner's write waveform flag
   * @see ScanningPulseProcess::writeWaveform
   */
  bool isWriteWaveform() const;
  /**
   * @brief Obtain the scanner's calc echowidth flag
   * @return Scanner's calc echowidth flag
   * @see ScanningPulseProcess::calcEchowidth
   */
  bool isCalcEchowidth() const;
  /**
   * @brief Obtain the scanner's all measurements vector
   * @return Scanner's all measurements vector
   * @see ScanningPulseProcess::allMeasurements
   */
  std::shared_ptr<std::vector<Measurement>>& getAllMeasurements() const;
  /**
   * @brief Obtain the scanner's all measurements mutex
   * @return Scanner's all measurements mutex
   * @see ScanningPulseProcess::allMeasurementsMutex
   */
  std::shared_ptr<std::mutex>& getAllMeasurementsMutex() const;
  /**
   * @brief Obtain the scanner's cycle measurements vector
   * @return Scanner's cycle measurements vector
   * @see ScanningPulseProcess::cycleMeasurements
   */
  std::shared_ptr<std::vector<Measurement>>& getCycleMeasurements() const;
  /**
   * @brief Obtain the scanner's cycle measurements mutex
   * @return Scanner's cycle measurements mutex
   * @see ScanningPulseProcess::cycleMeasurementsMutex
   */
  std::shared_ptr<std::mutex>& getCycleMeasurementsMutex() const;
};
