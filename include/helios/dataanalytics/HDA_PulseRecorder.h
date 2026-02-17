#ifdef DATA_ANALYTICS
#pragma once

#include <helios/dataanalytics/HDA_RecordBuffer.h>
#include <helios/dataanalytics/HDA_Recorder.h>

#include <memory>
#include <mutex>
#include <string>

namespace helios {
namespace analytics {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics recorder for pulses (more concretely, pulse
 *  tasks / runnables). It is a class which records relevant data from the
 *  many pulse computations.
 * @see FullWaveformPulseRunnable
 */
class HDA_PulseRecorder : public HDA_Recorder
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The vectors which components are variables involved on a
   *  particular intensity calculation for a given subray.
   *
   * [0, 1, 2] -> \f$(x, y, z)\f$
   *
   * [3] -> Incidence angle in radians.
   *
   * [4] -> The target range in meters, i.e., the distance between the beam's
   *  origin and the intersection point.
   *
   * [5] -> The target area in squared meters.
   *
   * [6] -> The radius in meters, i.e., the distance between the beam's
   *  center line and the intersection point.
   *
   * [7] -> The bidirectional reflectance function (BDRF).
   *
   * [8] -> The cross-section in squared meters.
   *
   * [9] -> The calculated received power, i.e., intensity.
   *
   * [10] -> 1 if the point was captured, 0 otherwise.
   *
   * [11] -> The emitted power.
   *
   * [12] -> The radius step on the discrete elliptical footprint method.
   *
   */
  std::shared_ptr<HDA_RecordBuffer<std::vector<double>>> intensityCalc;

  /**
   * @brief The vectors which components are indices involved on a
   *  particular intensity calculation for a given subray.
   *
   * [0] -> The index of the pulse/ray.
   */
  std::shared_ptr<HDA_RecordBuffer<std::vector<int>>> intensityCalcIndices;
  /**
   * @brief The vectors which components are variables involved on the
   *  subray simulation.
   *
   * [0] -> Subray hit (0 does not hit, 1 hit)
   *
   * [1] -> Divergence angle (in rad)
   *
   * [2] -> Ray direction norm
   *
   * [3] -> Subray direction norm
   *
   * [4] -> Angle between ray and subray (in rad)
   *
   * [5] -> Ray-subray sign check (1 if sign match, 0 otherwise)
   *
   * [6] -> Min time for subray intersection
   *
   * [7] -> Max time for subray intersection
   *
   * [8] -> Subray direction (x component)
   *
   * [9] -> Subray direction (y component)
   *
   * [10] -> Subray direction (z component)
   *
   * [11] -> Ray direction (x component)
   *
   * [12] -> Ray direction (y component)
   *
   * [13] -> Ray direction (z component)
   */
  std::shared_ptr<HDA_RecordBuffer<std::vector<double>>> subraySim;

  /**
   * @brief The mutex to handle concurrent writes to the buffers related to
   *  intensity calculation.
   */
  std::mutex intensityCalcMutex;
  /**
   * @brief The mutex to handle concurrent writes to the buffers related to
   *  subray simulation.
   */
  std::mutex subraySimMutex;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a HDA_PulseRecorder so pulse computations are written to
   *  data files in the directory specified through given path.
   * @param path Path of the directory where simulation records will be
   *  written.
   */
  HDA_PulseRecorder(std::string const& path)
    : HDA_Recorder(path)
  {
    openBuffers();
  }

  virtual ~HDA_PulseRecorder()
  {
    if (isAnyBufferOpen())
      closeBuffers();
  }

  // ***  RECORDER METHODS  *** //
  // ************************** //
  /**
   * @brief Check whether there are opened buffers or not
   * @return True if there is at least one opened buffer, false if there is
   *  not even a single opened buffer
   */
  bool isAnyBufferOpen();
  /**
   * @brief Open all the record buffers so the HDA_SimStepRecord can record
   */
  void openBuffers();
  /**
   * @brief Close all the record buffers. Once it is done, the
   *  HDA_SimStepRecorder will not be able to properly handle any new
   *  record.
   */
  void closeBuffers();

  // ***  RECORD METHODS  *** //
  // ************************ //
  /**
   * @brief Handle all the records for the current simulation step.
   */
  virtual void recordIntensityCalculation(std::vector<double> const& record,
                                          std::vector<int> const& indices);
  /**
   * @brief Like
   *  HDA_PulseRecorder::recordIntensityCalculation(std::vector<double>)
   *  but receiving many records at once.
   * @see HDA_PulseRecorder::recordIntensityCalculation(std::vector<double>)
   */
  virtual void recordIntensityCalculation(
    std::vector<std::vector<double>> const& records,
    std::vector<std::vector<int>> const& indices);
  /**
   * @brief Handle all the records for the current subray simulation.
   */
  virtual void recordSubraySimulation(std::vector<double> const& record);
  /**
   * @brief Like
   *  HDA_PulseRecorder::recordSubraySimulation(std::vector<double>)
   *  but receiving many records at once.
   * @see HDA_PulseRecorder::recordSubraySimulation(std::vector<double>)
   */
  virtual void recordSubraySimulation(
    std::vector<std::vector<double>> const& records);
};

}
}
#endif
