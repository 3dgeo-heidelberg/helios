#pragma once

#include <helios/noise/RandomnessGenerator.h>
#include <helios/scanner/detector/AbstractDetector.h>
#include <helios/scanner/detector/PulseTask.h>
class Measurement;
#include <helios/util/LasSpecification.h>
class Scanner;
#include <helios/scanner/SimulatedPulse.h>
#if DATA_ANALYTICS >= 2
#include <helios/dataanalytics/HDA_PulseRecorder.h>
#endif

#include <mutex>

/**
 * @brief Base abstract class for pulse runnables
 */
class AbstractPulseRunnable : public PulseTask
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Scanner used to simulate the pulse
   */
  std::shared_ptr<Scanner> scanner = nullptr;
  /**
   * @brief Detector used to simulate pulse
   */
  std::shared_ptr<AbstractDetector> detector = nullptr;
  /**
   * @brief The definition of the pulse to be simulated
   */
  SimulatedPulse pulse;
  /**
   * @brief Reference to the scene that is being scanned
   */
  Scene& scene;
  /**
   * @brief Function to apply error to received measurement
   * @see AbstractPulseRunnable::applyMeasurementErrorDirectly
   * @see AbstractPulseRunnable::applyMeasurementErrorFromExpr
   */
  std::function<void(RandomnessGenerator<double>& rg,
                     double& distance,
                     glm::dvec3& beamOrigin,
                     glm::dvec3& beamDirection)>
    applyMeasurementError;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Base constructor for pulse runnables
   * @see AbstractPulseRunnable::scanner
   * @see AbstractPulseRunnable::pulse
   * @see SimulatedPulse
   */
  AbstractPulseRunnable(std::shared_ptr<Scanner> const scanner,
                        SimulatedPulse const& pulse);

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Initialize pending attributes of the abstract pulse runnable
   *  before doing further computations.
   *
   * NOTE that this method alleviates the burden of the sequential thread
   *  by supporting deferred initialization whenever possible.
   */
  virtual void initialize();
  /**
   * @brief Capture point if proceed and write it
   * @param m Measurement
   * @param rg Randomness generator
   * @param allMeasurements Vector of all measurements to store captured
   * point if requested
   * @param allMeasurementsMutex Mutex to handle concurrent access to
   * vector of all measurements
   * @param cycleMeasurements Vector of current cycle measurements to store
   * captured point if requested
   * @param cycleMeasurementsMutex Mutex to handle concurrent access to
   * vector of current cycle measurements
   */
  void capturePoint(Measurement& m,
                    RandomnessGenerator<double>& rg,
                    std::vector<Measurement>* allMeasurements,
                    std::mutex* allMeasurementsMutex,
                    std::vector<Measurement>* cycleMeasurements,
                    std::mutex* cycleMeasurementsMutex
#if DATA_ANALYTICS >= 2
                    ,
                    std::vector<double>& calcIntensityRecord,
                    std::vector<int>& calcIntensityIndices,
                    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
  );
  /**
   * @brief Write the given pulse to the corresponding output file.
   *
   * Each pulse is represented as a vector such that:
   *
   * [0] -> Ray's origin (x coordinate)
   * [1] -> Ray's origin (y coordinate)
   * [2] -> Ray's origin (z coordinate)
   * [3] -> Ray's director vector (x component)
   * [4] -> Ray's director vector (y component)
   * [5] -> Ray's director vector (z component)
   * [6] -> Pulse time
   * [7] -> Device index
   * [8] -> Ray index
   *
   * @param beamDir The direction of the beam/ray (central ray wrt subrays)
   *  corresponding to the pulse.
   */
  void capturePulse(glm::dvec3 const& beamDir);

  /**
   * @brief Apply error to received measurement
   *
   * Let \f$y\f$ be the distance with error, \f$x\f$ be the clean distance,
   *  and \f$\mathcal{N} = \mathcal{N}(\mu=0, \sigma=d_{\mathrm{acc}})\f$ be
   *  a normal distribution with mean 0 and standard deviation equal to the
   *  device accuracy (\f$d_{\mathrm{acc}}\f$). But then, the distance with
   *  error can be determined as follows:
   *
   * \f[
   *  y = x + \mathcal{N}
   * \f]
   *
   * @param rg RandomnessGenerator to be used to apply error to the measure
   *  directly from a normal distribution
   * @param distance Reference to the distance where error shall be applied
   * @param beamOrigin Reference to the beam originWaypoint where error shall
   *  be applied
   * @bream beamDirection Reference to the beam direction where error shall
   * be applied
   * @see AbstractPulseRunnable::applyMeasurementError
   * @see AbstractPulseRunnable::applyMeasurementErrorFromExpr
   */
  void applyMeasurementErrorDirectly(RandomnessGenerator<double>& rg,
                                     double& distance,
                                     glm::dvec3& beamOrigin,
                                     glm::dvec3& beamDirection);
  /**
   * @brief Apply error to received measurement.
   *
   * Let \f$y\f$ be the distance with error, \f$x\f$ be the clean distance,
   *  and \f$\mathcal{N} = \mathcal{N}(\mu=0, \sigma=d_{\mathrm{acc}})\f$ be
   *  a normal distribution with mean \f$0\f$ and standard deviation equal to
   *  the device accuracy (\f$d_{\mathrm{acc}}\f$). But then, the distance
   *  with error can be determined as follows:
   *
   * \f[
   *  y = x + \mathcal{N} f(x)
   * \f]
   *
   * In the above equation, \f$f(x)\f$ is a given expression. For instance,
   *  it is possible to define a distance dependent error in parts per
   *  million \f$d_{ppm}\f$:
   *
   * \f[
   * \begin{split}
   *  y = &\; x + \frac{\mathcal{N}}{d_\mathrm{acc}} \left(
   *      d_{\mathrm{acc}} + x d_{\mathrm{ppm}} 10^{-6} \right) \\
   *    = &\; x + \mathcal{N} \left(
   *      1 + d_{\mathrm{acc}}^{-1} x d_{\mathrm{ppm}} 10^{-6}
   *    \right)
   * \end{split}
   * \f]
   *
   * More concretely, for the above expression:
   *
   * \f[
   *  f(x) = 1 + \frac{x d_{\mathrm{ppm}} 10^{-6}}{d_{\mathrm{acc}}}
   * \f]
   *
   * For example, let us assume \f$d_{\mathrm{acc}} = 0.05\f$ and
   *  \f$d_{ppm} = 30\f$. The expression string representing this would be
   *  (note \f$t\f$ must be used as the variable when using
   *  UnivarExprTreeNode):
   *
   * "1 + (t*30*10^(-6))/0.05"
   *
   * Simplifying to use less operations (more efficient):
   *
   * "1 + t*0.0006"
   *
   * Both expressions define the function:
   *
   * \f[
   *  f(x) = 1 + \frac{x \times 30 \times 10^{-6}}{0.05}
   * \f]
   *
   * Continuing the previous example, it is possible to transform the noise
   *  to a standard normal distribution with mean \f$0\f$ and standard
   *  deviation \f$1\f$ using:
   *
   * "1/0.05"
   *
   * Or equivalently:
   *
   * "20"
   *
   * The above expression corresponds to the function:
   *
   * \f[
   *  f(x) = \frac{1}{d_{\mathrm{acc}}}
   * \f]
   *
   * @param rg RandomnessGenerator to be used to apply error to the measure
   *  from a given univariate expression where \f$t\f$ is the clean distance
   *  in the expression such that \f$t = x\f$.
   * @param distance Reference to the distance where error shall be applied
   * @param beamOrigin Reference to the beam originWaypoint where error shall
   *  be applied
   * @param beamDirection Reference to the beam direction where error shall
   *  be applied
   * @see AbstractPulseRunnable::applyMeasurementError
   * @see AbstractPulseRunnable::applyMeasurementErrorDirectly
   * @see UnivarExprTreeNode
   * @see UnivarExprTreeStringFactory
   */
  void applyMeasurementErrorFromExpr(RandomnessGenerator<double>& rg,
                                     double& distance,
                                     glm::dvec3& beamOrigin,
                                     glm::dvec3& beamDirection);
};
