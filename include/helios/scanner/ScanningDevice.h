#pragma once

#include <helios/assetloading/Asset.h>
#include <helios/maths/Directions.h>
#include <helios/maths/Rotation.h>
#include <helios/maths/model/EnergyModel.h>
#include <helios/scanner/FWFSettings.h>
#include <helios/scanner/ScannerHead.h>
#include <helios/scanner/SimulatedPulse.h>
#include <helios/scanner/beamDeflector/AbstractBeamDeflector.h>
#include <helios/scene/Scene.h>
class AbstractDetector;
#include <helios/adt/exprtree/UnivarExprTreeNode.h>
#include <helios/noise/NoiseSource.h>
#include <helios/scene/RaySceneIntersection.h>
#if DATA_ANALYTICS >= 2
#include <helios/dataanalytics/HDA_PulseRecorder.h>
using helios::analytics::HDA_PulseRecorder;
#endif

#include <glm/glm.hpp>

#include <list>
#include <map>
#include <string>
#include <unordered_map>

class Scanner;
class SingleScanner;
class MultiScanner;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a scanning device. Any scanner needs to be
 *  composed of at least one scanning device to be computable.
 */
class ScanningDevice : public Asset
{
protected:
  // ***  F R I E N D S  *** //
  // *********************** //
  friend class Scanner;
  friend class SingleScanner;
  friend class MultiScanner;
  friend class BaseEnergyModel;
  friend class ImprovedEnergyModel;

  // ***  DEVICE ATTRIBUTES  *** //
  // *************************** //
  /**
   * @brief The index of the scanning device in the MultiScanner context
   */
  size_t devIdx;
  /**
   * @brief Device identifier
   */
  std::string id = "";
  /**
   * @brief Head relative emitter position
   */
  glm::dvec3 headRelativeEmitterPosition = glm::dvec3(0, 0, 0);
  /**
   * @brief Head relative emitter attitude
   */
  Rotation headRelativeEmitterAttitude = Rotation(Directions::right, 0);
  /**
   * @brief Beam divergence (radians)
   */
  double beamDivergence_rad = 0;
  /**
   * @brief Pulse length (nanoseconds)
   */
  double pulseLength_ns = 0;
  /**
   * @brief Average power (watts)
   */
  double averagePower_w;
  /**
   * @brief Beam quality
   */
  double beamQuality;
  /**
   * @brief Device efficiency
   */
  double efficiency;
  /**
   * @brief Receiver diameter (meters)
   */
  double receiverDiameter_m;
  /**
   * @brief Visibility (kilometers)
   */
  double visibility_km;
  /**
   * @brief Wave length (meters)
   */
  double wavelength_m;
  /**
   * @brief Atmospheric extinction
   */
  double atmosphericExtinction;
  /**
   * @brief Beam waist radius
   */
  double beamWaistRadius;
  /**
   * @brief Scanner head composing the scanning device
   * @see ScannerHead
   */
  std::shared_ptr<ScannerHead> scannerHead;
  /**
   * @brief Beam deflector composing the scanner
   * @see AbstractBeamDeflector
   */
  std::shared_ptr<AbstractBeamDeflector> beamDeflector;
  /**
   * @brief Detector composing the scanner
   * @see AbstractDetector
   */
  std::shared_ptr<AbstractDetector> detector;
  /**
   * @brief Full wave form settings for the scanner
   * @see FWFSettings
   */
  FWFSettings FWF_settings;
  /**
   * @brief Number of rays computed by the calcRaysNumber function
   * @see ScanningDevice::calcRaysNumber
   */
  int numRays = 0;
  /**
   * @brief Pulse frequencies (hertz) supoported by the scanner
   */
  std::list<int> supportedPulseFreqs_Hz;
  /**
   * @brief Maximum number of returns per pulse. When 0, it means there is
   *  not maximum at all
   * @see Scanner::checkMaxNOR
   */
  int maxNOR = 0;
  /**
   * @brief Number of bins defining the discretization size
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
   * @brief Time discretization vector
   */
  std::vector<double> time_wave;

  /**
   * @brief The expression tree to compute range errors as a function of
   *  vertical angle \f$\thneta\f$
   *
   * \f[
   *  \Delta r(\theta)
   * \f]
   *
   * @see UnivarExprTreeNode
   */
  std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr = nullptr;

  /**
   * @brief The energy model to compute the intensity for any pulse
   *  emitted by the scanning device.
   * @see EnergyModel
   * @see BaseEnergyModel
   */
  std::shared_ptr<EnergyModel> energyModel = nullptr;
  /**
   * @brief The minimum energy threshold (in Watts) to filter received energy
   *  values during fullwave analysis in
   *  FullWaveformPulseRunnable::handleFullWaveformBin and
   *  FullWaveformPulseRunnable::detectPeak methods.
   * @see FullWaveformPulseRunnable::handleFullWaveformBin
   * @see FullWaveformPulseRunnable::detectPeak
   */
  double receivedEnergyMin_W = 0.0001;

  // ***  STATE ATTRIBUTES  *** //
  // ************************** //
  /**
   * @brief Current pulse number
   */
  int state_currentPulseNumber = 0;

  /**
   * @brief Flag specifying if last pulse was hit (true) or not (false)
   */
  bool state_lastPulseWasHit = false;

public:
  // ***  CACHED ATTRIBUTES  *** //
  // *************************** //
  /**
   * @brief \f$D_{r2}\f$ understood as the square of receiver diameter
   *
   * \f[
   *  D_{r2} = \textrm{receiverDiamater}^{2}
   * \f]
   *
   * @see ScanningDevice::receiverDiameter_m
   */
  double cached_Dr2;
  /**
   * @brief \f$B_{t2}\f$ understood as the square of beam divergence
   *
   * \f[
   *  B_{t2} = \textrm{beamDivergence}^{2}
   * \f]
   *
   * @see ScanningDevice::beamDivergence_rad
   */
  double cached_Bt2;
  /**
   * @brief The rotation representing the subray divergence wrt to the
   *  central ray.
   */
  std::vector<Rotation> cached_subrayRotation;
  /**
   * @brief The divergence angle for each subray.
   */
  std::vector<double> cached_subrayDivergenceAngle_rad;
  /**
   * @brief The subray radius step or iteration.
   */
  std::vector<int> cached_subrayRadiusStep;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief ScanningDevice constructor from given values
   */
  ScanningDevice(
    size_t const devIdx,
    std::string const id,
    double const beamDiv_rad,
    glm::dvec3 const beamOrigin,
    Rotation const beamOrientation,
    std::list<int> const& pulseFreqs,
    double const pulseLength_ns,
    double const averagePower,
    double const beamQuality,
    double const efficiency,
    double const receiverDiameter_m,
    double const atmosphericVisibility_km,
    double const wavelength_m,
    std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr = nullptr);
  /**
   * @brief Copy constructor for the ScanningDevice
   * @param scdev The scanning device to be copied
   */
  ScanningDevice(ScanningDevice const& scdev);
  ~ScanningDevice() override = default;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Prepare the scanning device to deal with the simulation.
   *
   * For example, data related to the subray generation process will be
   *  cached to avoid redundant operations.
   *
   * @param legacyEnergyModel Whether to use the legacy energy model (True)
   *  or not (False).
   */
  void prepareSimulation(bool const legacyEnergyModel = false);

  /**
   * @brief Configure beam related attributes. It is recommended to
   *  reconfigure beam attributes always that beam divergence, beam quality
   *  or wavelength are updated.
   * @see ScanningDevice::beamDivergence_rad
   * @see ScanningDevice::beamQuality
   * @see ScanningDevice::wavelength_m
   * @see ScanningDevice::beamWaistRadius
   * @see ScanningDevice::cached_Bt2
   */
  void configureBeam();
  /**
   * @brief Compute the atmospheric attenuation to be used as the
   *  atmospheric extinction
   * @return Atmospheric attenuation
   * @see Scanner::atmosphericExtinction
   */
  double calcAtmosphericAttenuation() const;
  /**
   * @brief Compute the number of rays depending on beam sample quality
   */
  void calcRaysNumber();
  /**
   * @brief Do the simulation steps of the scanning device
   * @param legIndex Current simulation leg index
   * @param currentGpsTime Current simulated GPS time
   * @param simFreq_Hz The simulation frequency in Hertz
   * @param isActive True if the scanner is active, False otherwise
   * @param platformPosition The absolute mount position of the
   *  platform
   */
  void doSimStep(
    unsigned int legIndex,
    double const currentGpsTime,
    int const simFreq_Hz,
    bool const isActive,
    glm::dvec3 const& platformPosition,
    Rotation const& platformAttitude,
    std::function<void(glm::dvec3&, Rotation&)> handleSimStepNoise,
    std::function<void(SimulatedPulse const& sp)> handlePulseComputation);
  /**
   * @brief Compute the absolute beam attitude of the scanning device
   *  with respect to given absolute platform attitude
   * @þaram platformAttitude The absolute mount attitude of the platform
   *  where the scanning device is placed
   * @return The absolute beam attitude of the scanning device with respect
   *  to given absolute platform attitude
   */
  Rotation calcAbsoluteBeamAttitude(Rotation const& platformAttitude);
  /**
   * @brief Compute the exact absolute beam attitude (which means ignoring
   *  the mechanical errors).
   * @return The exact absolute beam attitude (no mechanical errors) of the
   *  scanning device with respect to given absolute platform attitude.
   * @see ScanningDevice::calcAbsoluteBeamAttitude
   */
  Rotation calcExactAbsoluteBeamAttitude(Rotation const& platformAttitude);

  /**
   * @see Scanner::computeSubrays
   */
  void computeSubrays(
    std::function<void(Rotation const& subrayRotation,
                       int const subrayRadiusStep,
                       NoiseSource<double>& intersectionHandlingNoiseSource,
                       std::map<double, double>& reflections,
                       vector<RaySceneIntersection>& intersects
#if DATA_ANALYTICS >= 2
                       ,
                       bool& subrayHit,
                       std::vector<double>& subraySimRecord
#endif
                       )> handleSubray,
    NoiseSource<double>& intersectionHandlingNoiseSource,
    std::map<double, double>& reflections,
    std::vector<RaySceneIntersection>& intersects
#if DATA_ANALYTICS >= 2
    ,
    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
  );
  /**
   * @see Scanner::initializeFullWaveform
   */
  bool initializeFullWaveform(double const minHitDist_m,
                              double const maxHitDist_m,
                              double& minHitTime_ns,
                              double& maxHitTime_ns,
                              double& nsPerBin,
                              double& distanceThreshold,
                              int& peakIntensityIndex,
                              int& numFullwaveBins);

  /**
   * @brief Compute intensity. It is the strength of the laser going back
   *  to the detector considering the emitted power \f$P_e\f$, and its
   *  corresponding received power \f$P_r\f$.
   *
   * Let \f$I_0\f$ be the average power of the scanning device, \f$\lambda\f$
   *  be the wavelength, \f$R\f$ be the target range, \f$R_0\f$ be the
   *  minimum range, and \f$w_0\f$ be the beam waist radius. Thus, the
   *  emitted power \f$P_e\f$ can be defined:
   * \f[
   *  P_e = I_0 \exp\left[- \frac{
   *          2 \pi^2 r^2 w_0^2
   *      }{
   *          \lambda^2 \left(R_0^2 + R^2\right)
   *      }\right]
   * \f]
   *
   * Note that if \f$a_e\f$ is the atmospheric extinction coefficient, then
   *  the atmospheric factor \f$\eta_a\f$ is:
   * \f[
   *  \eta_a = \exp\left(-2Ra_e\right)
   * \f]
   *
   * Now, let \f$\eta_s\f$ be the efficiency of the scanning device,
   *  \f$\sigma\f$ be the cross section between the target area and the
   *  incidence angle, \f$D_{r2}\f$ the squared receiver diameter, and
   *  \f$B_{t2}\f$ the squared beam divergence. Thus, the received power
   *  can be calculated as:
   * \f[
   * \begin{split}
   *  P_r =& \frac{P_e D_r^2 \eta_s \eta_a \sigma}{4 \pi R^4 B_t^2} \\
   *   =& \frac{
   *          I_0 D_r^2 \eta_s \sigma
   *      }{
   *          4 \pi R^4 B_t^2
   *      }
   *      \exp\left[-\left(
   *          \frac{2\pi^2r^2w_0^2}{\lambda^2\left(R_0^2 + R^2\right)} +
   *          2Ra_e
   *      \right)\right]
   * \end{split}
   * \f]
   *
   * Finally, the intensity would be \f$P_r 10^9\f$.
   *
   * @return Computed intensity \f$P_r 10^9\f$
   */
  double calcIntensity(double const incidenceAngle,
                       double const targetRange,
                       Material const& mat,
                       int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
                       ,
                       std::vector<std::vector<double>>& calcIntensityRecords
#endif
  ) const;

  /**
   * @brief Version of ScanningDevice::calcIntensity with precomputed
   *  \f$\sigma\f$
   * @see ScanningDevice::calcIntensity
   */
  double calcIntensity(double const targetRange,
                       double const sigma,
                       int const subrayRadiusStep) const;

  int calcTimePropagation(std::vector<double>& timeWave);

  /**
   * @brief Evaluate the expression tree modeling the mechanical range error
   * @return Mechanical range error term
   * @see ScanningDevice::rangeErrExpr
   */
  inline double evalRangeErrorExpression()
  {
    return rangeErrExpr->eval(beamDeflector->getCurrentExactBeamAngle());
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Check if last pulse was hit (true) or not (false) for the
   *  scanning device.
   * @param idx The index of the scanning device which last pulse must be
   *  checked.
   * @return True if the last pulse of the scanning device was hit, false
   *  otherwise.
   * @see ScanningDevice::state_lastPulseWasHit
   * @see Scanner::lastPulseWasHit(size_t const)
   */
  inline bool lastPulseWasHit() const { return state_lastPulseWasHit; }
  /**
   * @brief Specify if last pulse was hit (true) or not (false)
   * @param lastPulseWasHit New last pulse hit specification
   * @see Scanner::state_lastPulseWasHit
   */
  void setLastPulseWasHit(bool const value);
  /**
   * @brief Set the relative emitter position
   * @see ScanningDevice::headRelativeEmitterPosition
   */
  inline void setHeadRelativeEmitterPosition(
    glm::dvec3 const& headRelativeEmitterPosition)
  {
    this->headRelativeEmitterPosition = headRelativeEmitterPosition;
  }
  /**
   * @brief Set the relative emitter attitude
   * @see ScanningDevice::headRelativeEmitterAttitude
   */
  inline void setHeadRelativeEmitterAttitude(
    Rotation const& headRelativeEmitterAttitude)
  {
    this->headRelativeEmitterAttitude = headRelativeEmitterAttitude;
  }
  /**
   * @brief Obtain the Full Waveform settings of the scanning device
   * @see ScanningDevice::FWF_settings
   */
  inline FWFSettings const& getFWFSettings() { return FWF_settings; }
  /**
   * @brief Set the Full Waveform settings of the scanning device
   * @see ScanningDevice::FWF_settings
   */
  inline void setFWFSettings(std::shared_ptr<FWFSettings> FWF_settings)
  {
    this->FWF_settings = *FWF_settings;
  }
  /**
   * @brief Check whether the scanning device simulates mechanical errors
   *  (true) or not (false).
   *
   * A scanning device is said to simulate mechanical errors if at least
   *  one of its components (e.g., head or deflector) simulates mechanical
   *  errors. This includes the mechanical range error expression tree
   *  handled directly by the scanning device.
   *
   * @return True if the scanning device simulates mechanical errors,
   *  false otherwise.
   *
   * @see ScannerHead::hasMechanicalError
   * @see AbstractBeamDeflector::hasMechanicalError
   * @see SimulatedPulse::mechanicalError
   * @see ScanningDevice::hasMechanicalRangeError
   */
  inline bool hasMechanicalError()
  {
    return scannerHead->hasMechanicalError() ||
           beamDeflector->hasMechanicalError() ||
           hasMechanicalRangeErrorExpression();
  }
  /**
   * @brief Check whether the scanning device models mechanical range errors
   *  with an expression tree (true) or not (false).
   *
   * A scanning device is said to model mechanical range errors with an
   *  expression tree if it has an associated expression tree for range
   *  errors (i.e., non-null pointer).
   *
   * NOTE the expression tree for mechanical range errors is enough to
   *  consider that the scanning device has mechanical errors.
   *
   * @return True if the scanning device models mechanical range errors with
   *  an expression tree, False otherwise.
   *
   * @see ScanningDevice::hasMechanicalError
   * @see ScanningDevice::rangeErrExpr
   */
  inline bool hasMechanicalRangeErrorExpression()
  {
    return rangeErrExpr != nullptr;
  }
  /**
   * @brief Set the energy model for the scanning device.
   * @param energyModel The new energy model for the scanning device.
   */
  inline void setEnergyModel(std::shared_ptr<EnergyModel> energyModel)
  {
    this->energyModel = energyModel;
  }
  /**
   * @brief Get the energy model of the scanning device.
   * @return The energy model of the scanning device.
   */
  inline std::shared_ptr<EnergyModel> getEnergyModel() const
  {
    return energyModel;
  }

  /**
   * @brief Get the minimum received energy threshold of the scanning device.
   * @return The minimum received energy threshold of the scanning device.
   * @see ScanningDevice::receivedEnergyMin_W
   */
  inline double getReceivedEnergyMin() const { return receivedEnergyMin_W; }

  /**
   * @brief Set the minimum received energy threshold for the scanning
   *  device.
   * @param receivedEnergyMin_W The new minimum received energy threshold
   *  for the scanning device.
   */
  inline void setReceivedEnergyMin(double const receivedEnergyMin_W)
  {
    this->receivedEnergyMin_W = receivedEnergyMin_W;
  }
};
