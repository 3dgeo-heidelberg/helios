#include <ScanningDevice.h>
#include <cmath>
#include <limits>
#include <logging.hpp>
#include <maths/EnergyModel.h>
#include <maths/MathConstants.h>
#include <scanner/detector/AbstractDetector.h>
#include <stdexcept>
#include <utility>
#if DATA_ANALYTICS >= 2
#include <dataanalytics/HDA_GlobalVars.h>
using namespace helios::analytics;
#endif

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningDevice::ScanningDevice(
  size_t deviceIndex,
  std::string id,
  double beamDiv_rad,
  glm::dvec3 beamOrigin,
  Rotation beamOrientation,
  std::list<int> const& pulseFreqs,
  double pulseLength_ns,
  double pulseAmplitude_W,
  double efficiency,
  double receiverDiameter_m,
  double atmosphericVisibility_km,
  double wavelength_m,
  std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr)
  : devIdx(deviceIndex)
  , id(id)
  , headRelativeEmitterPosition(beamOrigin)
  , headRelativeEmitterAttitude(beamOrientation)
  , beamDivergenceX_rad(beamDiv_rad)
  , beamDivergenceY_rad(beamDiv_rad)
  , pulseLength_ns(pulseLength_ns)
  , pulseAmplitude_W(pulseAmplitude_W)
  , efficiency(efficiency)
  , receiverDiameter_m(receiverDiameter_m)
  , visibility_km(atmosphericVisibility_km)
  , wavelength_m(wavelength_m)
  , supportedPulseFreqs_Hz(pulseFreqs)
  , rangeErrExpr(rangeErrExpr)
{
  setPulseAmplitude(pulseAmplitude_W);
  preparePulse();
  atmosphericExtinction = calcAtmosphericAttenuation();
  cached_Dr2 = receiverDiameter_m * receiverDiameter_m;
}

ScanningDevice::ScanningDevice(ScanningDevice const& scdev)
{
  this->devIdx = scdev.devIdx;
  this->id = scdev.id;
  this->headRelativeEmitterPosition = scdev.headRelativeEmitterPosition;
  this->headRelativeEmitterAttitude = scdev.headRelativeEmitterAttitude;
  this->beamDivergenceX_rad = scdev.beamDivergenceX_rad;
  this->beamDivergenceY_rad = scdev.beamDivergenceY_rad;
  this->beamRotation_rad = scdev.beamRotation_rad;
  this->pulseLength_ns = scdev.pulseLength_ns;
  this->pulseAmplitude_W = scdev.pulseAmplitude_W;
  this->efficiency = scdev.efficiency;
  this->receiverDiameter_m = scdev.receiverDiameter_m;
  this->visibility_km = scdev.visibility_km;
  this->wavelength_m = scdev.wavelength_m;
  this->atmosphericExtinction = scdev.atmosphericExtinction;
  this->FWF_settings = scdev.FWF_settings;
  this->numRays = scdev.numRays;
  this->supportedPulseFreqs_Hz = scdev.supportedPulseFreqs_Hz;
  this->maxNOR = scdev.maxNOR;
  this->pulseModel = scdev.pulseModel;
  this->rangeErrExpr = scdev.rangeErrExpr;
  this->state_currentPulseNumber = scdev.state_currentPulseNumber;
  this->state_lastPulseWasHit = scdev.state_lastPulseWasHit;
  this->cfg_setting_opticsWarmupPhase_s = scdev.cfg_setting_opticsWarmupPhase_s;
  this->state_opticsWarmupApplied = scdev.state_opticsWarmupApplied;
  this->cached_Dr2 = scdev.cached_Dr2;

  if (scdev.scannerHead == nullptr)
    this->scannerHead = nullptr;
  else
    this->scannerHead = std::make_shared<ScannerHead>(*scdev.scannerHead);
  if (scdev.beamDeflector == nullptr)
    this->beamDeflector = nullptr;
  else
    this->beamDeflector = scdev.beamDeflector->clone();
  if (scdev.detector == nullptr)
    this->detector = nullptr;
  else
    this->detector = scdev.detector->clone();
  if (scdev.isSubrayPatternCurrent()) {
    buildSubrayPattern();
    if (scdev.energyModel != nullptr)
      energyModel = std::make_shared<EnergyModel>(*this);
  }
}

// ***  M E T H O D S  *** //
// *********************** //
void
ScanningDevice::prepareSimulation()
{

  preparePulse();
  buildSubrayPattern();
  energyModel = std::make_shared<EnergyModel>(*this);
}

bool
ScanningDevice::isSubrayPatternCurrent() const
{
  return sampledKey && *sampledKey == samplingKey();
}

std::vector<ScanningDevice::Subray> const&
ScanningDevice::getSubrays() const
{
  if (!isSubrayPatternCurrent())
    throw std::logic_error(
      "Subray settings changed; prepare the scanner before tracing");
  return subrays;
}

void
ScanningDevice::buildSubrayPattern()
{
  if (isSubrayPatternCurrent())
    return;
  auto const key = samplingKey();
  auto generated = makeSubrayPattern(key.beam, key.settings);
  subrays = std::move(generated);
  numRays = subrays.size();
  sampledKey = key;
}

void
ScanningDevice::setBeamRotation(double radians)
{
  double const normalized = normalizeBeamRotation(radians);
  double const previous = beamRotation_rad;
  beamRotation_rad = normalized;
  try {
    buildSubrayPattern();
  } catch (...) {
    beamRotation_rad = previous;
    throw;
  }
}

void
ScanningDevice::setBeamDivergence(double divergence_rad)
{
  setBeamDivergences(divergence_rad, divergence_rad);
}

void
ScanningDevice::setBeamDivergences(double x_rad, double y_rad)
{
  double const previousX = beamDivergenceX_rad;
  double const previousY = beamDivergenceY_rad;
  beamDivergenceX_rad = x_rad;
  beamDivergenceY_rad = y_rad;
  try {
    buildSubrayPattern();
  } catch (...) {
    beamDivergenceX_rad = previousX;
    beamDivergenceY_rad = previousY;
    throw;
  }
}

// Simulate energy loss from aerial particles (Carlsson et al., 2001)
// Three-dimensional laser radar modelling (Ove Steinvall, Tomas Carlsson) ?
double
ScanningDevice::calcAtmosphericAttenuation() const
{
  double const lambda_um = wavelength_m * 1e6; // Kruse relation in micrometers
  double const Vm = visibility_km;

  if (lambda_um < 0.5 || lambda_um > 2.0) {
    // Do nothing if wavelength is outside range, approximation will be bad
    logging::WARN(
      "calcAtmosphericAttenuation: wavelength outside 0.5-2.0 um, the Kruse "
      "approximation does not apply, assuming no attenuation");
    return 0.0;
  }

  double q;
  if (Vm > 50.0)
    q = 1.6;
  else if (Vm > 6.0)
    q = 1.3;
  else
    q = 0.585 * std::pow(Vm, 0.33);

  // 3.91/Vm is per kilometre; simulation ranges are measured in metres.
  return (3.91 / Vm) * std::pow(lambda_um / 0.55, -q) / 1000.0;
}

void
ScanningDevice::calcRaysNumber()
{
  buildSubrayPattern();
  numRays = subrays.size();
  std::stringstream ss;
  ss << "Number of subsampling rays (" << id << "): " << numRays;
  logging::INFO(ss.str());
}

void
ScanningDevice::doSimStep(
  unsigned int legIndex,
  double currentGpsTime,
  int simFreq_Hz,
  bool isActive,
  glm::dvec3 const& platformPosition,
  Rotation const& platformAttitude,
  std::function<void(glm::dvec3&, Rotation&)> handleSimStepNoise,
  std::function<void(SimulatedPulse const& sp)> handlePulseComputation)
{
  if (isActive && !state_opticsWarmupApplied) {
    applyWarmupPhase(simFreq_Hz);
  }

  // Do what must be done whether active or not
  // ------------------------------------------//
  // Update head attitude (we do this even when the scanner is inactive):
  scannerHead->doSimStep(simFreq_Hz);

  // Stop if not active
  // -------------------//
  if (!isActive)
    return;

  buildSubrayPattern();

  // Do what active scanner does
  // ----------------------------//
  // Update beam deflector attitude:
  beamDeflector->doSimStep();
  // Check last pulse
  if (!beamDeflector->lastPulseLeftDevice())
    return;
  // Pulse counter
  ++state_currentPulseNumber;
  // Calculate absolute beam originWaypoint:
  glm::dvec3 absoluteBeamOrigin =
    platformPosition + headRelativeEmitterPosition;
  // Calculate absolute beam attitude:
  Rotation absoluteBeamAttitude = calcAbsoluteBeamAttitude(platformAttitude);
  // Handle noise
  handleSimStepNoise(absoluteBeamOrigin, absoluteBeamAttitude);
  // Handle pulse computation
  if (hasMechanicalError()) { // Simulated pulse with mechanical error
    Rotation exactAbsoluteBeamAttitude =
      calcExactAbsoluteBeamAttitude(platformAttitude);
    double const mechanicalRangeError =
      hasMechanicalRangeErrorExpression() ? evalRangeErrorExpression() : 0.0;
    handlePulseComputation(SimulatedPulse(absoluteBeamOrigin,
                                          absoluteBeamAttitude,
                                          exactAbsoluteBeamAttitude,
                                          mechanicalRangeError,
                                          currentGpsTime,
                                          legIndex,
                                          state_currentPulseNumber,
                                          devIdx));
  } else { // Simulated pulse with NO mechanical error
    handlePulseComputation(SimulatedPulse(absoluteBeamOrigin,
                                          absoluteBeamAttitude,
                                          currentGpsTime,
                                          legIndex,
                                          state_currentPulseNumber,
                                          devIdx));
  }
}

void
ScanningDevice::applyWarmupPhase(int simFreq_Hz)
{
  if (state_opticsWarmupApplied || cfg_setting_opticsWarmupPhase_s <= 0.0 ||
      simFreq_Hz <= 0) {
    state_opticsWarmupApplied = true;
    return;
  }

  long long const warmupPulses = std::max(
    0LL, (long long)std::llround(cfg_setting_opticsWarmupPhase_s * simFreq_Hz));

  for (long long i = 0; i < warmupPulses; ++i) {
    scannerHead->doSimStep(simFreq_Hz);
    beamDeflector->doSimStep();
  }
  state_opticsWarmupApplied = true;
}

Rotation
ScanningDevice::calcAbsoluteBeamAttitude(Rotation const& platformAttitude)
{
  Rotation mountRelativeEmitterAttitude =
    scannerHead->getMountRelativeAttitude().applyTo(
      headRelativeEmitterAttitude);
  return platformAttitude.applyTo(mountRelativeEmitterAttitude)
    .applyTo(beamDeflector->getEmitterRelativeAttitude());
}

Rotation
ScanningDevice::calcExactAbsoluteBeamAttitude(Rotation const& platformAttitude)
{
  Rotation exactMountRelativeEmitterAttitude =
    scannerHead->getExactMountRelativeAttitude().applyTo(
      headRelativeEmitterAttitude);
  return platformAttitude.applyTo(exactMountRelativeEmitterAttitude)
    .applyTo(beamDeflector->getExactEmitterRelativeAttitude());
}

void
ScanningDevice::computeSubrays(
  std::function<void(Rotation const& subrayRotation,
                     std::size_t subrayIndex,
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
)
{
  auto const& table = getSubrays();
  std::size_t const numSubrays = table.size();
  for (std::size_t i = 0; i < numSubrays; ++i) {
#if DATA_ANALYTICS >= 2
    bool subrayHit;
    std::vector<double> subraySimRecord(
      14, std::numeric_limits<double>::quiet_NaN());
#endif
    handleSubray(table[i].rotation,
                 i,
                 intersectionHandlingNoiseSource,
                 reflections,
                 intersects
#if DATA_ANALYTICS >= 2
                 ,
                 subrayHit,
                 subraySimRecord
#endif
    );
#if DATA_ANALYTICS >= 2
    HDA_GV.incrementGeneratedSubraysCount();
    subraySimRecord[0] = (double)subrayHit;
    subraySimRecord[1] = table[i].angle_rad;
    pulseRecorder->recordSubraySimulation(subraySimRecord);
#endif
  }
}

void
ScanningDevice::preparePulse()
{
  FWF_settings.validate();
  if (FWF_settings.pulseShape == "sampled") {
    pulseModel = FWF_settings.sampledPulse;
    return;
  }
  pulseModel = PulseShape::create(FWF_settings.pulseShape,
                                  pulseLength_ns,
                                  FWF_settings.recordingTailTolerance,
                                  FWF_settings.gaussianMean_ns);
}

bool
ScanningDevice::initializeFullWaveform(double minHitDist_m,
                                       double maxHitDist_m,
                                       WaveformGrid& grid)
{
  grid = WaveMaths::recordingGrid(minHitDist_m,
                                  maxHitDist_m,
                                  *pulseModel,
                                  FWF_settings.sampleSpacing_m,
                                  FWF_settings.maxRecordingSpan_m);
  return grid.sampleCount > 0;
}

double
ScanningDevice::computeReceivedPower(
  double incidenceCosine,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& radiometryRecords
#endif
) const
{
  return energyModel->computeReceivedPower(incidenceCosine,
                                           targetRange,
                                           mat,
                                           subrayIndex
#if DATA_ANALYTICS >= 2
                                           ,
                                           radiometryRecords
#endif
  );
}
double
ScanningDevice::computeReceivedPowerWithSigma(double targetRange,
                                              double sigma,
                                              std::size_t subrayIndex) const
{
  return energyModel->computeReceivedPowerWithSigma(
    targetRange, sigma, subrayIndex);
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void
ScanningDevice::setLastPulseWasHit(bool value)
{
  if (value == state_lastPulseWasHit)
    return;
  // TODO see
  // https://www.codeproject.com/Articles/12362/A-quot-synchronized-quot-statement-for-C-like-in-J
  state_lastPulseWasHit = value;
}

void
ScanningDevice::setPulseAmplitude(double amplitude)
{
  if (!std::isfinite(amplitude) || amplitude < 0.0)
    throw std::invalid_argument(
      "Pulse amplitude must be finite and nonnegative (watts)");
  pulseAmplitude_W = amplitude;
}

double
ScanningDevice::computePowerRatio(
  double incidenceCosine,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& radiometryRecords
#endif
) const
{
  return energyModel->computePowerRatio(incidenceCosine,
                                        targetRange,
                                        mat,
                                        subrayIndex
#if DATA_ANALYTICS >= 2
                                        ,
                                        radiometryRecords
#endif
  );
}

double
ScanningDevice::computePowerRatioWithSigma(double targetRange,
                                           double sigma,
                                           std::size_t subrayIndex) const
{
  return energyModel->computePowerRatioWithSigma(
    targetRange, sigma, subrayIndex);
}
