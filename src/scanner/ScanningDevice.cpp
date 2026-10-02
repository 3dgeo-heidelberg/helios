#include <ScanningDevice.h>
#include <cmath>
#include <limits>
#include <logging.hpp>
#include <maths/EnergyMaths.h>
#include <maths/MathConstants.h>
#include <maths/model/EnergyModel.h>
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
  double averagePower_w,
  double beamQuality,
  double efficiency,
  double receiverDiameter_m,
  double atmosphericVisibility_km,
  double wavelength_m,
  std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr)
  : devIdx(deviceIndex)
  , id(id)
  , headRelativeEmitterPosition(beamOrigin)
  , headRelativeEmitterAttitude(beamOrientation)
  , beamDivergence_rad(beamDiv_rad)
  , pulseLength_ns(pulseLength_ns)
  , averagePower_w(averagePower_w)
  , beamQuality(beamQuality)
  , efficiency(efficiency)
  , receiverDiameter_m(receiverDiameter_m)
  , visibility_km(atmosphericVisibility_km)
  , wavelength_m(wavelength_m)
  , supportedPulseFreqs_Hz(pulseFreqs)
  , rangeErrExpr(rangeErrExpr)
{
  configureBeam();
  atmosphericExtinction = calcAtmosphericAttenuation();
  cached_Dr2 = receiverDiameter_m * receiverDiameter_m;
}

ScanningDevice::ScanningDevice(ScanningDevice const& scdev)
{
  this->devIdx = scdev.devIdx;
  this->id = scdev.id;
  this->headRelativeEmitterPosition = scdev.headRelativeEmitterPosition;
  this->headRelativeEmitterAttitude = scdev.headRelativeEmitterAttitude;
  this->beamDivergence_rad = scdev.beamDivergence_rad;
  this->pulseLength_ns = scdev.pulseLength_ns;
  this->averagePower_w = scdev.averagePower_w;
  this->beamQuality = scdev.beamQuality;
  this->efficiency = scdev.efficiency;
  this->receiverDiameter_m = scdev.receiverDiameter_m;
  this->visibility_km = scdev.visibility_km;
  this->wavelength_m = scdev.wavelength_m;
  this->atmosphericExtinction = scdev.atmosphericExtinction;
  this->beamWaistRadius = scdev.beamWaistRadius;
  this->FWF_settings = scdev.FWF_settings;
  this->numRays = scdev.numRays;
  this->supportedPulseFreqs_Hz = scdev.supportedPulseFreqs_Hz;
  this->maxNOR = scdev.maxNOR;
  this->numTimeBins = scdev.numTimeBins;
  this->peakIntensityIndex = scdev.peakIntensityIndex;
  this->time_wave = scdev.time_wave;
  this->rangeErrExpr = scdev.rangeErrExpr;
  this->state_currentPulseNumber = scdev.state_currentPulseNumber;
  this->state_lastPulseWasHit = scdev.state_lastPulseWasHit;
  this->cfg_setting_opticsWarmupPhase_s = scdev.cfg_setting_opticsWarmupPhase_s;
  this->state_opticsWarmupApplied = scdev.state_opticsWarmupApplied;
  this->cached_Dr2 = scdev.cached_Dr2;
  this->cached_Bt2 = scdev.cached_Bt2;

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
  if (scdev.isSubrayTableCurrent()) {
    buildSubrayTable();
    if (scdev.energyModel != nullptr)
      energyModel = std::make_shared<EnergyModel>(*this);
  }
}

// ***  M E T H O D S  *** //
// *********************** //
void
ScanningDevice::prepareSimulation()
{
  buildSubrayTable();
  energyModel = std::make_shared<EnergyModel>(*this);
}

bool
ScanningDevice::isSubrayTableCurrent() const
{
  return !subrays.empty() && sampledDivergence_rad == beamDivergence_rad &&
         sampledFactor == FWF_settings.beamSamplingFactor &&
         sampledQuality == FWF_settings.beamSampleQuality;
}

std::vector<ScanningDevice::Subray> const&
ScanningDevice::getSubrays() const
{
  if (!isSubrayTableCurrent())
    throw std::logic_error(
      "Subray settings changed; prepare the scanner before tracing");
  return subrays;
}

void
ScanningDevice::buildSubrayTable()
{
  if (isSubrayTableCurrent())
    return;
  FWF_settings.validateBeamSampling();
  if (!std::isfinite(beamDivergence_rad) || beamDivergence_rad <= 0.0 ||
      beamDivergence_rad >= PI)
    throw std::invalid_argument(
      "Full beam divergence must be finite and in (0, pi)");

  int const quality = FWF_settings.beamSampleQuality;
  double const tangent0 = std::tan(beamDivergence_rad / 2.0);
  double const cutoffTangent = FWF_settings.beamSamplingFactor * tangent0;
  double const cutoff = std::atan(cutoffTangent);
  if (!std::isfinite(cutoffTangent) || cutoff <= 0.0 || cutoff >= PI / 2.0)
    throw std::invalid_argument("Sampling cone is not representable");
  std::vector<Subray> generated;
  double innerTangent = 0.0;
  for (int ring = 0; ring < quality; ++ring) {
    double const population = (ring == 0) ? 1.0 : std::floor(PI_2 * ring);
    if (population >= static_cast<double>(generated.max_size()))
      throw std::invalid_argument("Too many subrays");
    std::size_t const count = static_cast<std::size_t>(population);
    if (count > generated.max_size() - generated.size())
      throw std::invalid_argument("Too many subrays");
    double const angle = cutoff * ring / (quality - 0.5);
    double const outerAngle =
      (ring == quality - 1) ? cutoff : cutoff * (ring + 0.5) / (quality - 0.5);
    double const outerTangent =
      (ring == quality - 1) ? cutoffTangent : std::tan(outerAngle);
    double const innerRatio = innerTangent / tangent0;
    double const outerRatio = outerTangent / tangent0;
    double const exponent = 2.0 * innerRatio * innerRatio;
    double const delta =
      2.0 * (outerRatio - innerRatio) * (outerRatio + innerRatio);
    // expm1 preserves the power of narrow annuli without subtracting
    // near-equals.
    double const share = std::exp(-exponent) * -std::expm1(-delta) / count;
    double const extentSquared =
      (outerTangent - innerTangent) * (outerTangent + innerTangent) / count;
    if (!std::isfinite(share) || !std::isfinite(PI * extentSquared) ||
        extentSquared <= 0.0)
      throw std::invalid_argument("Subray patch is not representable");
    Rotation const tilt(Directions::right, angle);
    for (std::size_t j = 0; j < count; ++j) {
      Rotation const azimuth(Directions::forward, PI_2 * j / count);
      generated.push_back({ azimuth.applyTo(tilt),
                            angle,
                            share,
                            PI * extentSquared,
                            outerAngle });
    }
    innerTangent = outerTangent;
  }
  subrays = std::move(generated);
  numRays = subrays.size();
  sampledDivergence_rad = beamDivergence_rad;
  sampledFactor = FWF_settings.beamSamplingFactor;
  sampledQuality = quality;
}

void
ScanningDevice::configureBeam()
{
  cached_Bt2 = beamDivergence_rad * beamDivergence_rad;
  beamWaistRadius = (beamQuality * wavelength_m) / (PI * beamDivergence_rad);
}

// Simulate energy loss from aerial particles (Carlsson et al., 2001)
// Three-dimensional laser radar modelling (Ove Steinvall, Tomas Carlsson) ?
double
ScanningDevice::calcAtmosphericAttenuation() const
{
  double q;
  double const lambda = wavelength_m * 1e9;
  double const Vm = visibility_km;

  if (lambda < 500 && lambda > 2000) {
    // Do nothing if wavelength is outside range, approximation will be bad
    return 0;
  }

  if (Vm > 50)
    q = 1.6;
  else if (Vm > 6 && Vm < 50)
    q = 1.3;
  else
    q = 0.585 * pow(Vm, 0.33);

  return (3.91 / Vm) * pow((lambda / 0.55), -q);
}

void
ScanningDevice::calcRaysNumber()
{
  buildSubrayTable();
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

  buildSubrayTable();

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

bool
ScanningDevice::initializeFullWaveform(double minHitDist_m,
                                       double maxHitDist_m,
                                       double& minHitTime_ns,
                                       double& maxHitTime_ns,
                                       double& nsPerBin,
                                       double& distanceThreshold,
                                       int& peakIntensityIndex,
                                       int& numFullwaveBins)
{
  // Calc time at minimum and maximum distance
  // (i.e. total beam time in fwf signal)
  nsPerBin = FWF_settings.binSize_ns;
  peakIntensityIndex = this->peakIntensityIndex;
  double const peakFactor = peakIntensityIndex * nsPerBin;
  // Time until first maximum minus rising flank
  minHitTime_ns = minHitDist_m / SPEEDofLIGHT_mPerNanosec - peakFactor;
  // Time until last maximum time for signal decay with 1 bin for buffer
  maxHitTime_ns = maxHitDist_m / SPEEDofLIGHT_mPerNanosec + pulseLength_ns -
                  peakFactor + nsPerBin; // 1 bin for buffer

  // Calc ranges and threshold
  double hitTimeDelta_ns = maxHitTime_ns - minHitTime_ns;
  double const maxFullwaveRange_ns = FWF_settings.maxFullwaveRange_ns;
  distanceThreshold = maxHitDist_m;
  if (maxFullwaveRange_ns > 0.0 && hitTimeDelta_ns > maxFullwaveRange_ns) {
    hitTimeDelta_ns = maxFullwaveRange_ns;
    maxHitTime_ns = minHitTime_ns + maxFullwaveRange_ns;
    distanceThreshold = SPEEDofLIGHT_mPerNanosec * maxFullwaveRange_ns;
  }

  // Check if full wave is possible
  if ((detector->cfg_device_rangeMin_m / SPEEDofLIGHT_mPerNanosec) >
      minHitTime_ns) {
    return false;
  }

  // Compute fullwave variables
  numFullwaveBins = ((int)std::ceil(maxHitTime_ns / nsPerBin)) -
                    ((int)ceil(minHitTime_ns / nsPerBin));

  // update maxHitTime to fit the discretized fullwave bins
  // minus 1 is necessary as the minimum is in bin #0
  maxHitTime_ns = minHitTime_ns + (numFullwaveBins - 1) * nsPerBin;

  return true;
}

double
ScanningDevice::calcIntensity(
  double incidenceAngle,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
) const
{
  return energyModel->computeIntensity(incidenceAngle,
                                       targetRange,
                                       mat,
                                       subrayIndex
#if DATA_ANALYTICS >= 2
                                       ,
                                       calcIntensityRecords
#endif
  );
}
double
ScanningDevice::calcIntensity(double targetRange,
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
