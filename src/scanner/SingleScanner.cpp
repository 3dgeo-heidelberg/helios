#include <maths/WaveMaths.h>
#include <scanner/EvalScannerHead.h>
#include <scanner/SingleScanner.h>
#include <scanner/beamDeflector/evaluable/EvalPolygonMirrorBeamDeflector.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SingleScanner::SingleScanner(
  double const beamDiv_rad,
  glm::dvec3 const beamOrigin,
  Rotation const beamOrientation,
  std::list<int> const& pulseFreqs,
  double const pulseLength_ns,
  std::string const id,
  double const averagePower,
  double const beamQuality,
  double const efficiency,
  double const receiverDiameter,
  double const atmosphericVisibility,
  int const wavelength,
  std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr,
  bool const writeWaveform,
  bool const writePulse,
  bool const calcEchowidth,
  bool const fullWaveNoise,
  bool const platformNoiseDisabled)
  : Scanner(id,
            pulseFreqs,
            writeWaveform,
            writePulse,
            calcEchowidth,
            fullWaveNoise,
            platformNoiseDisabled)
  , scanDev(0,
            id,
            beamDiv_rad,
            beamOrigin,
            beamOrientation,
            pulseFreqs,
            pulseLength_ns,
            averagePower,
            beamQuality,
            efficiency,
            receiverDiameter,
            atmosphericVisibility,
            wavelength / 1000000000.0,
            rangeErrExpr)
{
  // Report scanner state through logging system
  logging::INFO(toString());
}

SingleScanner::SingleScanner(SingleScanner& scanner)
  : Scanner(scanner)
  , scanDev(scanner.scanDev)
{
}

// ***   C L O N E   *** //
// ********************* //
std::shared_ptr<Scanner>
SingleScanner::clone()
{
  std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(*this);
  _clone(*scanner);
  return scanner;
}

void
SingleScanner::_clone(Scanner& sc) const
{
  // Call parent clone method
  Scanner::_clone(sc);
  // Clone attributes from SingleScanner class itself
  // SingleScanner & ssc = static_cast<SingleScanner &>(sc);  // Not used
  // sc.scanDev = scanDev;  // Not necessary because of copy constructor call
}

// ***  SIM STEP UTILS  *** //
// ************************ //
void
SingleScanner::prepareSimulation(bool const legacyEnergyModel)
{
  // Link the deflector angle with the evaluable scanner head
  std::shared_ptr<EvalScannerHead> sh =
    std::dynamic_pointer_cast<EvalScannerHead>(getScannerHead(0));
  std::shared_ptr<PolygonMirrorBeamDeflector> pmbd =
    std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(getBeamDeflector(0));
  if (sh != nullptr && pmbd != nullptr) {
    std::shared_ptr<EvalPolygonMirrorBeamDeflector> epmbd =
      std::dynamic_pointer_cast<EvalPolygonMirrorBeamDeflector>(pmbd);
    if (epmbd != nullptr) {
      sh->setDeflectorAnglePtr(&epmbd->state_currentExactBeamAngle_rad);
    } else {
      sh->setDeflectorAnglePtr(&pmbd->state_currentBeamAngle_rad);
    }
  }
  // Prepare scanning device
  scanDev.prepareSimulation(legacyEnergyModel);
}
void
SingleScanner::onLegComplete()
{
  // Call parent handler for on leg complete events
  Scanner::onLegComplete();
  // Call detector's handle method for on leg complete events
  scanDev.detector->onLegComplete();
}

// ***   M E T H O D S   *** //
// ************************* //
void
SingleScanner::applySettings(std::shared_ptr<ScannerSettings> settings,
                             size_t const idx)
{
  // Configure scanner and scanning device
  setMaxDuration(settings->maxDuration_s);
  setPulseFreq_Hz(settings->pulseFreq_Hz);
  setActive(settings->active);
  setBeamDivergence(settings->beamDivAngle, 0);
  trajectoryTimeInterval_ns = settings->trajectoryTimeInterval * 1000000000.0;
  scanDev.setWarmupPhase_s(settings->warmupPhase_s);
  scanDev.configureBeam();

  // Configure other components
  getDetector(0)->applySettings(settings);
  getScannerHead(0)->applySettings(settings);
  getBeamDeflector(0)->applySettings(settings);
}

void
SingleScanner::doSimStep(unsigned int legIndex, double const currentGpsTime)
{
  // Check whether the scanner is active or not
  bool const _isActive = isActive();

  // Simulate scanning devices
  scanDev.doSimStep(
    legIndex,
    currentGpsTime,
    cfg_setting_pulseFreq_Hz,
    _isActive,
    platform->getAbsoluteMountPosition(),
    platform->getAbsoluteMountAttitude(),
    [&](glm::dvec3& origin, Rotation& attitude) -> void {
      handleSimStepNoise(origin, attitude);
    },
    [&](SimulatedPulse const& sp) -> void { spp->handlePulseComputation(sp); });

  // If the scanner is inactive, stop here:
  if (!_isActive)
    return;

  // Handle trajectory output
  handleTrajectoryOutput(currentGpsTime);
}

void
SingleScanner::prepareDiscretization(size_t const idx)
{
  setNumTimeBins(getPulseLength_ns(0) / getFWFSettings(0).binSize_ns, 0);
  setTimeWave(vector<double>(getNumTimeBins(0)), 0);
  setPeakIntensityIndex(WaveMaths::calcPropagationTimeLegacy(
                          getTimeWave(0),
                          getNumTimeBins(0),
                          getFWFSettings(0).binSize_ns,
                          getPulseLength_ns(0),
                          7.0 // 3.5 too many ops., 7.0 just one op.
                          ),
                        0);
}

Rotation
SingleScanner::calcAbsoluteBeamAttitude(size_t const idx)
{
  return scanDev.calcAbsoluteBeamAttitude(platform->getAbsoluteMountAttitude());
}
void
SingleScanner::computeSubrays(
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
  vector<RaySceneIntersection>& intersects,
  size_t const idx
#if DATA_ANALYTICS >= 2
  ,
  std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
)
{
  scanDev.computeSubrays(handleSubray,
                         intersectionHandlingNoiseSource,
                         reflections,
                         intersects
#if DATA_ANALYTICS >= 2
                         ,
                         pulseRecorder
#endif
  );
}

bool
SingleScanner::initializeFullWaveform(double const minHitDist_m,
                                      double const maxHitDist_m,
                                      double& minHitTime_ns,
                                      double& maxHitTime_ns,
                                      double& nsPerBin,
                                      double& distanceThreshold,
                                      int& peakIntensityIndex,
                                      int& numFullwaveBins,
                                      size_t const idx)
{
  return scanDev.initializeFullWaveform(minHitDist_m,
                                        maxHitDist_m,
                                        minHitTime_ns,
                                        maxHitTime_ns,
                                        nsPerBin,
                                        distanceThreshold,
                                        peakIntensityIndex,
                                        numFullwaveBins);
}

double
SingleScanner::calcIntensity(
  double const incidenceAngle,
  double const targetRange,
  Material const& mat,
  int const subrayRadiusStep,
  size_t const idx
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
) const
{
  return scanDev.calcIntensity(incidenceAngle,
                               targetRange,
                               mat,
                               subrayRadiusStep
#if DATA_ANALYTICS >= 2
                               ,
                               calcIntensityRecords
#endif
  );
}
double
SingleScanner::calcIntensity(double const targetRange,
                             double const sigma,
                             int const subrayRadiusStep,
                             size_t const idx) const
{
  return scanDev.calcIntensity(targetRange, sigma, subrayRadiusStep);
}
