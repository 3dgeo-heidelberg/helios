#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_StateJSONReporter.h>
#include <filems/facade/FMSFacade.h>
#include <sim/core/SurveyPlayback.h>
#include <util/helios_version.h>

#include <memory>
#include <sstream>

using namespace helios::analytics;
using namespace helios::filems;
using std::shared_ptr;

// ***  MAIN REPORT METHODS  *** //
// ***************************** //
void
HDA_StateJSONReporter::report()
{
  // Write start of report
  std::stringstream ss;
  ss << "{\n"
     << openEntry("helios", 1, EntryType::OBJECT)
     << craftEntry("version", getHeliosVersion(), 2, true)
     << openEntry("state", 2, EntryType::OBJECT);
  writer.write(ss.str());
  ss.str("");

  // Write report contents
  reportSimulation();
  reportSurvey();
  reportFilems();
  reportPlatform();
  reportScanner();
  reportDeflector();
  reportDetector();
  reportScene();
  reportLegs();

  // Write end of report
  ss << closeEntry(2, true, EntryType::OBJECT) // Close state entry
     << closeEntry(1, true, EntryType::OBJECT) // Close helios entry
     << "}" << std::endl;
  writer.write(ss.str());
  writer.finish();
}

// ***  SECONDARY REPORT METHODS  *** //
// ********************************** //
void
HDA_StateJSONReporter::reportSimulation()
{
  Simulation* sim = (Simulation*)sp;
  std::stringstream ss;
  ss << openEntry("simulation", 3, EntryType::OBJECT)
     << craftEntry("parallelizationStrategy", sim->parallelizationStrategy, 4)
     << craftEntry("simSpeedFactor", sim->getSimSpeedFactor(), 4)
     << craftEntry("simFrequency", sim->getSimFrequency(), 4)
     << craftEntry("callbackFrequency", sim->callbackFrequency, 4)
     << craftEntry("stopped", sim->isStopped(), 4)
     << craftEntry("paused", sim->isPaused(), 4)
     << craftEntry("timeStart_ns", sim->timeStart_ns.count(), 4)
     << craftEntry("currentGpsTime_ns", sim->currentGpsTime_ns, 4)
     << craftEntry("fixedGpsTimeStart", sim->fixedGpsTimeStart, 4, true)
     << craftEntry("currentLegIndex", sim->mCurrentLegIndex, 4)
     << craftEntry("exportToFile", sim->exportToFile, 4)
     << craftEntry("exitAtEnd", sim->exitAtEnd, 4)
     << craftEntry("finished", sim->finished, 4)
     << craftEntry("legStarted", sp->mLegStarted, 4)
     << craftEntry("numEffectiveLegs", sp->getNumEffectiveLegs(), 4)
     << craftEntry("elapsedLength", sp->getElapsedLength(), 4)
     << craftEntry("progress", sp->getProgress(), 4)
     << craftEntry("legProgress", sp->getLegProgress(), 4)
     << craftEntry("legStartTime_ns", sp->getLegStartTime().count(), 4)
     << craftEntry("elapsedTime_ns", sp->getElapsedTime().count(), 4)
     << craftEntry("remainingTime_ns", sp->getRemainingTime(), 4)
     << craftEntry("legElapsedTime_ns", sp->getLegElapsedTime().count(), 4)
     << craftEntry(
          "legRemainingTime_ns", sp->getLegRemainingTime(), 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT) // Close simulation
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportSurvey()
{
  Survey* sv = sp->mSurvey.get();
  std::stringstream ss;
  ss << openEntry("survey", 3, EntryType::OBJECT)
     << craftEntry("name", sv->name, 4, true)
     << craftEntry("numRuns", sv->numRuns, 4)
     << craftEntry("simSpeedFactor", sv->simSpeedFactor, 4)
     << craftEntry("length", sv->getLength(), 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT) // Close survey
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportFilems()
{
  FMSFacade* fms = sp->fms.get();
  FMSWriteFacade& wf = fms->write;
  VectorialMeasurementWriter* mw = wf.getMeasurementWriter().get();
  TrajectoryWriter* tw = wf.getTrajectoryWriter().get();
  VectorialFullWaveformWriter* fw = wf.getFullWaveformWriter().get();
  std::stringstream ss;
  ss << openEntry("filems", 3, EntryType::OBJECT)
     << craftEntry("writeRootDir", wf.getRootDir(), 4, true)
     << craftEntry("measurementWriterOutputPath", mw->getOutputPath(), 4, true)
     << craftEntry("measurementWriterShift", mw->getShift(), 4)
     << craftEntry("measurementWriterIsLasOutput", mw->isLasOutput(), 4)
     << craftEntry("measurementWriterIsLas10", mw->isLas10(), 4)
     << craftEntry("measurementWriterIsZipOutput", mw->isZipOutput(), 4)
     << craftEntry("measurementWriterLasScale", mw->getLasScale(), 4)
     << craftEntry("trajectoryWriterOutputPath", tw->getOutputPath(), 4, true)
     << craftEntry("trajectoryWriterIsLasOutput", tw->isLasOutput(), 4)
     << craftEntry("trajectoryWriterIsLas10", tw->isLas10(), 4)
     << craftEntry("trajectoryWriterIsZipOutput", tw->isZipOutput(), 4)
     << craftEntry("trajectoryWriterLasScale", tw->getLasScale(), 4);
  if (fw != nullptr && fw->hasWriter()) {
    ss << craftEntry("fwfWriterOutputPath", fw->getOutputPath(), 4, true)
       << craftEntry("fwfWriterIsLasOutput", fw->isLasOutput(), 4)
       << craftEntry("fwfWriterIsLas10", fw->isLas10(), 4)
       << craftEntry("fwfWriterIsZipOutput", fw->isZipOutput(), 4)
       << craftEntry("fwfWriterLasScale", fw->getLasScale(), 4, false, true);
  } else {
    ss << craftEntry("fwfWriterOutputPath", "", 4, true)
       << craftEntry("fwfWriterIsLasOutput", false, 4)
       << craftEntry("fwfWriterIsLas10", false, 4)
       << craftEntry("fwfWriterIsZipOutput", false, 4)
       << craftEntry("fwfWriterLasScale", 0.0, 4, false, true);
  }
  ss << closeEntry(3, false, EntryType::OBJECT); // Close filems
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportPlatform()
{
  Platform* pf = sp->mSurvey->scanner->platform.get();
  double roll, pitch, yaw;
  pf->getRollPitchYaw(roll, pitch, yaw);
  std::stringstream ss;
  ss << openEntry("platform", 3, EntryType::OBJECT)
     << craftEntry(
          "relativeMountPosition", pf->cfg_device_relativeMountPosition, 4)
     << craftEntry(
          "relativeMountAttitude", pf->cfg_device_relativeMountAttitude, 4)
     << craftEntry("lastCheckZ", pf->lastCheckZ, 4)
     << craftEntry("lastGroundCheck", pf->lastGroundCheck, 4)
     << craftEntry("dmax", pf->dmax, 4)
     << craftEntry("prevWrittenPos", pf->prevWrittenPos, 4)
     << craftEntry("movePerSec_m", pf->cfg_settings_movePerSec_m, 4)
     << craftEntry("originWaypoint", pf->originWaypoint, 4)
     << craftEntry("targetWaypoint", pf->targetWaypoint, 4)
     << craftEntry("nextWaypoint", pf->nextWaypoint, 4)
     << craftEntry("onGround", pf->onGround, 4)
     << craftEntry("stopAndTurn", pf->stopAndTurn, 4)
     << craftEntry("smoothTurn", pf->smoothTurn, 4)
     << craftEntry("slowdownEnabled", pf->slowdownEnabled, 4)
     << craftEntry("position", pf->position, 4)
     << craftEntry("attitude", pf->attitude, 4)
     << craftEntry("setOrientationOnLegInit", pf->mSetOrientationOnLegInit, 4)
     << craftEntry("writeNextTrajectory", pf->writeNextTrajectory, 4)
     << craftEntry(
          "cached_absoluteMountPosition", pf->cached_absoluteMountPosition, 4)
     << craftEntry(
          "cached_absoluteMountAttitude", pf->cached_absoluteMountAttitude, 4)
     << craftEntry("cached_dir_current", pf->cached_dir_current, 4)
     << craftEntry("cached_dir_current_xy", pf->cached_dir_current_xy, 4)
     << craftEntry("cached_vectorToTarget", pf->cached_vectorToTarget, 4)
     << craftEntry("cached_vectorToTarget_xy", pf->cached_vectorToTarget_xy, 4)
     << craftEntry(
          "cached_distanceToTarget_xy", pf->cached_distanceToTarget_xy, 4)
     << craftEntry(
          "cached_originToTargetDir_xy", pf->cached_originToTargetDir_xy, 4)
     << craftEntry(
          "cached_targetToNextDir_xy", pf->cached_targetToNextDir_xy, 4)
     << craftEntry("cached_endTargetAngle_xy", pf->cached_endTargetAngle_xy, 4)
     << craftEntry("cached_currentAngle_xy", pf->cached_currentAngle_xy, 4)
     << craftEntry(
          "cached_originToTargetAngle_xy", pf->cached_originToTargetAngle_xy, 4)
     << craftEntry(
          "cached_originToTargetAngle_xy", pf->cached_targetToNextAngle_xy, 4)
     << craftEntry("roll", roll, 4) << craftEntry("pitch", pitch, 4)
     << craftEntry("yaw", yaw, 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT); // Close platform
  ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportScanner()
{
  Scanner* sc = sp->mSurvey->scanner.get();
  std::stringstream ss;
  ss << openEntry("scanner", 3, EntryType::OBJECT)
     << craftEntry("writeWaveform", sc->isWriteWaveform(), 4)
     << craftEntry("calcEchowidth", sc->isCalcEchowidth(), 4)
     << craftEntry("fullWaveNoise", sc->isFullWaveNoise(), 4)
     << craftEntry("platformNoiseDisabled", sc->isPlatformNoiseDisabled(), 4)
     << craftEntry("numRays", sc->getNumRays(), 4)
     << craftEntry("fixedIncidenceAngle", sc->isFixedIncidenceAngle(), 4)
     << craftEntry("beamDivergence_rad", sc->getBeamDivergence(), 4)
     << craftEntry("pulseLength_ns", sc->getPulseLength_ns(), 4)
     << craftEntry("pulseFreq_Hz", sc->getPulseFreq_Hz(), 4)
     << craftEntry("deviceID", sc->getDeviceId(), 4, true)
     << craftEntry("averagePower_w", sc->getAveragePower(), 4)
     << craftEntry("beamQuality", sc->getBeamQuality(), 4)
     << craftEntry("efficiency", sc->getEfficiency(), 4)
     << craftEntry("receiverDiameter_m", sc->getReceiverDiameter(), 4)
     << craftEntry("visibility_km", sc->getVisibility(), 4)
     << craftEntry("wavelength_m", sc->getWavelength(), 4)
     << craftEntry("atmosphericExtinction", sc->getAtmosphericExtinction(), 4)
     << craftEntry("beamWaistRadius", sc->getBeamWaistRadius(), 4)
     << craftEntry("currentPulseNumber", sc->getCurrentPulseNumber(), 4)
     << craftEntry("lastPulseWasHit", sc->lastPulseWasHit(), 4)
     << craftEntry("active", sc->isActive(), 4)
     << craftEntry("cached_Dr2", sc->getDr2(), 4)
     << craftEntry("cached_Bt2", sc->getBt2(), 4)
     << craftEntry("trajectoryTimeInterval", sc->trajectoryTimeInterval_ns, 4)
     << craftEntry("lastTrajectoryTime", sc->lastTrajectoryTime, 4)
     << craftEntry("FWFSettings", sc->getFWFSettings(), 4)
     << craftEntry("numTimeBins", sc->getNumTimeBins(), 4)
     << craftEntry("peakIntensityIndex", sc->getPeakIntensityIndex(), 4)
     << craftEntry("peakIntensityIndex", sc->getTimeWave(), 4)
     << craftEntry("headRelativeEmitterPosition",
                   sc->getHeadRelativeEmitterPosition(),
                   4)
     << craftEntry("headRelativeEmitterAttitude",
                   sc->getHeadRelativeEmitterAttitude(),
                   4)
     << craftEntry("supportedPulseFreqs_Hz", sc->getSupportedPulseFreqs_Hz(), 4)
     << craftEntry("maxNOR", sc->getMaxNOR(), 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT) // Close scanner
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportDeflector()
{
  AbstractBeamDeflector* bd = sp->mSurvey->scanner->getBeamDeflector().get();
  std::stringstream ss;
  ss << openEntry("deflector", 3, EntryType::OBJECT)
     << craftEntry("scanFreqMax_Hz", bd->cfg_device_scanFreqMax_Hz, 4)
     << craftEntry("scanFreqMin_Hz", bd->cfg_device_scanFreqMin_Hz, 4)
     << craftEntry("scanAngleMax_rad", bd->cfg_device_scanAngleMax_rad, 4)
     << craftEntry("scanFreq_Hz", bd->cfg_setting_scanFreq_Hz, 4)
     << craftEntry("scanAngle_rad", bd->cfg_setting_scanAngle_rad, 4)
     << craftEntry(
          "verticalAngleMin_rad", bd->cfg_setting_verticalAngleMin_rad, 4)
     << craftEntry(
          "verticalAngleMax_rad", bd->cfg_setting_verticalAngleMax_rad, 4)
     << craftEntry("currentBeamAngle_rad", bd->state_currentBeamAngle_rad, 4)
     << craftEntry("angleDiff_rad", bd->state_angleDiff_rad, 4)
     << craftEntry(
          "angleBetweenPulses_rad", bd->cached_angleBetweenPulses_rad, 4)
     << craftEntry("emitterRelativeAttitude",
                   bd->cached_emitterRelativeAttitude,
                   4,
                   false,
                   true)
     << closeEntry(3, false, EntryType::OBJECT) // Close deflector
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportDetector()
{
  AbstractDetector* ad = sp->mSurvey->scanner->getDetector().get();
  std::stringstream ss;
  ss << openEntry("detector", 3, EntryType::OBJECT)
     << craftEntry("accuracy_m", ad->cfg_device_accuracy_m, 4)
     << craftEntry("rangeMin_m", ad->cfg_device_rangeMin_m, 4)
     << craftEntry("rangeMax_m", ad->cfg_device_rangeMax_m, 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT) // Close detector
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportScene()
{
  Scene* sc = sp->mScanner->platform->scene.get();
  std::stringstream ss;
  ss << openEntry("scene", 3, EntryType::OBJECT)
     << craftEntry("bbox_min", sc->getBBox()->getMin(), 4)
     << craftEntry("bbox_max", sc->getBBox()->getMax(), 4)
     << craftEntry("bbox_crs_min", sc->getBBoxCRS()->getMin(), 4)
     << craftEntry("bbox_crs_max", sc->getBBoxCRS()->getMax(), 4)
     << craftEntry("numPrimitives", sc->primitives.size(), 4)
     << openEntry("parts", 4, EntryType::ARRAY);
  size_t const m = sc->parts.size() - 1;
  for (size_t i = 0; i < m; ++i) {
    std::stringstream ssKey;
    ssKey << "part_" << i;
    ss << craftEntry(ssKey.str(), *(sc->parts[i]), 5);
  }
  std::stringstream ssKey;
  ssKey << "part_" << m;
  ss << craftEntry(ssKey.str(), *(sc->parts[m]), 5, false, true);
  ss << closeEntry(4, false, EntryType::ARRAY) // Close parts
     << craftEntry("numVertices", sc->getAllVertices().size(), 4)
     << craftEntry("hasMovingObjects", sc->hasMovingObjects(), 4, false, true)
     << closeEntry(3, false, EntryType::OBJECT) // Close scene
    ;
  writer.write(ss.str());
}

void
HDA_StateJSONReporter::reportLegs()
{
  std::vector<std::shared_ptr<Leg>> legs = sp->mSurvey->legs;
  size_t const m = legs.size() - 1;
  std::stringstream ss;
  ss << openEntry("legs", 3, EntryType::ARRAY);
  for (size_t i = 0; i < m; ++i) {
    Leg* leg = legs[i].get();
    std::stringstream ssKey;
    ssKey << leg->getSerialId();
    ss << craftEntry(ssKey.str(), *leg, 4, false, false);
  }
  Leg* leg = legs[m].get();
  std::stringstream ssKey;
  ssKey << leg->getSerialId();
  ss << craftEntry(ssKey.str(), *leg, 4, false, true);
  ss << closeEntry(3, true, EntryType::ARRAY); // Close legs
  writer.write(ss.str());
}

// ***  UTIL METHODS  *** //
// ********************** //
template<typename ValType>
std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  ValType const& val,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << openEntry(key, depth, EntryType::VALUE);
  if (asString)
    ss << "\"";
  ss << val;
  if (asString)
    ss << "\"";
  if (!last)
    ss << ",";
  ss << "\n";
  return ss.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  double const& val,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  if (std::isnan(val))
    ss << "null";
  else
    ss << val;
  return craftEntry(key, ss.str(), depth, asString, last);
}
std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  glm::dvec3 const& u,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << "[" << u.x << ", " << u.y << ", " << u.z << "]";
  return craftEntry(key, ss.str(), depth, asString, last);
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  Rotation const& r,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << "[" << r.getQ0() << ", " << r.getQ1() << ", " << r.getQ2() << ", "
     << r.getQ3() << "]";
  return craftEntry(key, ss.str(), depth, asString, last);
}

template<typename T>
std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  std::vector<T> const& u,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << "[";
  if (u.size() > 0) {
    size_t const m = u.size() - 1;
    for (size_t i = 0; i < m; ++i) {
      ss << u[i] << ", ";
    }
    ss << u[m];
  }
  ss << "]";
  return craftEntry(key, ss.str(), depth, asString, last);
}

template<typename T>
std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  std::list<T> const& u,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << "[";
  typename std::list<T>::const_iterator it = u.begin();
  typename std::list<T>::const_iterator itprelast = --u.end();
  for (it = u.begin(); it != itprelast; ++it) {
    ss << *it << ", ";
  }
  ss << *itprelast << "]";
  return craftEntry(key, ss.str(), depth, asString, last);
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  FWFSettings const& fs,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  int const d2 = depth + 1;
  ss << openEntry(key, depth, EntryType::OBJECT)
     << craftEntry("binSize_ns", fs.binSize_ns, d2)
     << craftEntry("minEchoWidth", fs.minEchoWidth, d2)
     << craftEntry("peakEntry", fs.peakEnergy, d2)
     << craftEntry("apertureDiameter", fs.apertureDiameter, d2)
     << craftEntry("scannerEfficiency", fs.scannerEfficiency, d2)
     << craftEntry("atmosphericVisibility", fs.atmosphericVisibility, d2)
     << craftEntry("scannerWaveLength", fs.scannerWaveLength, d2)
     << craftEntry("beamDivergence_rad", fs.beamDivergence_rad, d2)
     << craftEntry("pulseLength_ns", fs.pulseLength_ns, d2)
     << craftEntry("beamSampleQuality", fs.beamSampleQuality, d2)
     << craftEntry("winSize_ns", fs.winSize_ns, d2)
     << craftEntry(
          "maxFullwaveRange_ns", fs.maxFullwaveRange_ns, d2, false, true)
     << closeEntry(depth, last, EntryType::OBJECT) // Close FWFSettings
    ;
  return ss.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  ScenePart const& sp,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  int const d2 = depth + 1;
  std::shared_ptr<AABB> bound = sp.bound;
  glm::dvec3 boundMin =
    (bound == nullptr) ? glm::dvec3(0, 0, 0) : sp.bound->getMin();
  glm::dvec3 boundMax =
    (bound == nullptr) ? glm::dvec3(0, 0, 0) : sp.bound->getMax();
  ss << openEntry(depth, EntryType::OBJECT)
     << craftEntry("ScenePartKey", key, d2, true)
     << craftEntry("ID", sp.mId, d2, true)
     << craftEntry("primitiveType", sp.primitiveType, d2)
     << craftEntry("numPrimitives", sp.mPrimitives.size(), d2)
     << craftEntry("numVertices", sp.getAllVertices().size(), d2)
     << craftEntry("centroid", sp.centroid, d2)
     << craftEntry("bound_min", boundMin, d2)
     << craftEntry("bound_max", boundMax, d2)
     << craftEntry("subpartLimit", sp.subpartLimit, d2)
     << craftEntry("onRayIntersectionMode", sp.onRayIntersectionMode, d2, true)
     << craftEntry(
          "onRayIntersectionArgument", sp.onRayIntersectionArgument, d2, true)
     << craftEntry("randomShift", sp.randomShift, d2)
     << craftEntry("hasLadlut", sp.ladlut != nullptr, d2)
     << craftEntry("origin", sp.mOrigin, d2)
     << craftEntry("rotation", sp.mRotation, d2)
     << craftEntry("scale", sp.mScale, d2)
     << craftEntry("forceOnGround", sp.forceOnGround, d2)
     << craftEntry("objectType", sp.getType(), d2)
     << craftEntry("primitiveType", sp.getPrimitiveType(), d2, false, true)
     << closeEntry(depth, last, EntryType::OBJECT);
  return ss.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  arma::colvec const& u,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  std::stringstream ss;
  ss << "[";
  size_t const m = u.n_elem - 1;
  for (size_t i = 0; i < m; ++i) {
    ss << u[i] << ", ";
  }
  ss << u[m] << "]";
  return craftEntry(key, ss.str(), depth, asString, last);
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  ScannerSettings const& ss,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  int const d2 = depth + 1;
  std::stringstream s;
  s << openEntry(key, depth, EntryType::OBJECT)
    << craftEntry("id", ss.id, d2, true) << craftEntry("active", ss.active, d2)
    << craftEntry("headRotatePerSec_rad", ss.headRotatePerSec_rad, d2)
    << craftEntry("headRotateStart_rad", ss.headRotateStart_rad, d2)
    << craftEntry("headRotateStop_rad", ss.headRotateStop_rad, d2)
    << craftEntry("pulseFreq_Hz", ss.pulseFreq_Hz, d2)
    << craftEntry("scanAngle_rad", ss.scanAngle_rad, d2)
    << craftEntry("verticalAngleMin_rad", ss.verticalAngleMin_rad, d2)
    << craftEntry("verticalAngleMax_rad", ss.verticalAngleMax_rad, d2)
    << craftEntry("scanFreq_Hz", ss.scanFreq_Hz, d2)
    << craftEntry("beamDivAngle", ss.beamDivAngle, d2)
    << craftEntry("trajectoryTimeInterval", ss.trajectoryTimeInterval, d2)
    << craftEntry("warmupPhase_s", ss.warmupPhase_s, d2, false, true)
    << closeEntry(depth, last, EntryType::OBJECT);
  return s.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  PlatformSettings const& ps,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  int const d2 = depth + 1;
  std::stringstream ss;
  ss << openEntry(key, depth, EntryType::OBJECT)
     << craftEntry("id", ps.id, d2, true) << craftEntry("x", ps.x, d2)
     << craftEntry("y", ps.y, d2) << craftEntry("z", ps.z, d2)
     << craftEntry("yawAtDepartureSpecified", ps.yawAtDepartureSpecified, d2)
     << craftEntry("yawAtDeparture", ps.yawAtDeparture, d2)
     << craftEntry("onGround", ps.onGround, d2)
     << craftEntry("stopAndTurn", ps.stopAndTurn, d2)
     << craftEntry("smoothTurn", ps.smoothTurn, d2)
     << craftEntry("slowdownEnabled", ps.slowdownEnabled, d2)
     << craftEntry("movePerSec_m", ps.movePerSec_m, d2, false, true)
     << closeEntry(depth, last, EntryType::OBJECT);
  return ss.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  TrajectorySettings const& ts,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  int const d2 = depth + 1;
  std::stringstream ss;
  ss << openEntry(key, depth, EntryType::OBJECT)
     << craftEntry("tStart", ts.tStart, d2) << craftEntry("tEnd", ts.tEnd, d2)
     << craftEntry("teleportToStart", ts.teleportToStart, d2, false, true)
     << closeEntry(depth, last, EntryType::OBJECT);
  return ss.str();
}

std::string
HDA_StateJSONReporter::craftEntry(std::string const& key,
                                  Leg const& leg,
                                  int const depth,
                                  bool const asString,
                                  bool const last)
{
  int const d2 = depth + 1;
  std::stringstream ss;
  ss << openEntry(depth, EntryType::OBJECT)
     << craftEntry("LegKey", key, d2, true)
     << craftEntry("length", leg.getLength(), d2)
     << craftEntry("belongToStrip", leg.getStrip() != nullptr, d2)
     << craftEntry("wasProcessed", leg.wasProcessed, d2)
     << craftEntry("scannerSettings", *(leg.mScannerSettings), d2)
     << craftEntry("platformSettings", *(leg.mPlatformSettings), d2);
  if (leg.mTrajectorySettings == nullptr)
    ss << craftEntry("trajectorySettings", 0, d2, false, true);
  else
    ss << craftEntry(
      "trajectorySettings", *(leg.mTrajectorySettings), d2, false, true);
  ss << closeEntry(depth, last, EntryType::OBJECT); // Close leg
  return ss.str();
}

std::string
HDA_StateJSONReporter::openEntry(std::string const& key,
                                 int const depth,
                                 EntryType const entryType)
{
  std::stringstream ss;
  for (int i = 0; i < depth; ++i)
    ss << "\t";
  ss << "\"" << key << "\": ";
  if (entryType == EntryType::OBJECT) {
    ss << "{\n";
  } else if (entryType == EntryType::ARRAY) {
    ss << "[\n";
  }
  return ss.str();
}
std::string
HDA_StateJSONReporter::openEntry(int const depth, EntryType const entryType)
{
  std::stringstream ss;
  for (int i = 0; i < depth; ++i)
    ss << "\t";
  if (entryType == EntryType::OBJECT) {
    ss << "{\n";
  } else if (entryType == EntryType::ARRAY) {
    ss << "[\n";
  }
  return ss.str();
}
std::string
HDA_StateJSONReporter::closeEntry(int const depth,
                                  bool const last,
                                  EntryType const entryType)
{
  if (entryType == EntryType::VALUE)
    return "";
  std::stringstream ss;
  for (int i = 0; i < depth; ++i)
    ss << "\t";
  if (entryType == EntryType::OBJECT) {
    if (last)
      ss << "}\n";
    else
      ss << "},\n";
  } else if (entryType == EntryType::ARRAY) {
    if (last)
      ss << "]\n";
    else
      ss << "],\n";
  }
  return ss.str();
}
#endif
