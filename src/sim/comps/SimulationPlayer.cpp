#include <helios/assetloading/ScenePart.h>
#include <helios/filems/factory/FMSFacadeFactory.h>
#include <helios/scene/Scene.h>
#include <helios/sim/comps/SimulationPlayer.h>
#include <helios/sim/core/Simulation.h>
#include <helios/sim/core/SurveyPlayback.h>
using helios::filems::FMSFacadeFactory;
#include <helios/platform/InterpolatedMovingPlatform.h>
#include <helios/platform/MovingPlatform.h>
#include <helios/scanner/Scanner.h>

#include <chrono>
using namespace std::chrono;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SimulationPlayer::SimulationPlayer(Simulation& sim)
  : sim(sim)
  , scene(*sim.getScanner()->platform->scene)
  , plays(0)
  , platformStart(sim.getScanner()->platform->clone())
{
}

// ***  MAIN PUBLIC METHODS  *** //
// ***************************** //
bool
SimulationPlayer::hasPendingPlays()
{
  return getNumComputedPlays() < getNumTargetPlays();
}

void
SimulationPlayer::endPlay()
{
  // Update the counts of plays
  ++plays;
  // Get all the scene parts (objects) that will do a swap on repeat
  std::vector<std::shared_ptr<ScenePart>> swapOnRepeatObjects =
    scene.getSwapOnRepeatObjects();
  bool const keepCRS = isKeepCRS(swapOnRepeatObjects);
  std::stringstream ss;
  for (std::shared_ptr<ScenePart> sp : swapOnRepeatObjects) {
    ss.str("");
    ss << "Swapping scene part \"" << sp->mId << "\" ...";
    logging::DEBUG(ss.str());
    std::shared_ptr<SwapOnRepeatHandler> sorh = sp->getSwapOnRepeatHandler();
    if (sorh->hasPendingSwaps()) {
      ss.str("");
      ss << "Scene part \"" << sp->mId << "\" has pending swaps.";
      logging::DEBUG(ss.str());
      // Backup rotation (because RotateFilter modifies inplace)
      Rotation const rotationBackup = sp->mRotation;
      // Do the swap
      sorh->swap(*sp);
      // Backup rotation (because RotateFilter modifies inplace)
      sp->mRotation = rotationBackup;
    }
    ss.str("");
    ss << "Swapped scene part \"" << sp->mId << "\"!";
    logging::DEBUG(ss.str());
  }
  // Prepare next play, if any
  if (plays < getNumTargetPlays()) {
    logging::DEBUG("Preparing next simulation play ...");
    // Restart platform
    logging::DEBUG("Restarting platform for next simulation play ...");
    restartPlatform(*sim.getScanner()->platform);
    // Restart filems
    logging::DEBUG(
      "Restarting file management system for next simulation play ...");
    restartFileMS(*sim.getScanner()->fms);
    // Restart scanner
    logging::DEBUG("Restarting scanner for next simulation play ...");
    restartScanner(*sim.getScanner());
    // Restart scene
    logging::DEBUG("Restarting scene for next simulation play ...");
    restartScene(*sim.getScanner()->platform->scene, keepCRS);
    // Restart simulation
    logging::DEBUG("Restarting context for next simulation play ...");
    restartSimulation(sim);
    logging::DEBUG("Next simulation play prepared.");
  }
}

int
SimulationPlayer::getNumTargetPlays()
{
  // Get all the scene parts (objects) that will do a swap on repeat
  std::vector<std::shared_ptr<ScenePart>> swapOnRepeatObjects =
    scene.getSwapOnRepeatObjects();
  // The number of target plays is one plus the maximum number of
  // target swaps among the many swap on repeat objects
  int numTargetPlays = 1;
  for (std::shared_ptr<ScenePart> sp : swapOnRepeatObjects) {
    std::shared_ptr<SwapOnRepeatHandler> sorh = sp->getSwapOnRepeatHandler();
    if (sorh->getNumTargetSwaps() >= numTargetPlays) {
      numTargetPlays = 1 + sorh->getNumTargetReplays();
    }
  }
  // Return the number of target plays
  return numTargetPlays;
}

int
SimulationPlayer::getNumComputedPlays()
{
  return plays;
}

void
SimulationPlayer::prepareRepeat()
{
  // Handle end of current simulation play to prepare next replay
  sim.mStopped = true;
  sim.getScanner()->getDetector()->shutdown();
}

// ***  UTIL PROTECTED METHODS  *** //
// ******************************** //
void
SimulationPlayer::restartPlatform(Platform& p)
{
  // Restart general platform
  p.attitude = platformStart->attitude;
  p.position = platformStart->position;
  p.originWaypoint = platformStart->originWaypoint;
  p.targetWaypoint = platformStart->targetWaypoint;
  // Restart moving platform
  try {
    MovingPlatform& mp = dynamic_cast<MovingPlatform&>(p);
    std::shared_ptr<MovingPlatform> mpStart =
      std::dynamic_pointer_cast<MovingPlatform>(platformStart);
    mp.setVelocity(mpStart->getVelocity());
    mp.cached_vectorToTarget = mpStart->cached_vectorToTarget;
  } catch (std::bad_cast& bcex) {
  }
  // Restart interpolated moving platform
  try {
    InterpolatedMovingPlatform& imp =
      dynamic_cast<InterpolatedMovingPlatform&>(p);
    imp.getTrajectoryFunctionRef().setLastTime(0);
  } catch (std::bad_cast& bcex) {
  }
}

void
SimulationPlayer::restartFileMS(helios::filems::FMSFacade& fms)
{
  // Build new facade with updated output directory
  std::shared_ptr<helios::filems::FMSFacade> newFacade =
    helios::filems::FMSFacadeFactory().buildFacade(
      fms.write.getOutDir(),
      fms.write.getMeasurementWriterLasScale(),
      fms.write.isMeasurementWriterLasOutput(),
      fms.write.isMeasurementWriterLas10(),
      fms.write.isMeasurementWriterZipOutput(),
      fms.write.isSplitByChannel(),
      *static_cast<SurveyPlayback&>(sim).mSurvey,
      false);
  // Replace writer of current facade with writer from new facade
  fms.write = newFacade->write;
}

void
SimulationPlayer::restartScanner(Scanner& sc)
{
  // Restart scanner head
  ScannerHead& sh = *sc.getScannerHead();
  sh.setCurrentRotateAngle_rad(sh.getRotateStart());
}

void
SimulationPlayer::restartScene(Scene& scene, bool const keepCRS)
{
  // Discard scene parts that should be null for the next play, and
  // get primitives from current scene parts (thus, those that belong to
  // non-existent scene parts will be discarded)
  std::vector<std::shared_ptr<ScenePart>> newParts;
  std::vector<Primitive*> newPrims;
  glm::dvec3 const oldCRSCenter = scene.getBBoxCRS()->getCentroid();
  glm::dvec3 const oldFinalCenter = scene.getBBox()->getCentroid();
  AABB const oldCRSBBox = *scene.getBBoxCRS();
  for (std::shared_ptr<ScenePart> sp : scene.parts) {
    // Handle scene parts that need to be discarded
    if (sp->sorh != nullptr && sp->sorh->needsDiscardOnReplay() &&
        !sp->isNull()) {
      for (Primitive* p : sp->sorh->getBaselinePrimitives())
        delete p;
      for (Primitive* p : sp->mPrimitives)
        delete p;
      if (sp->sorh->hasNoFuture())
        sp->sorh = nullptr;
      else {
        sp->sorh->setNull(true);
        sp->sorh->getBaselinePrimitives().clear();
        sp->mPrimitives.clear();
        newParts.push_back(sp);
      }
      continue;
    }
    // Handle scene parts that must be preserved
    if (sp->sorh == nullptr ||
        (sp->sorh != nullptr && !sp->sorh->isOnSwapFirstPlay())) {
      // Undo old CRS shift on primitives before scene update and reload
      for (Primitive* p : sp->mPrimitives) {
        Vertex* v = p->getVertices();
        for (size_t i = 0; i < p->getNumVertices(); ++i) {
          v[i].pos = v[i].pos + oldCRSCenter;
        }
      }
    }
    // Handle scene parts who are in the first play after a swap
    if (sp->sorh != nullptr && sp->sorh->isOnSwapFirstPlay()) {
      ScenePart::computeTransformations(sp, sp->sorh->isHolistic());
      sp->sorh->setOnSwapFirstPlay(false);
      if (!sp->mPrimitives.empty()) {
        sp->sorh->setNull(false);
        sp->sorh->setDiscardOnReplay(false);
      }
    }
    // Prepare new data for scene
    newParts.push_back(sp);
    newPrims.insert(
      newPrims.cend(), sp->mPrimitives.begin(), sp->mPrimitives.end());
  }
  scene.parts = newParts;
  scene.primitives = newPrims;
  // Apply default reflectances when needed
  for (Primitive* p : scene.primitives) {
    Material& mat = *p->material;
    if (std::isnan(mat.reflectance)) {
      mat.reflectance = scene.getDefaultReflectance();
    }
  }
  // Reload scene (without KDGrove rebuilding)
  std::shared_ptr<KDGroveFactory> kdgf = scene.getKDGroveFactory();
  scene.setKDGroveFactory(nullptr);
  scene.finalizeLoading(false);
  // Keep CRS if requested
  if (keepCRS) {
    // Undo current CRS shift on primitives
    glm::dvec3 const currentDiff = scene.getBBoxCRS()->getCentroid();
    double xmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymin = xmin, ymax = xmax, zmin = xmin, zmax = xmax;
    for (std::shared_ptr<ScenePart> sp : scene.parts) {
      for (Primitive* p : sp->mPrimitives) {
        Vertex* v = p->getVertices();
        for (size_t i = 0; i < p->getNumVertices(); ++i) {
          v[i].pos = v[i].pos + currentDiff;
        }
        p->update();
        // Also, find min and max coordinates
        for (size_t i = 0; i < p->getNumVertices(); ++i) {
          glm::dvec3 const pos = v[i].pos;
          if (pos.x < xmin)
            xmin = pos.x;
          if (pos.x > xmax)
            xmax = pos.x;
          if (pos.y < ymin)
            ymin = pos.y;
          if (pos.y > ymax)
            ymax = pos.y;
          if (pos.z < zmin)
            zmin = pos.z;
          if (pos.z > zmax)
            zmax = pos.z;
        }
      }
    }
    // Compute the size of the new bounding box
    glm::dvec3 const oldCRSMin = oldCRSBBox.getMin();
    glm::dvec3 const oldCRSMax = oldCRSBBox.getMax();
    if (oldCRSMin.x < xmin)
      xmin = oldCRSMin.x;
    if (oldCRSMin.y < ymin)
      ymin = oldCRSMin.y;
    if (oldCRSMin.z < zmin)
      zmin = oldCRSMin.z;
    if (oldCRSMax.x > xmax)
      xmax = oldCRSMax.x;
    if (oldCRSMax.y > ymax)
      ymax = oldCRSMax.y;
    if (oldCRSMax.z > zmax)
      zmax = oldCRSMax.z;
    glm::dvec3 const minOffset =
      glm::abs(oldCRSCenter - glm::dvec3(xmin, ymin, zmin));
    glm::dvec3 const maxOffset =
      glm::abs(oldCRSCenter - glm::dvec3(xmax, ymax, zmax));
    glm::dvec3 const halfSize(std::max(minOffset.x, maxOffset.x),
                              std::max(minOffset.y, maxOffset.y),
                              std::max(minOffset.z, maxOffset.z));
    // Calculate new CRS bounding box
    glm::dvec3 const newCRSMin = oldCRSCenter - halfSize;
    glm::dvec3 const newCRSMax = oldCRSCenter + halfSize;
    std::shared_ptr<AABB> newCRS = std::make_shared<AABB>(newCRSMin, newCRSMax);
    // Calculate new final bounding box
    glm::dvec3 const newBBoxMin = oldFinalCenter - halfSize;
    glm::dvec3 const newBBoxMax = oldFinalCenter + halfSize;
    std::shared_ptr<AABB> newBBox =
      std::make_shared<AABB>(newBBoxMin, newBBoxMax);
    // Apply new CRS shift on primitives
    for (std::shared_ptr<ScenePart> sp : scene.parts) {
      for (Primitive* p : sp->mPrimitives) {
        Vertex* v = p->getVertices();
        for (size_t i = 0; i < p->getNumVertices(); ++i) {
          v[i].pos = v[i].pos - oldCRSCenter;
        }
        p->update();
      }
    }
    // Assign bounding boxes to scene
    scene.setBBoxCRS(newCRS);
    scene.setBBox(newBBox);
    // Report
    logging::DEBUG("SimulationPlayer::restartScene kept the scene's CRS.");
  }
  // Rebuild KDGrove
  scene.setKDGroveFactory(kdgf);
  scene.buildKDGroveWithLog(false);
}

void
SimulationPlayer::restartSimulation(Simulation& sim)
{
  // Restart survey playback attributes
  logging::DEBUG("Restarting survey playback attributes ...");
  SurveyPlayback& sp = static_cast<SurveyPlayback&>(sim);
  sp.progress = 0;
  sp.legProgress = 0;
  sp.mLegStarted = true;
  sp.legProgress = 0;
  sp.legStartTime_ns =
    duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
  sp.timeStart_ns = sp.legStartTime_ns;
  sp.elapsedLength = 0;
  // Restart simulation attributes
  logging::DEBUG("Restarting simulation attributes ...");
  sim.finished = false;
  sim.mStopped = false;
  sim.mCurrentLegIndex = 0;
  // Restart simulation step loop (i.e., time)
  logging::DEBUG("Restarting simulation step loop ...");
  sim.getStepLoop().setCurrentStep(0);
  // Start first leg
  logging::DEBUG("Restarting first leg ...");
  sp.startLeg(sp.mCurrentLegIndex, true);
}

bool
SimulationPlayer::isKeepCRS(
  std::vector<std::shared_ptr<ScenePart>> const& sorObjects)
{
  // Check Keep CRS flag for each swap-on-repeat handler
  bool anyFalse = false, anyTrue = false;
  for (std::shared_ptr<ScenePart> const& sp : sorObjects) {
    if (!sp->sorh->isKeepCRS())
      anyFalse = true;
    else
      anyTrue = true;
  }
  // Report that both trues and falses were found
  if (anyFalse && anyTrue) {
    logging::WARN(
      "SimulationPlayer::isKeepCRS found at least one "
      "SwapOnRepeatHandler with a KeepCRS flag set to true and another "
      "one with the flag set to false.\n"
      "Consequently, the CRS of the current scene will not be kept "
      "because at least one flag is set to false.");
  }
  // Return true to keep CRS only if no SoRH has the flag to false
  return !anyFalse;
}
