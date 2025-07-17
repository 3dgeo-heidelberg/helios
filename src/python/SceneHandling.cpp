#include <DetailedVoxelLoader.h>
#include <GeoTiffFileLoader.h>
#include <KDTreeFactoryMaker.h>
#include <SceneHandling.h>
#include <SerialSceneWrapper.h>
#include <SpectralLibrary.h>
#include <WavefrontObjFileLoader.h>
#include <XYZPointCloudFileLoader.h>



#include <fluxionum/DiffDesignMatrixInterpolator.h>
#include <fluxionum/ParametricLinearPiecesFunction.h>
#include <logging.hpp>
#include <platform/InterpolatedMovingPlatformEgg.h>

std::shared_ptr<KDTreeFactory>
makeKDTreeFactory(int kdtFactoryType,
                  int kdtNumJobs,
                  int kdtGeomJobs,
                  int kdtSAHLossNodes)
{
  if (kdtFactoryType == 1) { // Simple
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeSimpleMultiThread(kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeSimple();
  } else if (kdtFactoryType == 2) { // SAH
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeSAHMultiThread(
        kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeSAH(kdtSAHLossNodes);
  } else if (kdtFactoryType == 3) { // Axis SAH
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeAxisSAHMultiThread(
        kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeAxisSAH(kdtSAHLossNodes);
  }

  if (kdtNumJobs > 1) {
    return KDTreeFactoryMaker::makeFastSAHMultiThread(
      kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
  }
  return KDTreeFactoryMaker::makeFastSAH(kdtSAHLossNodes);
}

void
finalizeStaticScene(std::shared_ptr<StaticScene> scene,
                    int kdtFactoryType,
                    int kdtNumJobs,
                    int kdtGeomJobs,
                    int kdtSAHLossNodes)
{
  // Loop over all scene parts and perform their final processing
  for (auto& sp : scene->parts) {
    // Append as a static object
    scene->appendStaticObject(sp);

    // Add scene part primitives to the scene
    scene->primitives.insert(
      scene->primitives.end(), sp->mPrimitives.begin(), sp->mPrimitives.end());
  }

  // Call scene finalization
  if (!scene->finalizeLoading()) {
    throw std::runtime_error("Finalizing the scene failed.");
  }

  // Build KDGroveFactory
  scene->setKDGroveFactory(std::make_shared<KDGroveFactory>(makeKDTreeFactory(
    kdtFactoryType, kdtNumJobs, kdtGeomJobs, kdtSAHLossNodes)));
  scene->buildKDGroveWithLog();
}

void
invalidateStaticScene(std::shared_ptr<StaticScene> scene)
{
  scene->clearStaticObjects();
  scene->setKDGroveFactory(nullptr);
  scene->primitives.clear();
}

void
setSceneReflectances(std::shared_ptr<StaticScene> scene,
                     std::vector<std::string> assetsPath,
                     float wavelength)
{
  SpectralLibrary spectralLibrary(wavelength, assetsPath, "spectra");
  spectralLibrary.readReflectances();
  spectralLibrary.setReflectances(scene.get());
  scene->setDefaultReflectance(spectralLibrary.getDefaultReflectance());
}

std::shared_ptr<ScenePart>
readObjScenePart(std::string filePath,
                 std::vector<std::string> assetsPath,
                 std::string upaxis)
{
  WavefrontObjFileLoader loader;
  loader.params["filepath"] = filePath;
  loader.params["up"] = upaxis;
  loader.setAssetsDir(assetsPath);
  std::shared_ptr<ScenePart> sp(loader.run());

  // Connect all primitives to their scene part
  for (auto p : sp->mPrimitives)
    p->part = sp;

  // Object lifetime caveat! Settings primsOut to nullptr will prevent the
  // loader destructor from deleting the primitives.
  loader.primsOut = nullptr;

  return sp;
}

std::shared_ptr<ScenePart>
readTiffScenePart(std::string filePath)
{

  GeoTiffFileLoader loader;
  loader.params["filepath"] = filePath;
  std::shared_ptr<ScenePart> sp(loader.run());

  // Connect all primitives to their scene part
  for (auto p : sp->mPrimitives)
    p->part = sp;

  // Object lifetime caveat! Settings primsOut to nullptr will prevent the
  // loader destructor from deleting the primitives.
  loader.primsOut = nullptr;

  return sp;
}

std::shared_ptr<ScenePart>
readXYZScenePart(std::string filePath,
                 std::vector<std::string> assetsPath,
                 std::string separator,
                 double voxelSize,
                 double maxColorValue,
                 glm::dvec3 defaultNormal,
                 bool sparse,
                 int estimate_normals,
                 int normalXIndex,
                 int normalYIndex,
                 int normalZIndex,
                 int rgbRIndex,
                 int rgbGIndex,
                 int rgbBIndex,
                 bool snapNeighborNormal)
{

  XYZPointCloudFileLoader loader;
  loader.params["filepath"] = filePath;
  loader.params["separator"] = separator;
  loader.params["voxelSize"] = voxelSize;
  if (maxColorValue != 0.0)
    loader.params["maxColorValue"] = maxColorValue;

  if (defaultNormal.x != std::numeric_limits<double>::max() &&
      defaultNormal.y != std::numeric_limits<double>::max() &&
      defaultNormal.z != std::numeric_limits<double>::max()) {
    loader.params["defaultNormal"] = defaultNormal;
  }

  if (sparse)
    loader.params["sparse"] = sparse;

  if (estimate_normals != 0)
    loader.params["estimateNormals"] = estimate_normals;

  if (normalXIndex != 3) {
    loader.params["normalXIndex"] = normalXIndex;
    loader.params["normalYIndex"] = normalYIndex;
    loader.params["normalZIndex"] = normalZIndex;
  }

  if (rgbRIndex != 6) {
    loader.params["rgbRIndex"] = rgbRIndex;
    loader.params["rgbGIndex"] = rgbGIndex;
    loader.params["rgbBIndex"] = rgbBIndex;
  }

  if (snapNeighborNormal)
    loader.params["snapNeighborNormal"] = snapNeighborNormal;

  loader.setAssetsDir(assetsPath);

  std::shared_ptr<ScenePart> sp(loader.run());
  for (auto p : sp->mPrimitives)
    p->part = sp;

  // Object lifetime caveat! Settings primsOut to nullptr will prevent the
  // loader destructor from deleting the primitives.
  loader.primsOut = nullptr;

  return sp;
}

std::shared_ptr<ScenePart>
readVoxScenePart(std::string filePath,
                 std::vector<std::string> assetsPath,
                 std::string intersectionMode,
                 double intersectionArgument,
                 bool randomShift,
                 std::string ladlutPath)
{
  if (intersectionMode == "fixed" && intersectionArgument != 0.0) {
    throw std::invalid_argument("'intersectionArgument' must not be provided "
                                "when 'intersectionMode' is 'fixed'.");
  }
  DetailedVoxelLoader loader;
  loader.params["filepath"] = filePath;
  loader.params["intersectionMode"] = intersectionMode;
  if (intersectionMode == "scaled")
    loader.params["intersectionArgument"] = intersectionArgument;
  if (randomShift)
    loader.params["randomShift"] = randomShift;
  if (!ladlutPath.empty())
    loader.params["ladlut"] = ladlutPath;

  loader.setAssetsDir(assetsPath);
  std::shared_ptr<ScenePart> sp(loader.run());

  for (auto p : sp->mPrimitives)
    p->part = sp;

  loader.primsOut = nullptr;

  return sp;
}

void
rotateScenePart(std::shared_ptr<ScenePart> sp, Rotation rotation)
{
  for (auto p : sp->mPrimitives)
    p->rotate(rotation);
}

void
scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor)
{
  for (auto p : sp->mPrimitives)
    p->scale(scaleFactor);
}

void
translateScenePart(std::shared_ptr<ScenePart> sp, glm::dvec3 offset)
{
  for (auto p : sp->mPrimitives)
    p->translate(offset);
}

void
writeSceneToBinary(const std::string& filename,
                   std::shared_ptr<Scene> scene,
                   bool isDynScene)
{

  SerialSceneWrapper::SceneType sceneType;
  if (isDynScene) {
    sceneType = SerialSceneWrapper::SceneType::DYNAMIC_SCENE;
  } else {
    sceneType = SerialSceneWrapper::SceneType::STATIC_SCENE;
  }
  SerialSceneWrapper(sceneType, scene.get()).writeScene(filename);
}

std::shared_ptr<Scene>
readSceneFromBinary(const std::string& filename)
{
  fs::path filePath = fs::path(filename);

  if (!fs::is_regular_file(filePath)) {
    throw std::runtime_error("Binary Scene file does not exist: " +
                             filePath.string());
  }

  SerialSceneWrapper* ssw = SerialSceneWrapper::readScene(filePath.string());
  std::shared_ptr<Scene> scene = std::shared_ptr<Scene>(ssw->getScene());
  delete ssw;

  return scene;
}

void
findNonDefaultScannerSettings(std::shared_ptr<ScannerSettings> base,
                              std::shared_ptr<ScannerSettings> ref,
                              std::string const defaultTemplateId,
                              std::unordered_set<std::string>& fields)
{
  if (ref->id != defaultTemplateId)
    fields.insert("baseTemplate");
  if (base->active != ref->active)
    fields.insert("active");
  if (base->headRotatePerSec_rad != ref->headRotatePerSec_rad)
    fields.insert("headRotatePerSec_rad");
  if (base->headRotateStart_rad != ref->headRotateStart_rad)
    fields.insert("headRotateStart_rad");
  if (base->headRotateStop_rad != ref->headRotateStop_rad)
    fields.insert("headRotateStop_rad");
  if (base->pulseFreq_Hz != ref->pulseFreq_Hz)
    fields.insert("pulseFreq_Hz");
  if (base->scanAngle_rad != ref->scanAngle_rad)
    fields.insert("scanAngle_rad");
  if (base->verticalAngleMin_rad != ref->verticalAngleMin_rad)
    fields.insert("verticalAngleMin_rad");
  if (base->verticalAngleMax_rad != ref->verticalAngleMax_rad)
    fields.insert("verticalAngleMax_rad");
  if (base->scanFreq_Hz != ref->scanFreq_Hz)
    fields.insert("scanFreq_Hz");
  if (base->beamDivAngle != ref->beamDivAngle)
    fields.insert("beamDivAngle");
  if (base->trajectoryTimeInterval != ref->trajectoryTimeInterval)
    fields.insert("trajectoryTimeInterval");
  if (base->verticalResolution_rad != ref->verticalResolution_rad)
    fields.insert("verticalResolution_rad");
  if (base->horizontalResolution_rad != ref->horizontalResolution_rad)
    fields.insert("horizontalResolution_rad");
}

void makeInterpolatedShift(
  Survey & survey,
  InterpolatedMovingPlatformEgg & ip,
  glm::dvec3 shift
  )
{
  size_t xi = ip.tdm->translateColumnNameToIndex("x");
  size_t yi = ip.tdm->translateColumnNameToIndex("y");
  size_t zi = ip.tdm->translateColumnNameToIndex("z");
  // Obtain trajectory interpolator, if any
  std::shared_ptr<fluxionum::ParametricLinearPiecesFunction<double, double>>
    trajInterp = nullptr;
  try {
    trajInterp = std::make_shared<
      fluxionum::ParametricLinearPiecesFunction<double, double>>(
      fluxionum::DiffDesignMatrixInterpolator::
        makeParametricLinearPiecesFunction(*ip.ddm, *ip.tdm));
  } catch (std::exception& ex) {
    throw HeliosException("Failed to create trajectory interpolator: " + std::string(ex.what()));
  }
  
  size_t original = survey.legs.size();
  for (size_t i = 0; i < original; ++i) {
    auto& leg = *survey.legs[i];
    if (!leg.mTrajectorySettings) {
      throw HeliosException("Leg " + std::to_string(i) +
                            " does not have trajectory settings, "
                            "cannot apply scene shift.");
    }
    // Configure start
    arma::Col<double> xStart;
    if (leg.mTrajectorySettings->hasStartTime()) {
      leg.mTrajectorySettings->tStart -= ip.startTime;
      xStart = (*trajInterp)(leg.mTrajectorySettings->tStart);
    }
    else {
        xStart = (*trajInterp)(0);
        leg.mTrajectorySettings->tStart = ip.tdm->getTimeVector().front();
    }
    leg.mPlatformSettings->x = xStart[xi];
    leg.mPlatformSettings->y = xStart[yi];
    leg.mPlatformSettings->z = xStart[zi];

    // Configure end
    arma::Col<double> xEnd;
    if (leg.mTrajectorySettings->hasEndTime()) {
      leg.mTrajectorySettings->tEnd -= ip.startTime;
      xEnd = (*trajInterp)(leg.mTrajectorySettings->tEnd);
    }
    else {
        xEnd = (*trajInterp)( arma::max(ip.tdm->getTimeVector()) );
        leg.mTrajectorySettings->tEnd =
          ip.tdm->getTimeVector().back();
    }
    
    // Insert stop leg
    auto stopLeg = std::make_shared<Leg>(leg);
    stopLeg->mScannerSettings =
      std::make_shared<ScannerSettings>(*leg.mScannerSettings);
    stopLeg->mScannerSettings->active = false;
    stopLeg->mPlatformSettings =
      std::make_shared<PlatformSettings>(*leg.mPlatformSettings);
    stopLeg->mPlatformSettings->x = xEnd[xi];
    stopLeg->mPlatformSettings->y = xEnd[yi];
    stopLeg->mPlatformSettings->z = xEnd[zi];
    stopLeg->mTrajectorySettings = std::make_shared<TrajectorySettings>();
    //survey.legs.push_back(stopLeg);
    survey.legs.insert(survey.legs.begin() + i + 1, stopLeg);
    ++i; ++original;
    // Insert teleport to start leg (after stop leg), if requested
    if (leg.mTrajectorySettings->teleportToStart) {
      auto startLeg = std::make_shared<Leg>(*stopLeg);
      startLeg->mPlatformSettings->x = leg.mPlatformSettings->x;
      startLeg->mPlatformSettings->y = leg.mPlatformSettings->y;
      startLeg->mPlatformSettings->z = leg.mPlatformSettings->z;
      startLeg->mTrajectorySettings->teleportToStart = true;
      leg.mTrajectorySettings->teleportToStart = false;
      survey.legs.insert( survey.legs.begin() + (i - 1), startLeg );
      ++i; ++original;
    }
  }

}

void makeSceneShift(Survey & survey,
                    bool legNoiseDisabled,
                    bool legRandomOffset,
                    double legRandomOffsetMean,
                    double legRandomOffsetStdev)
{
  glm::dvec3 shift = survey.scanner->platform->scene->getShift();
  // Prepare normal distribution if necessary
  RandomnessGenerator<double> rg(*DEFAULT_RG);
  if (legRandomOffset && !legNoiseDisabled) {
    rg.computeNormalDistribution(legRandomOffsetMean, legRandomOffsetStdev);
  }
  // Apply changes to interpolated charachteristics, if any
  if (auto ip = std::dynamic_pointer_cast<InterpolatedMovingPlatformEgg>(
    survey.scanner->platform)){
    makeInterpolatedShift(survey, *ip, shift);
  }
  // Apply scene shift to each leg
  size_t n0 = survey.legs.size();  
  for (size_t i = 0; i < n0; ++i) {
    auto& leg = *survey.legs[i];
    // Shift platform settings, if any
    if (leg.mPlatformSettings) {
      glm::dvec3 platformPos = leg.mPlatformSettings->getPosition();
      platformPos -= shift;
      // If specified, move waypoint z coordinate to ground level
      if (leg.mPlatformSettings->onGround) {
        auto groundZ = survey.scanner->platform
                          ->scene->getGroundPointAt(platformPos).z;
        platformPos.z = groundZ;
      }

      // Noise -> add a random offset in x,y,z to the measurements
      if (legRandomOffset && !legNoiseDisabled) {
        platformPos += glm::dvec3(rg.normalDistributionNext(),
                          rg.normalDistributionNext(),
                          rg.normalDistributionNext());
      }
      leg.mPlatformSettings->setPosition(platformPos);
    }

    if (leg.mScannerSettings) {
      std::shared_ptr<ScannerSettings> default_settings = std::make_shared<ScannerSettings>();
      auto currentSettings =
                survey.scanner->retrieveCurrentSettings();
      std::unordered_set<std::string> scannerFields;
      findNonDefaultScannerSettings(
        leg.mScannerSettings, default_settings, default_settings->id, scannerFields);
        leg.mScannerSettings = currentSettings->cherryPick(
          leg.mScannerSettings, scannerFields); 
    }
  }
}
