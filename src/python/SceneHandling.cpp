#include <DetailedVoxelLoader.h>
#include <GeoTiffFileLoader.h>
#include <KDTreeFactoryMaker.h>
#include <MaterialsFileReader.h>
#include <SceneHandling.h>
#include <SerialSceneWrapper.h>
#include <SpectralLibrary.h>
#include <WavefrontObjFileLoader.h>
#include <XYZPointCloudFileLoader.h>
#include <XmlAssetsLoader.h>

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

  scene->setKDGroveFactory(std::make_shared<KDGroveFactory>(makeKDTreeFactory(
    kdtFactoryType, kdtNumJobs, kdtGeomJobs, kdtSAHLossNodes)));
  // Call scene finalization
  if (!scene->finalizeLoading()) {
    throw std::runtime_error("Finalizing the scene failed.");
  }
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
  try {
    std::shared_ptr<ScenePart> sp(loader.run());

    // Connect all primitives to their scene part
    for (auto p : sp->mPrimitives)
      p->part = sp;

    // Object lifetime caveat: prevent destructor from deleting these
    // primitives.
    loader.primsOut = nullptr;

    return sp;
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Failed to read OBJ scene part '" << filePath << "': " << e.what();
    logging::ERR(ss.str());
    throw;
  }
}

std::shared_ptr<ScenePart>
readTiffScenePart(std::string filePath, std::vector<std::string> assetsPath)
{

  GeoTiffFileLoader loader;
  loader.params["filepath"] = filePath;
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
  sp->bound = nullptr;
}

void
scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor)
{
  for (auto p : sp->mPrimitives)
    p->scale(scaleFactor);
  sp->bound = nullptr;
}

void
translateScenePart(std::shared_ptr<ScenePart> sp, glm::dvec3 offset)
{
  for (auto p : sp->mPrimitives)
    p->translate(offset);
  sp->bound = nullptr;
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

void
makeInterpolatedShift(Survey& survey,
                      InterpolatedMovingPlatformEgg& ip,
                      glm::dvec3 shift)
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
    throw HeliosException("Failed to create trajectory interpolator: " +
                          std::string(ex.what()));
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
    } else {
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
    } else {
      xEnd = (*trajInterp)(arma::max(ip.tdm->getTimeVector()));
      leg.mTrajectorySettings->tEnd = ip.tdm->getTimeVector().back();
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
    survey.legs.insert(survey.legs.begin() + i + 1, stopLeg);
    ++i;
    ++original;
    // Insert teleport to start leg (after stop leg), if requested
    if (leg.mTrajectorySettings->teleportToStart) {
      auto startLeg = std::make_shared<Leg>(*stopLeg);
      startLeg->mPlatformSettings->x = leg.mPlatformSettings->x;
      startLeg->mPlatformSettings->y = leg.mPlatformSettings->y;
      startLeg->mPlatformSettings->z = leg.mPlatformSettings->z;
      startLeg->mTrajectorySettings->teleportToStart = true;
      leg.mTrajectorySettings->teleportToStart = false;
      survey.legs.insert(survey.legs.begin() + (i - 1), startLeg);
      ++i;
      ++original;
    }
  }
}

void
makeSceneShift(Survey& survey)
{
  glm::dvec3 shift = survey.scanner->platform->scene->getShift();
  // Apply changes to interpolated charachteristics, if any
  if (auto ip = std::dynamic_pointer_cast<InterpolatedMovingPlatformEgg>(
        survey.scanner->platform)) {
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
        auto groundZ =
          survey.scanner->platform->scene->getGroundPointAt(platformPos).z;
        platformPos.z = groundZ;
      }
      leg.mPlatformSettings->setPosition(platformPos);
    }

    if (leg.mScannerSettings) {
      std::shared_ptr<ScannerSettings> default_settings =
        std::make_shared<ScannerSettings>();
      auto currentSettings = survey.scanner->retrieveCurrentSettings();
      std::unordered_set<std::string> scannerFields;
      findNonDefaultScannerSettings(leg.mScannerSettings,
                                    default_settings,
                                    default_settings->id,
                                    scannerFields);
      leg.mScannerSettings =
        currentSettings->cherryPick(leg.mScannerSettings, scannerFields);
    }
  }
}

void
addScenePartToScene(std::shared_ptr<StaticScene> scene,
                    std::shared_ptr<ScenePart> scenePart)
{
  invalidateStaticScene(scene);
  scene->setKDGrove(nullptr);
  for (auto& sp : scene->parts) {
    // Append as a static object
    scene->appendStaticObject(sp);

    // Add scene part primitives to the scene
    scene->primitives.insert(
      scene->primitives.end(), sp->mPrimitives.begin(), sp->mPrimitives.end());
  }
  // Add the new scene part
  for (Primitive* primitive : scenePart->mPrimitives) {
    if (primitive->part == nullptr) {
      primitive->part = scenePart;
    }
  }
  scene->appendStaticObject(scenePart);
  scene->primitives.insert(scene->primitives.end(),
                           scenePart->mPrimitives.begin(),
                           scenePart->mPrimitives.end());
  scene->registerParts();
  invalidateStaticScene(scene); // invalidate it for further finalization
}

std::shared_ptr<Material>
readMaterialFromFile(std::string materialPath,
                     std::vector<std::string> assetsPath,
                     std::string materialId)
{
  fs::path searchfile(materialPath);
  if (searchfile.is_relative()) {
    for (const auto path : assetsPath) {
      if (fs::exists(fs::path(path) / searchfile)) {
        searchfile = fs::path(path) / searchfile;
        break;
      }
    }
  }
  std::string resolved_path = searchfile.string();
  if (resolved_path.empty()) {
    throw std::runtime_error("Material file not found: " + materialPath);
  }
  auto mats = MaterialsFileReader::loadMaterials(resolved_path);

  if (mats.empty()) {
    throw std::runtime_error("No materials found in material file: " +
                             materialPath);
  }

  if (mats.find(materialId) == mats.end()) {
    throw std::runtime_error("Material with name: '" + materialId +
                             "' not found in material file: " + materialPath);
  }
  auto it = mats.find(materialId);
  if (it == mats.end()) {
    throw std::runtime_error("Material with name: '" + materialId +
                             "' not found in material file: " + materialPath);
  }

  return it->second;
}

void
changeMaterialInstance(std::shared_ptr<ScenePart> scenePart,
                       const std::string& oldName,
                       std::shared_ptr<Material> newMaterial)
{
  bool found = false;

  for (auto& prim : scenePart->mPrimitives) {
    if (prim->material && prim->material->name == oldName) {
      prim->material = newMaterial;
      found = true;
    }
  }
  if (!found) {
    throw std::runtime_error("Material with name '" + oldName +
                             "' not found in the scene part.");
  }
}

void
applyMaterialToPrimitivesRange(std::shared_ptr<ScenePart> scenePart,
                               std::shared_ptr<Material> material,
                               size_t start,
                               size_t stop)
{
  size_t n = scenePart->mPrimitives.size();
  if (start >= n - 1 || stop >= n || start >= stop) {
    throw std::out_of_range(
      "Invalid range for applying material to primitives.");
  }
  for (size_t i = start; i <= stop; ++i) {
    scenePart->mPrimitives[i]->material = material;
  }
}

void
applyMaterialToPrimitivesIndices(std::shared_ptr<ScenePart> scenePart,
                                 std::shared_ptr<Material> material,
                                 const std::vector<size_t>& indices)
{
  for (const auto& index : indices) {
    if (index >= scenePart->mPrimitives.size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " is out of range for primitives.");
    }
    scenePart->mPrimitives[index]->material = material;
  }
}

std::vector<std::pair<std::string, std::shared_ptr<Material>>>
getMaterialsMap(const std::shared_ptr<ScenePart>& part)
{
  std::vector<std::pair<std::string, std::shared_ptr<Material>>> out;
  out.reserve(part->mPrimitives.size());

  std::unordered_set<std::string> seen;
  seen.reserve(part->mPrimitives.size());

  for (const auto& prim : part->mPrimitives) {
    if (!prim->material)
      continue;
    const auto& name = prim->material->name;
    if (seen.insert(name).second) {
      out.emplace_back(name, prim->material);
    }
  }
  return out;
}
