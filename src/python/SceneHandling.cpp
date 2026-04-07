#include <DetailedVoxelLoader.h>
#include <GeoTiffFileLoader.h>
#include <KDTreeFactoryMaker.h>
#include <MaterialsFileReader.h>
#include <SceneHandling.h>
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

  std::shared_ptr<InterpolatedMovingPlatformEgg> pe =
    std::dynamic_pointer_cast<InterpolatedMovingPlatformEgg>(
      survey.scanner->platform);

  if (!pe || !pe->tdm)
    return;

  size_t xIdx = 3, yIdx = 4, zIdx = 5;
  pe->tdm->addToColumn(xIdx, -shift.x);
  pe->tdm->addToColumn(yIdx, -shift.y);
  pe->tdm->addToColumn(zIdx, -shift.z);
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
  if (n == 0 || start >= n || stop >= n || start > stop) {
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

std::shared_ptr<ScenePart>
readNumpyScenePart(const double* data,
                   std::size_t nrows,
                   std::size_t ncols,
                   std::ptrdiff_t rowStrideElems,
                   std::ptrdiff_t colStrideElems,
                   std::vector<std::string> assetsPath,
                   double voxelSize,
                   double maxColorValue,
                   glm::dvec3 defaultNormal,
                   bool sparse,
                   bool estimate_normals,
                   int normalXIndex,
                   int normalYIndex,
                   int normalZIndex,
                   int rgbRIndex,
                   int rgbGIndex,
                   int rgbBIndex,
                   bool snapNeighborNormal)
{
  XYZPointCloudFileLoader loader;

  if (!data)
    throw std::runtime_error("rows data is null");
  if (ncols < 3)
    throw std::runtime_error("rows must have at least 3 columns: x,y,z");
  const std::size_t N = nrows;
  const std::size_t C = ncols;

  loader.setAssetsDir(assetsPath);
  loader.params["voxelSize"] = voxelSize;
  loader.params["sparse"] = sparse;

  const double effectiveMaxColorValue =
    (maxColorValue != 0.0) ? maxColorValue : 255.0;
  loader.params["maxColorValue"] = effectiveMaxColorValue;

  const bool hasDefaultNormal =
    defaultNormal.x != std::numeric_limits<double>::max() &&
    defaultNormal.y != std::numeric_limits<double>::max() &&
    defaultNormal.z != std::numeric_limits<double>::max();
  if (hasDefaultNormal) {
    loader.params["defaultNormal"] = defaultNormal;
    loader.defaultNormal = glm::normalize(defaultNormal);
    loader.assignDefaultNormal = true;
  }
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
  if (snapNeighborNormal) {
    loader.params["snapNeighborNormal"] = true;
    loader.snapNeighborNormal = true;
  }

  auto at = [&](std::size_t i, std::size_t j) -> double {
    return *(data + i * rowStrideElems + j * colStrideElems);
  };

  loader.primsOut = new ScenePart();
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  mat->name = "default";
  loader.materials[mat->name] = mat;

  loader.minX = loader.minY = loader.minZ = std::numeric_limits<double>::max();
  loader.maxX = loader.maxY = loader.maxZ =
    std::numeric_limits<double>::lowest();

  for (std::size_t i = 0; i < N; ++i) {
    const double x = at(i, 0);
    const double y = at(i, 1);
    const double z = at(i, 2);

    loader.minX = std::min(loader.minX, x);
    loader.minY = std::min(loader.minY, y);
    loader.minZ = std::min(loader.minZ, z);

    loader.maxX = std::max(loader.maxX, x);
    loader.maxY = std::max(loader.maxY, y);
    loader.maxZ = std::max(loader.maxZ, z);
  }

  double halfVoxelSize = voxelSize * 0.5;
  int estimateNormalsForLoader = estimate_normals ? 1 : 0;
  loader.prepareVoxelsGrid(estimateNormalsForLoader, halfVoxelSize);

  const int nxI = normalXIndex;
  const int nyI = normalYIndex;
  const int nzI = normalZIndex;

  const int rI = rgbRIndex;
  const int gI = rgbGIndex;
  const int bI = rgbBIndex;
  const bool rgbInBounds =
    (rI >= 0 && gI >= 0 && bI >= 0 && rI < C && gI < C && bI < C);
  const bool nInBounds =
    (nxI >= 0 && nyI >= 0 && nzI >= 0 && nxI < C && nyI < C && nzI < C);

  std::vector<std::size_t> idx_of_row;
  if (estimate_normals) {
    idx_of_row.resize(N);
    std::size_t I, J, K;
    for (std::size_t i = 0; i < N; ++i) {
      idx_of_row[i] =
        loader.indexFromCoordinates(at(i, 0), at(i, 1), at(i, 2), I, J, K);
    }
  }

  for (std::size_t i = 0; i < N; ++i) {
    const double x = at(i, 0);
    const double y = at(i, 1);
    const double z = at(i, 2);

    double rr = 0.0, gg = 0.0, bb = 0.0;
    if (rgbInBounds) {
      rr = at(i, rI) / effectiveMaxColorValue;
      gg = at(i, gI) / effectiveMaxColorValue;
      bb = at(i, bI) / effectiveMaxColorValue;
    }

    double xnorm = 0.0, ynorm = 0.0, znorm = 0.0;
    if (!estimate_normals) {
      if (nInBounds) {
        xnorm = at(i, nxI);
        ynorm = at(i, nyI);
        znorm = at(i, nzI);
        if (!loader.correctNormal(xnorm, ynorm, znorm))
          continue;
      } else if (hasDefaultNormal) {
        xnorm = loader.defaultNormal.x;
        ynorm = loader.defaultNormal.y;
        znorm = loader.defaultNormal.z;
      }
    }
    loader.digestVoxel(estimateNormalsForLoader,
                       halfVoxelSize,
                       x,
                       y,
                       z,
                       rr,
                       gg,
                       bb,
                       xnorm,
                       ynorm,
                       znorm);
  }

  loader.warnAboutPotentialErrors("numpy");
  loader.postProcess(mat->name, estimateNormalsForLoader);
  if (estimate_normals) {

    std::size_t startIdx = 0;
    while (startIdx < loader.maxNVoxels) {
      std::size_t pointsCount = 0;
      std::size_t endIdx = startIdx;

      for (; endIdx < loader.maxNVoxels; ++endIdx) {
        if (!loader.voxelGrid->hasVoxel(endIdx))
          continue;

        const std::size_t voxelNumPoints =
          loader.voxelGrid->getVoxel(endIdx)->numPoints;
        if (pointsCount + voxelNumPoints > loader.batchSize) {
          if (pointsCount > 0)
            break;

          loader.voxelGrid->setMatrix(endIdx,
                                      new arma::Mat<double>(3, voxelNumPoints));
          loader.voxelGrid->setCursor(endIdx, 0);
          pointsCount = voxelNumPoints;
          ++endIdx;
          break;
        }

        pointsCount += voxelNumPoints;
        loader.voxelGrid->setMatrix(endIdx,
                                    new arma::Mat<double>(3, voxelNumPoints));
        loader.voxelGrid->setCursor(endIdx, 0);
      }
      if (pointsCount == 0 || endIdx == startIdx)
        break;

      for (std::size_t row = 0; row < N; ++row) {
        const std::size_t IDX = idx_of_row[row];
        if (IDX < startIdx || IDX >= endIdx)
          continue;
        loader.voxelGrid->setNextMatrixCol(
          IDX, at(row, 0), at(row, 1), at(row, 2));
      }

      loader._estimateNormals(startIdx, endIdx);
      loader.voxelGrid->deleteMatrices();
      startIdx = endIdx;
    }
  }

  loader.voxelsGridToScenePart();
  loader.loadMaterial();

  std::shared_ptr<ScenePart> sp(loader.primsOut);
  for (auto p : sp->mPrimitives)
    p->part = sp;

  loader.primsOut = nullptr;
  return sp;
}

std::shared_ptr<ScenePart>
readOpen3DMeshScenePart(const double* verticesData,
                        std::size_t nVertices,
                        std::ptrdiff_t vertexRowStrideElems,

                        const int* trianglesData,
                        std::size_t nTriangles,
                        std::ptrdiff_t triangleRowStrideElems,

                        const double* vertexNormalsData,
                        std::ptrdiff_t vertexNormalRowStrideElems,

                        const double* triangleNormalsData,
                        std::ptrdiff_t triangleNormalRowStrideElems,

                        const double* vertexColorsData,
                        std::ptrdiff_t colorRowStrideElems,

                        const double* triangleUvsData,
                        std::ptrdiff_t triangleUvRowStrideElems,

                        std::string upaxis)
{
  if (!verticesData)
    throw std::invalid_argument("verticesData is null");
  if (!trianglesData)
    throw std::invalid_argument("trianglesData is null");

  WavefrontObjFileLoader loader;

  const bool hasVertexNormals = (vertexNormalsData != nullptr);
  const bool hasColors = (vertexColorsData != nullptr);
  const bool hasTriangleNormals = (triangleNormalsData != nullptr);
  const bool hasTextures = (triangleUvsData != nullptr);

  bool yIsUp = false;
  if (upaxis == "y") {
    yIsUp = true;
  } else if (upaxis != "z") {
    throw std::runtime_error("up-axis must be either 'y' or 'z'");
  }

  auto mapPos = [&](std::size_t i) -> glm::dvec3 {
    const double x = *(verticesData + i * vertexRowStrideElems + 0);
    const double y = *(verticesData + i * vertexRowStrideElems + 1);
    const double z = *(verticesData + i * vertexRowStrideElems + 2);
    return yIsUp ? glm::dvec3(x, -z, y) : glm::dvec3(x, y, z);
  };

  auto mapVertexNormal = [&](std::size_t i) -> glm::dvec3 {
    const double x = *(vertexNormalsData + i * vertexNormalRowStrideElems + 0);
    const double y = *(vertexNormalsData + i * vertexNormalRowStrideElems + 1);
    const double z = *(vertexNormalsData + i * vertexNormalRowStrideElems + 2);
    return yIsUp ? glm::dvec3(x, -z, y) : glm::dvec3(x, y, z);
  };

  auto mapTriangleNormal = [&](std::size_t i) -> glm::dvec3 {
    const double x =
      *(triangleNormalsData + i * triangleNormalRowStrideElems + 0);
    const double y =
      *(triangleNormalsData + i * triangleNormalRowStrideElems + 1);
    const double z =
      *(triangleNormalsData + i * triangleNormalRowStrideElems + 2);
    return yIsUp ? glm::dvec3(x, -z, y) : glm::dvec3(x, y, z);
  };

  auto mapColor = [&](std::size_t i) -> Color4f {
    const float r =
      static_cast<float>(*(vertexColorsData + i * colorRowStrideElems + 0));
    const float g =
      static_cast<float>(*(vertexColorsData + i * colorRowStrideElems + 1));
    const float b =
      static_cast<float>(*(vertexColorsData + i * colorRowStrideElems + 2));
    return Color4f(r, g, b, 1.0f);
  };

  auto mapTriangleUv = [&](std::size_t i) -> glm::dvec2 {
    const double u = *(triangleUvsData + i * triangleUvRowStrideElems + 0);
    const double v = *(triangleUvsData + i * triangleUvRowStrideElems + 1);
    return glm::dvec2(u, v);
  };

  auto mat = std::make_shared<Material>();
  mat->name = "default";
  mat->useVertexColors = hasColors;
  loader.materials[mat->name] = mat;

  loader.primsOut->mPrimitives.reserve(nTriangles);

  for (std::size_t fi = 0; fi < nTriangles; ++fi) {
    const int i0 = *(trianglesData + fi * triangleRowStrideElems + 0);
    const int i1 = *(trianglesData + fi * triangleRowStrideElems + 1);
    const int i2 = *(trianglesData + fi * triangleRowStrideElems + 2);

    if (i0 < 0 || i1 < 0 || i2 < 0 ||
        static_cast<std::size_t>(i0) >= nVertices ||
        static_cast<std::size_t>(i1) >= nVertices ||
        static_cast<std::size_t>(i2) >= nVertices) {
      std::stringstream ss;
      ss << "Open3D mesh loader: triangle index out of bounds at face " << fi
         << ": [" << i0 << ", " << i1 << ", " << i2
         << "], nVertices=" << nVertices;
      throw std::out_of_range(ss.str());
    }

    Vertex v0, v1, v2;
    v0.pos = mapPos(static_cast<std::size_t>(i0));
    v1.pos = mapPos(static_cast<std::size_t>(i1));
    v2.pos = mapPos(static_cast<std::size_t>(i2));

    if (hasVertexNormals) {
      v0.normal = mapVertexNormal(static_cast<std::size_t>(i0));
      v1.normal = mapVertexNormal(static_cast<std::size_t>(i1));
      v2.normal = mapVertexNormal(static_cast<std::size_t>(i2));
    } else if (hasTriangleNormals) {
      glm::dvec3 triNormal = mapTriangleNormal(fi);
      v0.normal = triNormal;
      v1.normal = triNormal;
      v2.normal = triNormal;
    }

    if (hasColors) {
      v0.color = mapColor(static_cast<std::size_t>(i0));
      v1.color = mapColor(static_cast<std::size_t>(i1));
      v2.color = mapColor(static_cast<std::size_t>(i2));
    }

    if (hasTextures) {
      const std::size_t uvBase = 3 * fi;
      v0.texcoords = mapTriangleUv(uvBase + 0);
      v1.texcoords = mapTriangleUv(uvBase + 1);
      v2.texcoords = mapTriangleUv(uvBase + 2);
    }

    Triangle* tri = new Triangle(v0, v1, v2);
    tri->material = mat;
    loader.primsOut->mPrimitives.push_back(tri);
  }

  loader.primsOut->subpartLimit.push_back(loader.primsOut->mPrimitives.size());

  std::shared_ptr<ScenePart> sp(loader.primsOut);
  for (auto p : sp->mPrimitives)
    p->part = sp;

  loader.primsOut = nullptr;
  return sp;
}

ScenePartVisualizationBuffers
extractScenePartVisualizationBuffers(ScenePart const& sp)
{
  std::vector<double> triangleVerticesFlat;
  std::vector<int> triangleIndicesFlat;
  std::vector<double> voxelCentersFlat;

  triangleVerticesFlat.reserve(sp.mPrimitives.size() * 9);
  triangleIndicesFlat.reserve(sp.mPrimitives.size() * 3);
  voxelCentersFlat.reserve(sp.mPrimitives.size() * 3);

  int nextVertexIndex = 0;

  for (Primitive* primitive : sp.mPrimitives) {
    if (primitive == nullptr) {
      continue;
    }

    if (auto const* tri = dynamic_cast<Triangle const*>(primitive)) {
      for (int i = 0; i < 3; ++i) {
        Vertex const& v = tri->verts[i];
        triangleVerticesFlat.push_back(v.pos.x);
        triangleVerticesFlat.push_back(v.pos.y);
        triangleVerticesFlat.push_back(v.pos.z);
      }

      triangleIndicesFlat.push_back(nextVertexIndex + 0);
      triangleIndicesFlat.push_back(nextVertexIndex + 1);
      triangleIndicesFlat.push_back(nextVertexIndex + 2);
      nextVertexIndex += 3;
      continue;
    }

    if (auto const* dv = dynamic_cast<DetailedVoxel const*>(primitive)) {
      voxelCentersFlat.push_back(dv->v.pos.x);
      voxelCentersFlat.push_back(dv->v.pos.y);
      voxelCentersFlat.push_back(dv->v.pos.z);
      continue;
    }

    if (auto const* voxel = dynamic_cast<Voxel const*>(primitive)) {
      voxelCentersFlat.push_back(voxel->v.pos.x);
      voxelCentersFlat.push_back(voxel->v.pos.y);
      voxelCentersFlat.push_back(voxel->v.pos.z);
      continue;
    }
  }

  ssize_t const numTriangleVertices =
    static_cast<ssize_t>(triangleVerticesFlat.size() / 3);
  ssize_t const numTriangles =
    static_cast<ssize_t>(triangleIndicesFlat.size() / 3);
  ssize_t const numVoxelCenters =
    static_cast<ssize_t>(voxelCentersFlat.size() / 3);

  py::array_t<double> triangleVertices(
    { numTriangleVertices, static_cast<ssize_t>(3) });
  py::array_t<int> triangleIndices({ numTriangles, static_cast<ssize_t>(3) });
  py::array_t<double> voxelCenters(
    { numVoxelCenters, static_cast<ssize_t>(3) });

  if (!triangleVerticesFlat.empty()) {
    std::memcpy(triangleVertices.mutable_data(),
                triangleVerticesFlat.data(),
                triangleVerticesFlat.size() * sizeof(double));
  }

  if (!triangleIndicesFlat.empty()) {
    std::memcpy(triangleIndices.mutable_data(),
                triangleIndicesFlat.data(),
                triangleIndicesFlat.size() * sizeof(int));
  }

  if (!voxelCentersFlat.empty()) {
    std::memcpy(voxelCenters.mutable_data(),
                voxelCentersFlat.data(),
                voxelCentersFlat.size() * sizeof(double));
  }

  return ScenePartVisualizationBuffers{
    std::move(triangleVertices),
    std::move(triangleIndices),
    std::move(voxelCenters),
  };
}
