#include <assetloading/geometryfilter/DenseVoxelGrid.h>
#include <assetloading/geometryfilter/SparseVoxelGrid.h>
#include <assetloading/geometryfilter/XYZPointCloudFileLoader.h>

#include <logging.hpp>
#include <util/FileUtils.h>
#include <util/HeliosException.h>

#include "Material.h"
#include "PlaneFitter.h"

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/variant/get.hpp>

#include <boost/filesystem.hpp>
#include <glm/glm.hpp>
namespace fs = boost::filesystem;

// ***  R U N  *** //
// *************** //
ScenePart*
XYZPointCloudFileLoader::run()
{
  // Determine filepath
  std::vector<std::string> filePaths =
    FileUtils::handleFilePath(params, assetsDir);

  // Read separator
  std::string const& pSep = boost::get<std::string const&>(params["separator"]);
  if (!pSep.empty())
    separator = pSep;

  // Read voxel size
  double pVoxelSize = boost::get<double>(params["voxelSize"]);
  if (pVoxelSize != 0)
    voxelSize = pVoxelSize;

  // Read max color value
  maxColorValue = 255.0;
  if (params.find("maxColorValue") != params.end()) {
    double pMaxCol = boost::get<double>(params["maxColorValue"]);
    if (pMaxCol != 0) {
      maxColorValue = pMaxCol;
    }
  }

  // Default normal
  if (params.find("defaultNormal") != params.end()) {
    defaultNormal = boost::get<glm::dvec3>(params["defaultNormal"]);
    defaultNormal = glm::normalize(defaultNormal);
    assignDefaultNormal = true;
  }

  // Parse
  for (std::string const& filePath : filePaths) {
    if (!fs::exists(fs::path(filePath))) { // Error : file not found
      std::stringstream ss;
      ss << "File not found: " << filePath << std::endl;
      logging::ERR(ss.str());
      exit(1);
    }
    lastNumVoxels = primsOut->mPrimitives.size();
    parse(filePath);
    primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
  }

  // Return
  return primsOut;
}

// ***  MAIN PARSING METHODS  *** //
// ****************************** //
void
XYZPointCloudFileLoader::parse(std::string const& filePath)
{
  logging::INFO("Reading point cloud from XYZ file " + filePath + " ...");

  // Initialize counters
  unsafeNormalEstimations = 0;
  discardedPointsByNormal = 0;

  // Material
  logging::INFO("Adding default material");
  Material mat;
  // Legacy default material commented below
  /*mat.useVertexColors = true;
  mat.isGround = true;*/
  materials.insert(materials.end(),
                   std::pair<std::string, Material>(mat.name, mat));

  // Open file input stream
  std::ifstream ifs;
  try {
    ifs = std::ifstream(filePath, std::ifstream::binary);
  } catch (std::exception& e) {
    logging::ERR("Failed to open xyz point cloud file: " + filePath +
                 "\nEXCEPTION: " + e.what());
    exit(-1);
  }

  // First pass
  firstPass(filePath, ifs);

  // Second pass
  secondPass(filePath, mat.name, ifs);

  // Release
  if (voxelGrid != nullptr) {
    voxelGrid->release();
    delete voxelGrid;
    voxelGrid = nullptr;
  }

  // Load material
  loadMaterial();

  // Final report
  std::stringstream ss;
  ss << "Point cloud file read successful ("
     << primsOut->mPrimitives.size() - lastNumVoxels << " voxels)\n";
  if (unsafeNormalEstimations > 0) {
    ss << "\t" << unsafeNormalEstimations
       << " voxels did not have enough points ("
       << minPointsForSafeNormalEstimation << ") to safely "
       << "estimate their normal\n";
  }
  if (discardedPointsByNormal > 0) {
    ss << "\t" << discardedPointsByNormal << " points have a non valid "
       << "normal";
  }
  logging::INFO(ss.str());
  ss.str("");
}

void
XYZPointCloudFileLoader::firstPass(std::string const& filePathString,
                                   std::ifstream& ifs)
{
  // Initial values
  minX = std::numeric_limits<double>::max();
  minY = std::numeric_limits<double>::max();
  minZ = std::numeric_limits<double>::max();
  maxX = std::numeric_limits<double>::lowest();
  maxY = std::numeric_limits<double>::lowest();
  maxZ = std::numeric_limits<double>::lowest();

  // Parse
  std::string line;
  double x, y, z;
  try {
    n = 0;
    while (getline(ifs, line)) {
      if (isLineComment(line))
        continue;
      boost::algorithm::trim(line);
      std::vector<std::string> lineParts;
      boost::split(lineParts, line, boost::is_any_of(separator));
      // ########## BEGIN Read vertex position ##########
      unsigned int size = lineParts.size();
      if (size >= 3) {
        x = std::strtod(lineParts[0].c_str(), nullptr);
        y = std::strtod(lineParts[1].c_str(), nullptr);
        z = std::strtod(lineParts[2].c_str(), nullptr);
        if (x < minX)
          minX = x;
        if (x > maxX)
          maxX = x;
        if (y < minY)
          minY = y;
        if (y > maxY)
          maxY = y;
        if (z < minZ)
          minZ = z;
        if (z > maxZ)
          maxZ = z;
        n++;
      } else {
        std::stringstream ss;
        ss << "XYZPointCloudFileLoader ERROR:\n"
           << "\tRecord/row with no (x,y,z) has been found.";
        logging::ERR(ss.str());
        throw HeliosException(
          "Exception at XYZPointCloudFileLoader::firstPass\n"
          "\tRecord with no (x,y,z)");
      }
    }

    // Compute number of necessary batches
    numBatches = std::ceil((double)n / (double)batchSize);

    // Restore file cursor position to beginning
    restartInputFileStream(ifs);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to read xyz point cloud file \"" << filePathString
       << "\"\nEXCEPTION: " << e.what();
    logging::ERR(ss.str());
    exit(-1);
  }
}

void
XYZPointCloudFileLoader::secondPass(std::string const& filePathString,
                                    std::string const& matName,
                                    std::ifstream& ifs)
{
  // Prepare voxels grid
  int estimateNormals;
  double halfVoxelSize;
  prepareVoxelsGrid(estimateNormals, halfVoxelSize);

  // Fill voxels grid
  fillVoxelsGrid(ifs, estimateNormals, halfVoxelSize, filePathString);

  // Warning about potential specification errors
  warnAboutPotentialErrors(filePathString);

  // Post-processing of voxels
  postProcess(matName, estimateNormals);

  // Normals estimation
  if (estimateNormals > 1)
    XYZPointCloudFileLoader::estimateNormalsBatch(ifs);
  else if (estimateNormals == 1)
    XYZPointCloudFileLoader::estimateNormals(ifs);

  // Compose scene part with voxels
  voxelsGridToScenePart();
}

void
XYZPointCloudFileLoader::loadMaterial()
{
  // Parse materials
  std::vector<std::shared_ptr<Material>> matvec = parseMaterials();
  if (matvec.empty())
    return;

  // Assign material to each detailed voxel
  size_t j, n = primsOut->mPrimitives.size(), m = matvec.size();
  for (size_t i = 0; i < n; i++) {
    Voxel* vxl = (Voxel*)primsOut->mPrimitives[i];
    j = i % m;
    vxl->material = matvec[j];
  }
}

// ***  AUXILIAR PARSING METHODS  *** //
// ********************************** //
void
XYZPointCloudFileLoader::prepareVoxelsGrid(int& estimateNormals,
                                           double& halfVoxelSize)
{
  // Compute voxel grid configuration
  halfVoxelSize = voxelSize / 2.0;
  double deltaX = maxX - minX;
  if (deltaX == 0.0) {
    minX = -halfVoxelSize;
    maxX = halfVoxelSize;
  }
  double deltaY = maxY - minY;
  if (deltaY == 0.0) {
    minY = -halfVoxelSize;
    maxY = halfVoxelSize;
  }
  double deltaZ = maxZ - minZ;
  if (deltaZ == 0.0) {
    minZ = -halfVoxelSize;
    maxZ = halfVoxelSize;
  }
  nx = std::ceil(deltaX / voxelSize);
  if (nx == 0)
    nx = 1;
  ny = std::ceil(deltaY / voxelSize);
  if (ny == 0)
    ny = 1;
  nz = std::ceil(deltaZ / voxelSize);
  if (nz == 0)
    nz = 1;
  nynz = ny * nz;
  maxNVoxels = nx * nynz;
  xCoeff = 1 / voxelSize;
  yCoeff = 1 / voxelSize;
  zCoeff = 1 / voxelSize;

  // Instantiate voxel grid
  if (params.find("sparse") != params.end() &&
      !boost::get<bool>(params["sparse"])) { // Dense voxel grid
    voxelGrid = new DenseVoxelGrid(maxNVoxels);
  } else { // Sparse voxel grid (default)
    voxelGrid = new SparseVoxelGrid(maxNVoxels);
  }

  // Check if voxel grid needs normal estimation or not
  estimateNormals = 0;
  if (params.find("estimateNormals") != params.end()) {
    estimateNormals = boost::get<int>(params["estimateNormals"]);
  }
}

void
XYZPointCloudFileLoader::fillVoxelsGrid(std::ifstream& ifs,
                                        int estimateNormals,
                                        double halfVoxelSize,
                                        std::string const& filePathString)
{
  // Retrieve voxel grid parameters
  std::size_t rgbRIndex = 6;
  std::size_t rgbGIndex = 7;
  std::size_t rgbBIndex = 8;
  std::size_t normalXIndex = 3;
  std::size_t normalYIndex = 4;
  std::size_t normalZIndex = 5;
  if (params.find("rgbRIndex") != params.end()) {
    rgbRIndex = (std::size_t)boost::get<int>(params["rgbRIndex"]);
    rgbGIndex = (std::size_t)boost::get<int>(params["rgbGIndex"]);
    rgbBIndex = (std::size_t)boost::get<int>(params["rgbBIndex"]);
  }
  if (params.find("normalXIndex") != params.end()) {
    normalXIndex = (std::size_t)boost::get<int>(params["normalXIndex"]);
    normalYIndex = (std::size_t)boost::get<int>(params["normalYIndex"]);
    normalZIndex = (std::size_t)boost::get<int>(params["normalZIndex"]);
  }
  if (params.find("snapNeighborNormal") != params.end()) {
    snapNeighborNormal = boost::get<bool>(params["snapNeighborNormal"]);
  }

  // Fill voxel grid
  try {
    std::string line;
    double x = 0, y = 0, z = 0, r = 0, g = 0, b = 0, xnorm = 0, ynorm = 0,
           znorm = 0;
    while (getline(ifs, line)) {
      // Parse
      if (isLineComment(line))
        continue;
      boost::algorithm::trim(line);
      std::vector<std::string> lineParts;
      boost::split(lineParts, line, boost::is_any_of(separator));
      unsigned int size = lineParts.size();

      // Coordinates
      if (size >= 3) {
        x = std::strtod(lineParts[0].c_str(), nullptr);
        y = std::strtod(lineParts[1].c_str(), nullptr);
        z = std::strtod(lineParts[2].c_str(), nullptr);
      }

      // RGB
      if (size >= rgbBIndex) {
        r = std::strtod(lineParts[rgbRIndex].c_str(), nullptr) /
            this->maxColorValue;
        g = std::strtod(lineParts[rgbGIndex].c_str(), nullptr) /
            this->maxColorValue;
        b = std::strtod(lineParts[rgbBIndex].c_str(), nullptr) /
            this->maxColorValue;
      }

      // Normal
      if (!estimateNormals) {
        if (size >= normalZIndex) {
          xnorm = std::strtod(lineParts[normalXIndex].c_str(), nullptr);
          ynorm = std::strtod(lineParts[normalYIndex].c_str(), nullptr);
          znorm = std::strtod(lineParts[normalZIndex].c_str(), nullptr);
          // Correct voxel normal if necessary
          if (!correctNormal(xnorm, ynorm, znorm)) {
            // Ignore points which normals are not correct and
            // cannot be corrected neither
            discardedPointsByNormal += 1;
            continue;
          }
        }
      }

      // Digest voxel
      digestVoxel(
        estimateNormals, halfVoxelSize, x, y, z, r, g, b, xnorm, ynorm, znorm);
    }
    // Restore file cursor position to beginning
    restartInputFileStream(ifs);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to read (2nd pass, fillVoxelsGrid) xyz point cloud"
          "file \""
       << filePathString << "\"\nEXCEPTION: " << e.what();
    logging::ERR(ss.str());
  }
}

bool
XYZPointCloudFileLoader::correctNormal(double& x, double& y, double& z)
{
  bool valid = !std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
               (x != 0 || y != 0 || z != 0);

  if (!valid) {
    // Try to correct non-valid normal if possible
    if (assignDefaultNormal) {
      x = defaultNormal.x;
      y = defaultNormal.y;
      z = defaultNormal.z;
      valid = true;
    }
  }

  return valid;
}

void
XYZPointCloudFileLoader::digestVoxel(int estimateNormals,
                                     double halfVoxelSize,
                                     double x,
                                     double y,
                                     double z,
                                     double r,
                                     double g,
                                     double b,
                                     double xnorm,
                                     double ynorm,
                                     double znorm)
{
  // Compute index and indices to obtain corresponding voxel
  std::size_t I, J, K;
  std::size_t const IDX = indexFromCoordinates(x, y, z, I, J, K);
  Voxel* voxel = voxelGrid->getVoxel(IDX);

  // If voxel does not exist, create it
  if (voxel == nullptr) {
    voxel = voxelGrid->setVoxel(IDX,
                                minX + halfVoxelSize * (double)(2 * I + 1),
                                minY + halfVoxelSize * (double)(2 * J + 1),
                                minZ + halfVoxelSize * (double)(2 * K + 1),
                                halfVoxelSize);
  }

  // Populate voxel
  voxel->numPoints++;
  voxel->r +=
    r * r; // averaging RGB colors can be approximated by averaging the squares
  voxel->g +=
    g * g; // see https://sighack.com/post/averaging-rgb-colors-the-right-way
  voxel->b += b * b;
  if (!estimateNormals) {
    if (snapNeighborNormal) {
      // Snap closest neighbor normal
      double const xDiff = x - voxel->v.getX();
      double const yDiff = y - voxel->v.getY();
      double const zDiff = z - voxel->v.getZ();
      double const distance =
        std::sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
      if (distance < voxelGrid->getClosestPointDistance(IDX)) {
        voxelGrid->setClosestPointDistance(IDX, distance);
        voxel->v.normal[0] = xnorm;
        voxel->v.normal[1] = ynorm;
        voxel->v.normal[2] = znorm;
      }
    } else {
      // Aggregate all points normals to compute voxel normal
      voxel->v.normal[0] += xnorm;
      voxel->v.normal[1] += ynorm;
      voxel->v.normal[2] += znorm;
    }
  }
}

void
XYZPointCloudFileLoader::warnAboutPotentialErrors(
  std::string const& filePathString)
{
  // Iterate over voxels to check them
  Voxel* voxel;
  bool nonUnitaryNormals = false;
  voxelGrid->whileLoopStart();
  while (voxelGrid->whileLoopHasNext()) { // For each occupied voxel in grid
    voxel = voxelGrid->whileLoopNext();
    if (voxel->v.normal.x < -1.0 || voxel->v.normal.x > 1.0 ||
        voxel->v.normal.y < -1.0 || voxel->v.normal.y > 1.0 ||
        voxel->v.normal.z < -1.0 || voxel->v.normal.z > 1.0) {
      nonUnitaryNormals = true;
      break; // If more checks are added in the future, remove this break
    }
  }

  // Report checks
  if (nonUnitaryNormals) {
    std::stringstream ss;
    ss << "Non unitary normals were found in point cloud loaded from "
       << "file:\n\t\"" << filePathString << "\"";
    logging::WARN(ss.str());
  }
}

void
XYZPointCloudFileLoader::postProcess(std::string const& matName,
                                     int estimateNormals)
{
  bool tooPopulatedWarning = false;

  // Post processing loop
  Voxel* voxel;
  voxelGrid->whileLoopStart();
  while (voxelGrid->whileLoopHasNext()) { // For each occupied voxel in grid
    voxel = voxelGrid->whileLoopNext();
    if (voxel->numPoints > voxelPopulationThreshold) {
      tooPopulatedWarning = true; // Check voxel population is in bounds
    }
    // Post-process voxel
    voxel->v.color =
      Color4f(std::sqrt(((float)voxel->r) / ((float)voxel->numPoints)),
              std::sqrt(((float)voxel->g) / ((float)voxel->numPoints)),
              std::sqrt(((float)voxel->b) / ((float)voxel->numPoints)),
              1.0);
    if (!estimateNormals && voxel->hasNormal() && !snapNeighborNormal)
      voxel->v.normal = glm::normalize(voxel->v.normal);
    voxel->material = getMaterial(matName);
  }

  // Warning about too populated voxels
  if (tooPopulatedWarning) {
    std::stringstream ss;
    ss << "There are too populated voxels (those with more than "
       << voxelPopulationThreshold << " points)\n"
       << "Computation might be to slow or even run out of memory.\n"
       << "Please, consider reducing voxel size.";
    logging::WARN(ss.str());
  }
}

void
XYZPointCloudFileLoader::estimateNormals(std::ifstream& ifs)
{
  // Prepare voxel matrices
  for (size_t i = 0; i < maxNVoxels; i++) {
    if (voxelGrid->hasVoxel(i)) {
      voxelGrid->setMatrix(
        i, new arma::Mat<double>(3, voxelGrid->getVoxel(i)->numPoints));
    }
  }

  // Populate voxel matrices
  try {
    std::string line;
    std::size_t I, J, K, IDX;
    double x, y, z;
    while (getline(ifs, line)) {
      // Parse
      if (isLineComment(line))
        continue;
      boost::algorithm::trim(line);
      std::vector<std::string> lineParts;
      boost::split(lineParts, line, boost::is_any_of(separator));
      unsigned int size = lineParts.size();
      // Extract coordinates
      if (size >= 3) {
        x = std::strtod(lineParts[0].c_str(), nullptr);
        y = std::strtod(lineParts[1].c_str(), nullptr);
        z = std::strtod(lineParts[2].c_str(), nullptr);
        // Populate coordinates matrix
        IDX = indexFromCoordinates(x, y, z, I, J, K);
        voxelGrid->setNextMatrixCol(IDX, x, y, z);
      }
    }
    // Restore file cursor position to beginning
    restartInputFileStream(ifs);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to read (estimate normals) xyz point cloud file \n"
          "EXCEPTION: "
       << e.what();
    logging::ERR(ss.str());
    exit(-1);
  }

  // Estimate normals
  _estimateNormals(0, maxNVoxels);

  // Free matrices
  voxelGrid->deleteMatrices();
}

void
XYZPointCloudFileLoader::estimateNormalsBatch(std::ifstream& ifs)
{
  // Prepare variables
  std::size_t startIdx = 0; // Start index for current batch [INCLUSIVE]
  std::size_t endIdx;       // End index of current batch [EXCLUSIVE]

  // Estimate normals by batches
  for (std::size_t batch = 0; batch < numBatches; batch++) {
    // Initialize points count to 0  (How many points considered for batch)
    std::size_t pointsCount = 0;

    // Prepare voxel matrices
    for (endIdx = startIdx; endIdx < maxNVoxels; endIdx++) {
      // Skip empty cells (grid cells with no voxel)
      if (!voxelGrid->hasVoxel(endIdx))
        continue;
      // Handle number of points for this batch
      std::size_t const voxelNumPoints = voxelGrid->getVoxel(endIdx)->numPoints;
      std::size_t const newPointsCount = pointsCount + voxelNumPoints;
      if (newPointsCount > batchSize) {
        // If batch size has been exceeded then finish
        break;
      }
      // Continue batch
      pointsCount = newPointsCount;
      voxelGrid->setMatrix(endIdx, new arma::Mat<double>(3, voxelNumPoints));
    }

    // Populate voxel matrices
    try {
      std::string line;
      std::size_t I, J, K, IDX;
      double x, y, z;
      while (getline(ifs, line)) {
        // Parse
        if (isLineComment(line))
          continue;
        boost::algorithm::trim(line);
        std::vector<std::string> lineParts;
        boost::split(lineParts, line, boost::is_any_of(separator));
        unsigned int size = lineParts.size();
        // Extract coordinates
        if (size >= 3) {
          x = std::strtod(lineParts[0].c_str(), nullptr);
          y = std::strtod(lineParts[1].c_str(), nullptr);
          z = std::strtod(lineParts[2].c_str(), nullptr);
          // Check point is inside batch (ignore if it is not)
          IDX = indexFromCoordinates(x, y, z, I, J, K);
          if (IDX < startIdx || IDX >= endIdx)
            continue;
          // Populate coordinates matrix
          voxelGrid->setNextMatrixCol(IDX, x, y, z);
        }
      }
      // Restore file cursor position to beginning
      restartInputFileStream(ifs);
    } catch (std::exception& e) {
      std::stringstream ss;
      ss << "Failed to read (estimate normals batch) xyz point cloud"
            "file \nEXCEPTION: "
         << e.what();
      logging::ERR(ss.str());
      exit(-1);
    }

    // Estimate normals for current batch
    _estimateNormals(startIdx, endIdx);

    // Free matrices
    voxelGrid->deleteMatrices();

    // Update startIdx for next batch
    startIdx = endIdx;
  }
}

void
XYZPointCloudFileLoader::_estimateNormals(std::size_t start, std::size_t end)
{
  // Find populated matrices
  voxelGrid->whileLoopStart();
  while (voxelGrid->whileLoopHasNext()) {
    size_t key;
    Voxel* voxel = voxelGrid->whileLoopNext(&key);
    arma::Mat<double>* matrix = voxelGrid->getMatrix(key);
    if (matrix != nullptr) {
      if (voxel->numPoints <
          XYZPointCloudFileLoader::minPointsForSafeNormalEstimation) {
        // Not enough points for normal estimation
        unsafeNormalEstimations += 1;
        if (assignDefaultNormal) {
          // Use default normal
          voxel->v.normal = defaultNormal;
        } else {
          // Discard voxel
          voxelGrid->deleteVoxel(key);
          voxelGrid->deleteMatrix(key);
        }
      } else {
        // Estimate normal
        std::vector<double> orthonormal =
          PlaneFitter::bestFittingPlaneOrthoNormal(*matrix, true);
        voxel->v.normal.x = orthonormal[0];
        voxel->v.normal.y = orthonormal[1];
        voxel->v.normal.z = orthonormal[2];
      }
    }
  }
}

void
XYZPointCloudFileLoader::voxelsGridToScenePart()
{
  voxelGrid->whileLoopStart();
  while (voxelGrid->whileLoopHasNext()) { // For each occupied voxel in grid
    Voxel* voxel = voxelGrid->whileLoopNext();
    // Add to primitives only if voxel has not been deleted (i.e., not null)
    if (voxel != nullptr)
      primsOut->mPrimitives.push_back(voxel);
  }
}

std::size_t
XYZPointCloudFileLoader::indexFromCoordinates(double x,
                                              double y,
                                              double z,
                                              std::size_t& I,
                                              std::size_t& J,
                                              std::size_t& K)
{
  I = (std::size_t)((x - minX) * xCoeff);
  if (I >= nx)
    I = nx - 1;
  J = (std::size_t)((y - minY) * yCoeff);
  if (J >= ny)
    J = ny - 1;
  K = (std::size_t)((z - minZ) * zCoeff);
  if (K >= nz)
    K = nz - 1;
  return I * nynz + J * nz + K;
}

bool
XYZPointCloudFileLoader::isLineComment(std::string const& line)
{
  return line[0] == '/' && line[1] == '/';
}

void
XYZPointCloudFileLoader::restartInputFileStream(std::ifstream& ifs)
{
  // Restore file cursor position to beginning
  ifs.clear();
  ifs.seekg(0, std::ios::beg);
}
