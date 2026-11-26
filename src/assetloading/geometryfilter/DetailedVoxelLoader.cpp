#include "DetailedVoxelLoader.h"
#include "VoxelFileParser.h"
#include <FileUtils.h>
#include <LeafAngleDistribution.h>
#include <assetloading/MaterialsFileReader.h>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <noise/UniformNoiseSource.h>

namespace fs = std::filesystem;

ScenePart*
DetailedVoxelLoader::run()
{
  // Retrieve params (transmittive is the default mode)
  bool transmittiveMode = true;
  if (params.find("intersectionMode") != params.end()) {
    primsOut->onRayIntersectionMode =
      std::get<std::string>(params["intersectionMode"]);
    boost::to_upper(primsOut->onRayIntersectionMode);
    transmittiveMode =
      DetailedVoxel::isTransmittiveMode(primsOut->onRayIntersectionMode);
    if (primsOut->onRayIntersectionMode == "SCALED") {
      primsOut->onRayIntersectionArgument = 0.5;
    }
  }
  if (params.find("intersectionArgument") != params.end()) {
    primsOut->onRayIntersectionArgument =
      std::get<double>(params["intersectionArgument"]);
  }
  if (params.find("randomShift") != params.end()) {
    primsOut->randomShift = std::get<bool>(params["randomShift"]);
  }

  // Determine filepath
  std::vector<std::string> filePaths =
    FileUtils::handleFilePath(params, assetsDir);
  for (std::string filePath : filePaths) {
    std::stringstream ss;
    ss << "Reading detailed voxels from " << filePath;
    LOG_INFO(ss.str());
  }

  // Load DV files. Transmittive mode discards voxels without plant area
  // (PAD 0 or NaN), scaled mode those whose size cannot be derived (NaN)
  bool const discardNanPad =
    transmittiveMode || primsOut->onRayIntersectionMode == "SCALED";
  for (std::string const& pathString : filePaths) {
    loadDv(pathString, transmittiveMode, discardNanPad);
    primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
  }

  // Load material if any
  loadMaterial();

  // Load leaf angle distribution if any (only used in transmittive mode)
  loadLeafAngleDistribution(filePaths, transmittiveMode);

  // Return detailed voxels as ScenePart *
  return primsOut;
}

void
DetailedVoxelLoader::loadDv(std::string const& pathString,
                            bool const discardNullPad,
                            bool const discardNanPad)
{
  // Check path exists
  fs::path fsPath(pathString);
  if (!fs::exists(pathString)) {
    std::stringstream ss;
    ss << "Voxel file not found: " << pathString;
    LOG_ERR(ss.str());
    throw HeliosException(ss.str());
  }
  // Prepare default material
  Material mat;
  materials[mat.name] = std::make_shared<Material>(mat);
  // Legacy default material for vegetation studies commented below
  /*mat.isGround = false;
  mat.useVertexColors = true;
  mat.reflectance = 0.5;
  mat.ks = 0.5;
  mat.classification = 1;
  mat.spectra = "wood";*/

  // Parse detailed voxels
  VoxelFileParser vfp;
  std::vector<DetailedVoxel*> dvs =
    vfp.bruteParseDetailed(pathString, 2, false, discardNullPad);

  // Prepare detailed voxels
  for (DetailedVoxel* dv : dvs) {
    if (discardNanPad && std::isnan((*dv)["PadBVTotal"])) {
      delete dv;
      continue;
    }
    dv->material = getMaterial(mat.name);
    for (size_t i = 0; i < dv->getNumVertices(); i++) {
      Color4f& color = dv->getVertices()[i].color;
      color.x = 0.5;
      color.y = 0.5;
      color.z = 0.5;
      color.w = 0.5;
    }
    primsOut->mPrimitives.push_back(dv);
  }
}

void
DetailedVoxelLoader::loadMaterial()
{
  // Parse materials
  std::vector<std::shared_ptr<Material>> matvec = parseMaterials();
  if (matvec.empty())
    return;

  // Assign material to each detailed voxel
  size_t j, n = primsOut->mPrimitives.size(), m = matvec.size();
  for (size_t i = 0; i < n; i++) {
    DetailedVoxel* dv = (DetailedVoxel*)primsOut->mPrimitives[i];
    j = i % m;
    dv->material = matvec[j];
  }
}

std::string
DetailedVoxelLoader::readLadType(std::string const& path)
{
  // AMAPVox writes e.g. "#lad_type:Planophile" into the header lines
  static std::string const key = "#lad_type:";
  static std::size_t const maxHeaderLines = 20;
  std::ifstream ifs(path);
  std::string line;
  for (std::size_t i = 0; i < maxHeaderLines && std::getline(ifs, line); ++i) {
    std::size_t const pos = line.find(key);
    if (pos == std::string::npos)
      continue;
    std::size_t const start = pos + key.size();
    std::size_t const end = line.find_first_of(" \t\r\n#", start);
    return line.substr(start, end == std::string::npos ? end : end - start);
  }
  return "";
}

void
DetailedVoxelLoader::loadLeafAngleDistribution(
  std::vector<std::string> const& filePaths,
  bool const transmittiveMode)
{
  // The former look-up tables are no longer supported
  if (params.find("ladlut") != params.end()) {
    std::string const msg =
      "The \"ladlut\" parameter is no longer supported. Use \"lad\" with "
      "the name of a leaf angle distribution (planophile, erectophile, "
      "plagiophile, extremophile, spherical, uniform) or the path to a file "
      "with leaf inclinations (degrees) and their density g_L.";
    LOG_ERR(msg);
    throw HeliosException(msg);
  }

  // Only voxels in transmittive mode are turbid media
  if (!transmittiveMode) {
    if (params.find("lad") != params.end()) {
      std::stringstream ss;
      ss << "The leaf angle distribution (\"lad\") is ignored in intersection "
         << "mode " << primsOut->onRayIntersectionMode
         << ". It is only used in transmittive mode.";
      LOG_WARN(ss.str());
    }
    return;
  }

  // If no leaf angle distribution is specified, use the one the voxel file
  // header declares (the one used to derive PAD), or the default one
  if (params.find("lad") == params.end()) {
    std::string ladType;
    for (std::string const& filePath : filePaths) {
      std::string const t = readLadType(filePath);
      if (ladType.empty()) {
        ladType = t;
      } else if (!t.empty() && !boost::iequals(t, ladType)) {
        std::stringstream ss;
        ss << "Voxel files declare different leaf angle distributions ("
           << ladType << ", " << t << "). Using " << ladType << ".";
        LOG_WARN(ss.str());
      }
    }
    if (ladType.empty())
      return;
    if (!LeafAngleDistribution::isNamedType(ladType)) {
      std::stringstream ss;
      ss << "Leaf angle distribution \"" << ladType << "\" from the voxel "
         << "file header is not supported. Using the default (spherical). "
         << "Specify \"lad\" to choose a distribution.";
      LOG_WARN(ss.str());
      return;
    }
    primsOut->lad = std::make_shared<LeafAngleDistribution const>(
      LeafAngleDistribution::fromName(ladType));
    return;
  }

  std::string const lad = std::get<std::string>(params["lad"]);
  if (LeafAngleDistribution::isNamedType(lad)) {
    primsOut->lad = std::make_shared<LeafAngleDistribution const>(
      LeafAngleDistribution::fromName(lad));
    return;
  }

  // Resolve the file path: absolute or relative to the working directory
  // first, then relative to the asset directories
  fs::path path(lad);
  if (path.is_relative() && !fs::exists(path)) {
    for (auto const& assetPath : assetsDir) {
      if (fs::exists(fs::path(assetPath) / lad)) {
        path = fs::path(assetPath) / lad;
        break;
      }
    }
  }
  primsOut->lad = std::make_shared<LeafAngleDistribution const>(
    LeafAngleDistribution::fromFile(path.string()));
}
