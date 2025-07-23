#include "DetailedVoxelLoader.h"
#include "VoxelFileParser.h"
#include <FileUtils.h>
#include <LadLutLoader.h>
#include <assetloading/MaterialsFileReader.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <noise/UniformNoiseSource.h>

namespace fs = boost::filesystem;

ScenePart*
DetailedVoxelLoader::run()
{
  // Retrieve params
  bool transmittiveMode = false;
  if (params.find("intersectionMode") != params.end()) {
    primsOut->onRayIntersectionMode =
      boost::get<std::string>(params["intersectionMode"]);
    boost::to_upper(primsOut->onRayIntersectionMode);
    if (primsOut->onRayIntersectionMode == "SCALED") {
      primsOut->onRayIntersectionArgument = 0.5;
    }
  } else
    transmittiveMode = true;
  if (params.find("intersectionArgument") != params.end()) {
    primsOut->onRayIntersectionArgument =
      boost::get<double>(params["intersectionArgument"]);
  }
  if (params.find("randomShift") != params.end()) {
    primsOut->randomShift = boost::get<bool>(params["randomShift"]);
  }

  // Determine filepath
  std::vector<std::string> filePaths =
    FileUtils::handleFilePath(params, assetsDir);
  for (std::string filePath : filePaths) {
    std::stringstream ss;
    ss << "Reading detailed voxels from " << filePath;
    logging::INFO(ss.str());
  }

  // Load DV files
  for (std::string const& pathString : filePaths) {
    loadDv(pathString, transmittiveMode);
    primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
  }

  // Load material if any
  loadMaterial();

  // Load ladlut if any
  loadLadlut();

  // Return detailed voxels as ScenePart *
  return primsOut;
}

void
DetailedVoxelLoader::loadDv(std::string const& pathString,
                            bool const discardNullPad)
{
  // Check path exists
  fs::path fsPath(pathString);
  if (!fs::exists(pathString)) {
    std::stringstream ss;
    ss << "Voxel file not found: " << pathString;
    logging::ERR(ss.str());
    exit(1);
  }
  // Prepare default material
  Material mat;
  // Legacy default material for vegetation studies commented below
  /*mat.isGround = false;
  mat.useVertexColors = true;
  mat.reflectance = 0.5;
  mat.specularity = 0.5;
  mat.classification = 1;
  mat.ka[0] = 0.5;    mat.ka[1] = 0.5;
  mat.ka[2] = 0.5;    mat.ka[2] = 0.5;
  mat.kd[0] = 0.5;    mat.kd[1] = 0.5;
  mat.kd[2] = 0.5;    mat.kd[3] = 0.5;
  mat.ks[0] = 0.5;    mat.ks[1] = 0.5;
  mat.ks[2] = 0.5;    mat.ks[3] = 0.5;
  mat.spectra = "wood";*/

  // Parse detailed voxels
  VoxelFileParser vfp;
  std::vector<DetailedVoxel*> dvs =
    vfp.bruteParseDetailed(pathString, 2, false, discardNullPad);

  // Prepare detailed voxels
  for (DetailedVoxel* dv : dvs) {
    dv->material = std::make_shared<Material>(mat);
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

void
DetailedVoxelLoader::loadLadlut()
{
  // If no LadLut is specified, get out of here
  if (params.find("ladlut") == params.end())
    return;

  // Load LadLut
  std::string ladlutPath = boost::get<std::string>(params["ladlut"]);
  LadLutLoader ladlutLoader;
  primsOut->ladlut = ladlutLoader.load(ladlutPath);
}
