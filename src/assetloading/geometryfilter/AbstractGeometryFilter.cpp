#include "AbstractGeometryFilter.h"
#include "MaterialsFileReader.h"
#include "UniformNoiseSource.h"

std::shared_ptr<Material>
AbstractGeometryFilter::getMaterial(std::string materialName)
{
  auto it = materials.find(materialName);

  if (it == materials.end()) {
    std::shared_ptr<Material> mat = std::make_shared<Material>();
    mat->name = materialName;
    materials[materialName] = mat;
    return mat;
  }

  return it->second;
}

std::vector<std::shared_ptr<Material>>
AbstractGeometryFilter::parseMaterials()
{
  if (params.count("randomMaterials") || params.count("randomRange"))
    throw HeliosException(
      "randomMaterials/randomRange were removed; supply explicit materials");

  if (params.find("matfile") == params.end()) {
    return std::vector<std::shared_ptr<Material>>(0);
  }

  fs::path matfilePath = std::get<std::string>(params["matfile"]);
  bool found = false;
  for (const auto& base : assetsDir) {
    fs::path candidate = fs::path(base) / matfilePath;
    if (fs::exists(candidate)) {
      matfilePath = candidate;
      found = true;
      break;
    }
  }

  if (!found) {
    std::stringstream ss;
    ss << "Material file not found: " << matfilePath.string();
    logging::ERR(ss.str());
    throw HeliosException(ss.str());
  }

  // Pick material
  auto mats = MaterialsFileReader::loadMaterials(matfilePath.string());
  std::vector<std::shared_ptr<Material>> matvec(0);
  if (params.find("matname") != params.end()) { // Pick by name
    std::string matname = std::get<std::string>(params["matname"]);
    auto it = mats.find(matname);
    if (it == mats.end()) {
      std::stringstream ss;
      ss << "Material with name '" << matname
         << "' was not found in material file: " << matfilePath.string();
      logging::ERR(ss.str());
      throw HeliosException(ss.str());
    }
    matvec.push_back(it->second);
  } else { // No name, so pick first
    if (!mats.empty())
      matvec.push_back(mats.begin()->second);
  }

  return matvec;
}
