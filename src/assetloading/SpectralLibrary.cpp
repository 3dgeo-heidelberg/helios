#include "SpectralLibrary.h"

#include <fstream>
#include <iostream>
#include <logger_core.hpp>
#include <set>
#include <typeinfo>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

SpectralLibrary::SpectralLibrary(float wavelength_m,
                                 std::vector<std::string> assetsDir,
                                 const std::string spectra)
  : assetsDir(assetsDir)
  , spectra(spectra)
{

  reflectanceMap = std::map<string, float>();
  wavelength_um = wavelength_m * 1000000;
}

float
SpectralLibrary::interpolateReflectance(float w0, float w1, float r0, float r1)
{
  if (w0 == w1) {
    throw HeliosException("Cannot interpolate spectral reflectance "
                          "between identical wavelengths.");
  }
  float wRange = w1 - w0;
  float wShift = wavelength_um - w0;
  float factor = wShift / wRange;
  float rRange = r1 - r0;

  return r0 + (factor * rRange);
}

void
SpectralLibrary::readFileAster(fs::path path)
{
  try {
    std::ifstream ins(path.string(), std::ifstream::binary);
    if (!ins.is_open()) {
      LOG_WARN("Failed to open spectral file '" + path.string() + "'.");
      return;
    }
    float wavelength = 0;
    float reflectance = 0;
    float prevWavelength = 0;
    float prevReflectance = 0;
    std::string line;
    bool reflectanceFound = false;

    // Skip the header
    for (int i = 0; i < 26; i++) {
      getline(ins, line);
    }
    while (getline(ins, line)) {

      std::vector<std::string> values;
      boost::split(values, line, boost::is_any_of("\t"));
      wavelength = boost::lexical_cast<float>(values.at(0));
      boost::trim_right(values.at(1));
      reflectance = boost::lexical_cast<float>(values.at(1));

      if (wavelength < wavelength_um) {
        prevWavelength = wavelength;
        prevReflectance = reflectance;
        continue;
      }
      if (wavelength > wavelength_um) {
        reflectance = interpolateReflectance(
          prevWavelength, wavelength, prevReflectance, reflectance);
      }
      reflectanceFound = true;
      break;
    }
    ins.close();

    if (!reflectanceFound) {
      LOG_WARN("No usable reflectance value found in spectral file '" +
               path.string() + "'.");
      return;
    }

    std::string file = path.filename().string();
    if (file.find_first_of(".") > 0) {
      file = file.substr(0, file.find_last_of("."));
    }
    reflectanceMap.insert(std::pair<std::string, float>(file, reflectance));
  } catch (const std::exception& e) {
    LOG_WARN("Failed to read spectral file '" + path.string() +
             "': " + e.what());
  }
}

void
SpectralLibrary::readReflectances()
{
  LOG_INFO("Reading Spectral Library...");

  bool found = false;
  for (const auto path : assetsDir) {
    if (!fs::is_directory(fs::path(path) / spectra))
      continue;

    found = true;
    for (auto& p : fs::directory_iterator(fs::path(path) / spectra)) {
      readFileAster(p.path());
    }
  }

  if (!found) {
    LOG_WARN("Spectral library folder '" + spectra +
             "' was not found; default reflectance values will be used.");
    return;
  }

  std::stringstream ss;
  ss << reflectanceMap.size() << " materials found";
  LOG_INFO(ss.str());
}

void
SpectralLibrary::setReflectances(Scene* scene)
{
  std::set<std::string> materialsWithoutSpectra;
  std::set<std::string> spectraMissingFromLibrary;

  for (Primitive* prim : scene->primitives) {
    if (!isnan(prim->material->reflectance)) {
      continue; // if a reflectance was set, this has precedence over spectra
    }

    prim->material->reflectance =
      defaultReflectance; // otherwise, set the default reflectance

    if (prim->material->spectra.empty()) {
      const std::string& materialKey = prim->material->matFilePath.empty()
                                         ? prim->material->name
                                         : prim->material->matFilePath;

      if (materialsWithoutSpectra.insert(materialKey).second) {
        LOG_WARN("Material '" + prim->material->name + "' (" +
                 prim->material->matFilePath +
                 ") has no spectral definition; "
                 "using default reflectance.");
      }
      continue;
    }

    if (reflectanceMap.find(prim->material->spectra) == reflectanceMap.end()) {
      if (matsMissing.find(prim->material->spectra) == matsMissing.end()) {
        matsMissing.insert(prim->material->spectra);
        LOG_WARN("Spectra " + prim->material->spectra +
                 "' referenced by material '" + prim->material->name + " (" +
                 prim->material->matFilePath +
                 ") is not in the spectral library"
                 "using default reflectance.");
      }
      continue;
    }

    prim->material->reflectance =
      reflectanceMap.find(prim->material->spectra)->second;
    prim->material->setSpecularity();
  }
}
