#include "SpectralLibrary.h"

#include <fstream>
#include <iostream>
#include <logging.hpp>
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

      logging::ERR("failed to open " + path.string());
      throw std::exception();
    }
    float wavelength = 0;
    float reflectance = 0;
    float prevWavelength = 0;
    float prevReflectance = 0;
    std::string line;

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
      break;
    }
    ins.close();

    std::string file = path.filename().string();
    if (file.find_first_of(".") > 0) {
      file = file.substr(0, file.find_last_of("."));
    }
    reflectanceMap.insert(std::pair<std::string, float>(file, reflectance));
  } catch (std::exception& e) {
    logging::WARN("Error: readFileAster " + path.string() + "\n" +
                  "EXCEPTION: " + e.what());
  }
}

void
SpectralLibrary::readReflectances()
{
  logging::INFO("Reading Spectral Library...");

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
    logging::ERR("ERROR: folder " + spectra + " not found");
    return;
  }

  std::stringstream ss;
  ss << reflectanceMap.size() << " materials found";
  logging::WARN(ss.str());
}

void
SpectralLibrary::setReflectances(Scene* scene)
{
  std::set<std::string> matsMissing;

  for (Primitive* prim : scene->primitives) {
    if (!isnan(prim->material->reflectance)) {
      continue; // if a reflectance was set, this has precedence over spectra
    }

    prim->material->reflectance =
      defaultReflectance; // otherwise, set the default reflectance

    if (prim->material->spectra.empty()) {
      if (matsMissing.find(prim->material->spectra) == matsMissing.end()) {
        matsMissing.insert(prim->material->spectra);
        logging::WARN("Warning: material " + prim->material->name +
                      " of primitive " + typeid(*prim).name() + " (" +
                      prim->material->matFilePath +
                      ") has no spectral definition");
      }
      continue;
    }

    if (reflectanceMap.find(prim->material->spectra) == reflectanceMap.end()) {
      if (matsMissing.find(prim->material->spectra) == matsMissing.end()) {
        matsMissing.insert(prim->material->spectra);
        logging::WARN("Warning: spectra " + prim->material->spectra + " (" +
                      prim->material->matFilePath +
                      ") is not in the spectral library");
      }
      continue;
    }

    prim->material->reflectance =
      reflectanceMap.find(prim->material->spectra)->second;
    prim->material->setSpecularity();
  }
}
