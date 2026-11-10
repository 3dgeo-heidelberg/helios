#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <logging.hpp>

#include "MaterialsFileReader.h"

std::map<std::string, std::shared_ptr<Material>>
MaterialsFileReader::loadMaterials(std::string filePathString)
{
  std::ifstream ifs;
  Material newMat;
  std::map<std::string, std::shared_ptr<Material>> newMats;
  bool firstMaterial = true;

  logging::DEBUG("Reading materials from .mtl file '" + filePathString + "'");
  std::string line;

  try {
    ifs = std::ifstream(filePathString, std::ifstream::binary);
    if (!ifs.is_open()) {
      std::stringstream ss;
      ss << "Failed to open materials file: " << filePathString;
      throw std::runtime_error(ss.str());
    }
    while (getline(ifs, line)) {
      if (line.empty() || line == "\r" || line == "\r\n" || line == "\n")
        continue;
      std::vector<std::string> lineParts;
      boost::regex_split(
        std::back_inserter(lineParts), line, boost::regex("\\s+"));

      // ####### BEGIN Wavefront .mtl standard attributes #########
      if (lineParts[0] == "newmtl" && lineParts.size() >= 2) {
        // Before starting a new material, put previous material to the map:
        if (firstMaterial)
          firstMaterial = false;
        else {
          newMat.validate();
          newMats[newMat.name] = std::make_shared<Material>(newMat);
        }
        newMat = Material();
        newMat.matFilePath = filePathString;
        newMat.name = lineParts[1];
      } else if (lineParts[0] == "Ns" && lineParts.size() >= 2) {
        // Read specular exponent
        newMat.specularExponent = boost::lexical_cast<double>(lineParts[1]);
      } else if (lineParts[0] == "map_Kd" && lineParts.size() >= 2) {
        newMat.map_Kd = lineParts[1];
      }
      // ####### END Wavefront .mtl standard attributes #########

      // ######### BEGIN HELIOS-specific additions to the wavefront .mtl
      // standard #########
      else if (lineParts[0] == "helios_specularity") {
        if (lineParts.size() < 2)
          throw std::invalid_argument("Missing helios_specularity");

        newMat.specularity = boost::lexical_cast<double>(lineParts[1]);
      } else if (lineParts[0] == "helios_is_direction_independent") {
        if (lineParts.size() < 2)
          throw std::invalid_argument("Missing direction-independent flag");

        auto value = lineParts[1];
        std::transform(value.begin(),
                       value.end(),
                       value.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        if (value != "true" && value != "false" && value != "1" && value != "0")
          throw std::invalid_argument(
            "Invalid direction-independent flag for " + newMat.name);

        newMat.isDirectionIndependent = value == "true" || value == "1";
      } else if (lineParts[0] == "helios_reflectance" &&
                 lineParts.size() >= 2) {
        newMat.reflectance = boost::lexical_cast<double>(lineParts[1]);
      }

      else if (lineParts[0] == "helios_isGround" && lineParts.size() >= 2) {
        transform(lineParts[1].begin(),
                  lineParts[1].end(),
                  lineParts[1].begin(),
                  ::tolower);
        newMat.isGround = lineParts[1] == "true" || lineParts[1] == "1";
      }

      else if (lineParts[0] == "helios_useVertexColors" &&
               lineParts.size() >= 2) {
        newMat.useVertexColors = boost::lexical_cast<bool>(lineParts[1]);
      } else if (lineParts[0] == "helios_classification" &&
                 lineParts.size() >= 2) {
        newMat.classification = boost::lexical_cast<int>(lineParts[1]);
      } else if (lineParts[0] == "helios_spectra" && lineParts.size() >= 2) {
        newMat.spectra = lineParts[1];
      }
      // ######### END HELIOS-specific additions to the wavefront .mtl standard
      // #########
    }

    // Don't forget to put final material to the map:
    newMat.validate();
    newMats[newMat.name] = std::make_shared<Material>(newMat);
    std::stringstream ss;
    ss << newMats.size() << " material(s) loaded.";
    logging::DEBUG(ss.str());
  } catch (std::exception& e) {
    throw std::runtime_error("Error reading materials file '" + filePathString +
                             "' (material " + newMat.name + "): " + e.what());
  }
  ifs.close();

  return newMats;
}
