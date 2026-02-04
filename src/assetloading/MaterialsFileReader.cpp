#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
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
      throw HeliosException(ss.str());
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
        else
          newMats[newMat.name] = std::make_shared<Material>(newMat);
        newMat = Material();
        newMat.matFilePath = filePathString;
        newMat.name = lineParts[1];
      } else if (lineParts[0] == "Ka" && lineParts.size() >= 4) {
        // Read ambient
        newMat.ka[0] = boost::lexical_cast<float>(lineParts[1]);
        newMat.ka[1] = boost::lexical_cast<float>(lineParts[2]);
        newMat.ka[2] = boost::lexical_cast<float>(lineParts[3]);
      } else if (lineParts[0] == "Kd" && lineParts.size() >= 4) {
        // Read diffuse
        newMat.kd[0] = boost::lexical_cast<float>(lineParts[1]);
        newMat.kd[1] = boost::lexical_cast<float>(lineParts[2]);
        newMat.kd[2] = boost::lexical_cast<float>(lineParts[3]);
      } else if (lineParts[0] == "Ks" && lineParts.size() >= 4) {
        // Read specular
        newMat.ks[0] = boost::lexical_cast<float>(lineParts[1]);
        newMat.ks[1] = boost::lexical_cast<float>(lineParts[2]);
        newMat.ks[2] = boost::lexical_cast<float>(lineParts[3]);
      } else if (lineParts[0] == "Ns" && lineParts.size() >= 2) {
        // Read specular exponent
        newMat.specularExponent = boost::lexical_cast<double>(lineParts[1]);
      } else if (lineParts[0] == "map_Kd" && lineParts.size() >= 2) {
        newMat.map_Kd = lineParts[1];
      }
      // ####### END Wavefront .mtl standard attributes #########

      // ######### BEGIN HELIOS-specific additions to the wavefront .mtl
      // standard #########
      else if (lineParts[0] == "helios_reflectance" && lineParts.size() >= 2) {
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
    newMat.setSpecularity(); // Specularity scalar from components
    newMats[newMat.name] = std::make_shared<Material>(newMat);
    std::stringstream ss;
    ss << newMats.size() << " material(s) loaded.";
    logging::DEBUG(ss.str());
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Error reading materials file '" << filePathString << "'\n"
       << "EXCEPTION: " << e.what();
    logging::ERR(ss.str());
    throw;
  }
  ifs.close();

  return newMats;
}
