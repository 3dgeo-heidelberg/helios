#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
using namespace std;

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <logging.hpp>

#include "MaterialsFileReader.h"

map<string, Material>
MaterialsFileReader::loadMaterials(string filePathString)
{
  ifstream ifs;
  Material newMat;
  map<string, Material> newMats;
  bool firstMaterial = true;

  logging::DEBUG("Reading materials from .mtl file '" + filePathString + "'");
  string line;

  try {
    ifs = ifstream(filePathString, ifstream::binary);
    while (getline(ifs, line)) {
      if (line.empty() || line == "\r" || line == "\r\n" || line == "\n")
        continue;
      vector<string> lineParts;
      boost::regex_split(
        std::back_inserter(lineParts), line, boost::regex("\\s+"));

      // ####### BEGIN Wavefront .mtl standard attributes #########
      if (lineParts[0] == "newmtl" && lineParts.size() >= 2) {
        // Before starting a new material, put previous material to the map:
        if (firstMaterial)
          firstMaterial = false;
        else
          newMats.insert(pair<string, Material>(newMat.name, newMat));
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
    newMats.insert(newMats.end(), pair<string, Material>(newMat.name, newMat));
    std::stringstream ss;
    ss << newMats.size() << " material(s) loaded.";
    logging::DEBUG(ss.str());
  } catch (std::exception& e) {
    logging::WARN("Failed to load materials file: " + filePathString + "\n" +
                  "line='" + line + "'\n" + "EXCEPTION: " + e.what());
  }
  ifs.close();

  return newMats;
}
