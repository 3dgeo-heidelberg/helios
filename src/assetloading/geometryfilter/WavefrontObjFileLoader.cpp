#include "WavefrontObjFileLoader.h"
#include "WavefrontObj.h"
#include "WavefrontObjCache.h"
#include "logging.hpp"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>

#include "maths/Rotation.h"
typedef boost::variant<bool, int, float, double, std::string, dvec3, Rotation>
    ObjectT;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <FileUtils.h>

#include "Triangle.h"

#include "MaterialsFileReader.h"

// ***  MAIN METHODS *** //
// ********************* //
ScenePart *WavefrontObjFileLoader::run() {
  bool yIsUp = false;
  try {
    // ######### BEGIN Read up axis ###########
    string const &upAxis = boost::get<string const &>(params["up"]);
    if (upAxis == "y") {
      yIsUp = true;
    }
  } catch (std::exception &e) {
    stringstream ss;
    ss << "Error reading params['up']\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
  }
  // ######### END Read up axis ###########

  // Determine filepath
  bool extendedFilePath = false;
  std::vector<std::string> filePaths(0);
  try {
    filePathString = boost::get<string const &>(params["efilepath"]);
    extendedFilePath = true;
  } catch (std::exception &e) {
    try {
      filePathString = boost::get<string const &>(params["filepath"]);
      filePaths.push_back(filePathString);
    } catch (std::exception &e2) {
      stringstream ss;
      ss << "No filepath was provided.\nEXCEPTION: " << e2.what();
      logging::ERR(ss.str());
    }
  }

  // If extended file path, determine all file paths
  if (extendedFilePath)
    filePaths = FileUtils::getFilesByExpression(filePathString);

  // Load OBJ Cache
  auto &cache = WavefrontObjCache::getInstance();
  WavefrontObj * loadedObj;
  for (std::string const &pathString : filePaths) {
    stringstream ss;
    if (!cache.contains(pathString)) {
      ss << ".obj not found in cache";
      loadedObj = loadObj(pathString, yIsUp);
      cache.insert(pathString, loadedObj);
    } else {
      ss << "Loading .obj from cache";
    }

    logging::INFO(ss.str());
    if (loadedObj != nullptr)
    {
      primsOut->addObj(cache.get(pathString));
      primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
    }
  }

  // Post-processing
  bool rvn = boost::get<bool>(params["recomputeVertexNormals"]);

  if (rvn) {
    // TODO 5: Find out why this does really weird things (distorted triangles)
    // when applied to a mesh that already has correct vertex normals (e.g. one
    // of the nice big houses)
    // primsOut->smoothVertexNormals();
  }

  // Report
  std::stringstream ss;
  ss << "# total primitives loaded: " << primsOut->mPrimitives.size();
  logging::INFO(ss.str());

  // Return

  return primsOut;
}

Vertex WavefrontObjFileLoader::readVertex(vector<string> &lineParts,
                                          bool yIsUp) {
  stringstream ss;
  Vertex v;

  // Read position:
  double x = 0, y = 0, z = 0;

  try {
    if (yIsUp) {
      x = boost::lexical_cast<double>(lineParts[1]);
      y = -boost::lexical_cast<double>(lineParts[3]);
      z = boost::lexical_cast<double>(lineParts[2]);
    } else {
      x = boost::lexical_cast<double>(lineParts[1]);
      y = boost::lexical_cast<double>(lineParts[2]);
      z = boost::lexical_cast<double>(lineParts[3]);
    }
  } catch (boost::bad_lexical_cast &e) {
    ss << "Error reading vertex.\nEXCEPTION: " << e.what() << endl;
    logging::WARN(ss.str());
    ss.str("");
  }

  v.pos = dvec3(x, y, z);

  // ######## BEGIN Read vertex color #########
  if (lineParts.size() >= 7) {
    float r = 1, g = 1, b = 1;

    try {
      r = boost::lexical_cast<float>(lineParts[4]);
      g = boost::lexical_cast<float>(lineParts[5]);
      b = boost::lexical_cast<float>(lineParts[6]);
    } catch (boost::bad_lexical_cast &e) {
      ss << "Error reading vertex color.\nEXCEPTION: " << e.what() << endl;
      logging::WARN(ss.str());
      ss.str("");
    }

    Color4f color = Color4f(r, g, b, 1);
    v.color = color;
  }

  return v;
}

dvec3 WavefrontObjFileLoader::readNormalVector(vector<string> &lineParts,
                                               bool yIsUp) {

  stringstream ss;
  dvec3 normal{};
  try {
    double x = 0, y = 0, z = 0;

    if (yIsUp) {
      x = boost::lexical_cast<double>(lineParts[1]);
      y = -boost::lexical_cast<double>(lineParts[3]);
      z = boost::lexical_cast<double>(lineParts[2]);
    } else {
      x = boost::lexical_cast<double>(lineParts[1]);
      y = boost::lexical_cast<double>(lineParts[2]);
      z = boost::lexical_cast<double>(lineParts[3]);
    }

    normal = dvec3(x, y, z);
  } catch (boost::bad_lexical_cast &e) {
    ss << "Error reading normal vector.\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
    ss.str("");
  }

  return normal;
}

void WavefrontObjFileLoader::readPrimitive(
    WavefrontObj *loadedObj, vector<string> &lineParts,
    vector<Vertex> &vertices, vector<dvec2> &texcoords, vector<dvec3> &normals,
    string &currentMat, const string &pathString) {

  stringstream ss;
  // ######### BEGIN Read triangle or quad ##############
  if (lineParts.size() >= 4 && lineParts.size() <= 5) {
    Vertex verts[4];

    // Try to read vertex position, texture and normal indices:
    try {
      for (size_t i = 0; i < lineParts.size() - 1; i++) {
        std::vector<string> fields;
        boost::split(fields, lineParts[i + 1], boost::is_any_of("/"));
        int vi = boost::lexical_cast<int>(fields[0]);
        int ti = 0;
        if (fields.size() > 1 && fields[1].length() > 0)
          ti = boost::lexical_cast<int>(fields[1]);
        int ni = 0;
        if (fields.size() > 2 && fields[2].length() > 0)
          ni = boost::lexical_cast<int>(fields[2]);
        buildPrimitiveVertex(verts[i], vertices.at(vi - 1), ti - 1, ni - 1,
                             texcoords, normals);
      }
    } catch (std::exception &e) {
      // TODO: catch in the caller
      ss << "Exception during attempt to read primitive:\n\t" << e.what()
         << "\n"
         << "\tInput file: \"" << pathString << "\"";
      logging::WARN(ss.str());
      ss.str("");
      return;
    }

    // Read a triangle:
    if (lineParts.size() == 4) {
      Triangle *tri = new Triangle(verts[0], verts[1], verts[2]);
      tri->material = getMaterial(currentMat);
      //      primsOut->mPrimitives.push_back(tri);
      loadedObj->primitives.push_back(tri);
    }

    // Read a quad (two triangles):
    else if (lineParts.size() == 5) {
      Triangle *tri1 = new Triangle(verts[0], verts[1], verts[2]);
      tri1->material = getMaterial(currentMat);
      //      primsOut->mPrimitives.push_back(tri1);
      loadedObj->primitives.push_back(tri1);

      Triangle *tri2 = new Triangle(verts[0], verts[2], verts[3]);
      tri2->material = getMaterial(currentMat);
      //      primsOut->mPrimitives.push_back(tri2);
      loadedObj->primitives.push_back(tri2);
    }
  } else {
    ss << "Unsupported primitive!";
    logging::DEBUG(ss.str());
    ss.str("");
  }
}

WavefrontObj *WavefrontObjFileLoader::loadObj(std::string const &pathString,
                                              bool yIsUp) {
  stringstream ss;

  WavefrontObj *loadedObj = new WavefrontObj();

  ss << "Reading 3D model from .obj file '" << pathString << "'...";
  logging::INFO(ss.str());
  ss.str("");

  fs::path filePath(pathString);
  if (!fs::exists(filePath)) {
    ss << "File not found: " << pathString << endl;
    logging::ERR(ss.str());
    exit(1);
  }

  ifstream is;
  try {
    is = ifstream(pathString, ifstream::binary);
  } catch (std::exception &e) {
    ss << "Failed to create buffered reader for file: " << pathString
       << "\nEXCEPTION: " << e.what() << endl;
    logging::ERR(ss.str());
    exit(-1);
  }

  vector<Vertex> vertices;
  vector<dvec3> normals;
  vector<dvec2> texcoords;

  string currentMat = "default";

  Material mat;
  mat.useVertexColors = true;
  mat.matFilePath = filePath.string();
  materials.insert(std::pair<string, Material>(currentMat, mat));

  string line;
  try {
    while (getline(is, line)) {
      boost::algorithm::trim(line);

      if (line.empty() || line == "\r") {
        continue;
      }

      if (line.substr(0, 1) == "#") {
        continue;
      }

      vector<string> lineParts;
      boost::regex_split(std::back_inserter(lineParts), line,
                         boost::regex("\\s+"));

      // ########## BEGIN Read vertex ##########
      if (lineParts[0] == "v" && lineParts.size() >= 4) {
        Vertex v = WavefrontObjFileLoader::readVertex(lineParts, yIsUp);
        // Add vertex to vertex list:
        vertices.push_back(v);
      }
      // ########## END Read vertex ##########

      // ############ BEGIN Read normal vector ##############
      else if (lineParts[0] == "vn" && lineParts.size() >= 4) {
        dvec3 normal =
            WavefrontObjFileLoader::readNormalVector(lineParts, yIsUp);
        normals.push_back(normal);
      }
      // ############ END Read normal vector ##############

      // ############ BEGIN Read texture coordinates ############
      else if (lineParts[0] == "vt" && lineParts.size() >= 3) {
        dvec2 tc = dvec2(boost::lexical_cast<double>(lineParts[1]),
                         boost::lexical_cast<double>(lineParts[2]));
        texcoords.push_back(tc);
      }
      // ############ END Read texture coordinates ############

      // Read face:
      else if (lineParts[0] == "f") {
        // ######### BEGIN Read triangle or quad ##############
        WavefrontObjFileLoader::readPrimitive(loadedObj, lineParts, vertices,
                                              texcoords, normals, currentMat,
                                              pathString);
        // ######### END Read triangle or quad ##############
      }

      // ######### BEGIN Load materials from materials file ########
      else if (lineParts[0] == "mtllib") {
        string s = filePath.parent_path().string() + "/" + lineParts[1];
        map<string, Material> mats = MaterialsFileReader::loadMaterials(s);
        materials.insert(mats.begin(), mats.end());
      }
      // ######### END Load materials from materials file ########

      else if (lineParts[0] == "usemtl") {
        currentMat = lineParts[1];
      }

      else if (lineParts[0] == "s") {
        // TODO 4: What?
      }

      else {
        ss << "Unknown line '" << line << "'";
        logging::DEBUG(ss.str());
        ss.str("");
      }
    }

  } catch (std::exception &e) {
    ss << "Error reading primitives.\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
    ss.str("");
  }

  is.close();

  return loadedObj;
}

// ***  ASSIST METHODS  *** //
// ************************ //
void WavefrontObjFileLoader::buildPrimitiveVertex(
    Vertex &dstVert, Vertex &srcVert, int texIdx, int normalIdx,
    std::vector<dvec2> const &texcoords, std::vector<dvec3> const &normals) {
  dstVert = srcVert.copy();
  if (texIdx >= 0)
    dstVert.texcoords = texcoords[texIdx];
  if (normalIdx >= 0)
    dstVert.normal = normals[normalIdx];
}
