#include "WavefrontObjFileLoader.h"
#include "WavefrontObj.h"
#include "WavefrontObjCache.h"
#include <filems/read/comps/BufferedLineFileReader.h>
#include <filems/read/exceptions/EndOfReadingException.h>

#include <iostream>
#include <string>
#include <util/logger/logging.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>

#include "maths/Rotation.h"
typedef boost::
  variant<bool, int, float, double, std::string, glm::dvec3, Rotation>
    ObjectT;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <FileUtils.h>

#include "Triangle.h"

#include "MaterialsFileReader.h"

// ***  MAIN METHODS *** //
// ********************* //
ScenePart*
WavefrontObjFileLoader::run()
{
  // Determine filepath
  auto filePaths = FileUtils::handleFilePath(params, assetsDir);

  // Determine up-axis
  bool yIsUp = false;
  try {
    // ######### BEGIN Read up axis ###########
    std::string const& upAxis = boost::get<std::string const&>(params["up"]);
    if (upAxis == "y") {
      yIsUp = true;
    } else if (upAxis != "z") {
      std::stringstream ss;
      ss << "Error: 'up'-axis in the scene XML file may only be one of 'y' or "
            "'z'.\nSetting 'up' to 'z'.";
      logging::ERR(ss.str());
    }
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to read 'up'-axis from scene XML file.\n"
       << "Assuming 'z' axis points upwards for scene part \"" << filePathString
       << "\".\n"
       << "Set up axis explicitly to silence this warning.\n"
       << "C++ Exception: " << e.what();
    logging::INFO(ss.str());
  }
  // ######### END Read up axis ###########

  // Load OBJ Cache
  auto& cache = WavefrontObjCache::getInstance();
  WavefrontObj* loadedObj = nullptr;
  for (std::string const& pathString : filePaths) {
    std::stringstream ss;
    // TODO Restore cache usage below
    /*if (!cache.contains(pathString)) {
      ss << ".obj not found in cache";
      loadedObj = loadObj(pathString, yIsUp);
      cache.insert(pathString, loadedObj);
    } else {
      ss << "Loading .obj from cache";
    }*/
    // TODO Remove cache usage below
    if (!cache.contains(pathString)) {
      ss << ".obj not found in cache";
      loadedObj = loadObj(pathString, yIsUp);
      // cache.insert(pathString, loadedObj);
    } else {
      ss << "Loading .obj from cache";
    }

    logging::DEBUG(ss.str());
    if (loadedObj != nullptr) {
      // TODO Restore addObj below
      // primsOut->addObj(cache.get(pathString));
      // TODO Remove addObj below
      primsOut->addObj(loadedObj);
      primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
    }

    delete loadedObj;
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
  logging::DEBUG(ss.str());

  // Return

  return primsOut;
}

Vertex
WavefrontObjFileLoader::readVertex(std::vector<std::string> const& lineParts,
                                   bool const yIsUp)
{
  std::stringstream ss;
  Vertex v;

  // Read position:
  try {
    if (yIsUp) {
      v.pos = glm::dvec3(boost::lexical_cast<double>(lineParts[1]),
                         -boost::lexical_cast<double>(lineParts[3]),
                         boost::lexical_cast<double>(lineParts[2]));
    } else {
      v.pos = glm::dvec3(boost::lexical_cast<double>(lineParts[1]),
                         boost::lexical_cast<double>(lineParts[2]),
                         boost::lexical_cast<double>(lineParts[3]));
    }
  } catch (boost::bad_lexical_cast& e) {
    v.pos = glm::dvec3(0, 0, 0);
    ss << "Error reading vertex.\nEXCEPTION: " << e.what() << std::endl;
    logging::WARN(ss.str());
    ss.str("");
  }

  // ######## BEGIN Read vertex color #########
  if (lineParts.size() >= 7) {
    float r = 1, g = 1, b = 1;

    try {
      r = boost::lexical_cast<float>(lineParts[4]);
      g = boost::lexical_cast<float>(lineParts[5]);
      b = boost::lexical_cast<float>(lineParts[6]);
    } catch (boost::bad_lexical_cast& e) {
      ss << "Error reading vertex color.\nEXCEPTION: " << e.what() << std::endl;
      logging::WARN(ss.str());
      ss.str("");
    }

    Color4f color = Color4f(r, g, b, 1);
    v.color = color;
  }

  return v;
}

glm::dvec3
WavefrontObjFileLoader::readNormalVector(
  std::vector<std::string> const& lineParts,
  bool const yIsUp)
{
  try {
    if (yIsUp) {
      return glm::dvec3(boost::lexical_cast<double>(lineParts[1]),
                        -boost::lexical_cast<double>(lineParts[3]),
                        boost::lexical_cast<double>(lineParts[2]));
    }
    return glm::dvec3(boost::lexical_cast<double>(lineParts[1]),
                      boost::lexical_cast<double>(lineParts[2]),
                      boost::lexical_cast<double>(lineParts[3]));
  } catch (boost::bad_lexical_cast& e) {
    std::stringstream ss;
    ss << "Error reading normal vector.\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
  }

  return glm::dvec3{};
}

void
WavefrontObjFileLoader::readPrimitive(WavefrontObj* loadedObj,
                                      std::vector<std::string> const& lineParts,
                                      std::vector<Vertex> const& vertices,
                                      std::vector<glm::dvec2> const& texcoords,
                                      std::vector<glm::dvec3> const& normals,
                                      std::string const& currentMat,
                                      std::string const& pathString)
{

  std::stringstream ss;
  // ######### BEGIN Read triangle or quad ##############
  if (lineParts.size() >= 4 && lineParts.size() <= 5) {
    Vertex verts[4];

    // Try to read vertex position, texture and normal indices:
    try {
      for (size_t i = 0; i < lineParts.size() - 1; i++) {
        std::vector<std::string> fields;
        boost::split(fields, lineParts[i + 1], boost::is_any_of("/"));
        int vi = boost::lexical_cast<int>(fields[0]);
        int ti = 0;
        if (fields.size() > 1 && fields[1].length() > 0)
          ti = boost::lexical_cast<int>(fields[1]);
        int ni = 0;
        if (fields.size() > 2 && fields[2].length() > 0)
          ni = boost::lexical_cast<int>(fields[2]);
        buildPrimitiveVertex(
          verts[i], vertices.at(vi - 1), ti - 1, ni - 1, texcoords, normals);
      }
    } catch (std::exception& e) {
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
      Triangle* tri = new Triangle(verts[0], verts[1], verts[2]);
      tri->material = getMaterial(currentMat);
      //      primsOut->mPrimitives.push_back(tri);
      loadedObj->primitives.push_back(tri);
    }

    // Read a quad (two triangles):
    else if (lineParts.size() == 5) {
      Triangle* tri1 = new Triangle(verts[0], verts[1], verts[2]);
      tri1->material = getMaterial(currentMat);
      //      primsOut->mPrimitives.push_back(tri1);
      loadedObj->primitives.push_back(tri1);

      Triangle* tri2 = new Triangle(verts[0], verts[2], verts[3]);
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

WavefrontObj*
WavefrontObjFileLoader::loadObj(std::string const& pathString, bool const yIsUp)
{
  std::stringstream ss;

  WavefrontObj* loadedObj = new WavefrontObj();

  ss << "Reading 3D model from .obj file '" << pathString << "'...";
  logging::DEBUG(ss.str());
  ss.str("");

  fs::path filePath(pathString);
  if (!fs::exists(filePath)) {
    ss << "File not found: " << pathString << std::endl;
    logging::ERR(ss.str());
    exit(1);
  }
  try {
    helios::filems::BufferedLineFileReader lfr(pathString);
    std::vector<Vertex> vertices;
    std::vector<glm::dvec3> normals;
    std::vector<glm::dvec2> texcoords;
    std::string currentMat = "default";
    Material mat;
    mat.useVertexColors = true;
    mat.matFilePath = filePath.string();
    materials.insert(std::pair<std::string, Material>(currentMat, mat));
    std::string line;
    try {
      while (true) { // Loop until EndOfReadingException
        line = lfr.read();
        if (line.empty() || line == "\r" || line.substr(0, 1) == "#") {
          continue;
        }
        boost::algorithm::trim(line);
        std::vector<std::string> lineParts;
        boost::regex_split(
          std::back_inserter(lineParts), line, boost::regex("\\s+"));

        // Read vertex
        if (lineParts[0] == "v" && lineParts.size() >= 4) {
          // Add vertex to vertex list:
          vertices.push_back(
            WavefrontObjFileLoader::readVertex(lineParts, yIsUp));
        }

        // Read normal vector
        else if (lineParts[0] == "vn" && lineParts.size() >= 4) {
          normals.push_back(
            WavefrontObjFileLoader::readNormalVector(lineParts, yIsUp));
        }

        // Read texture coordinates
        else if (lineParts[0] == "vt" && lineParts.size() >= 3) {
          texcoords.push_back(
            glm::dvec2(boost::lexical_cast<double>(lineParts[1]),
                       boost::lexical_cast<double>(lineParts[2])));
        }

        // Read face
        else if (lineParts[0] == "f") {
          WavefrontObjFileLoader::readPrimitive(loadedObj,
                                                lineParts,
                                                vertices,
                                                texcoords,
                                                normals,
                                                currentMat,
                                                pathString);
        }

        // Read materials
        else if (lineParts[0] == "mtllib") {
          std::string s = filePath.parent_path().string() + "/" + lineParts[1];
          std::map<std::string, Material> mats =
            MaterialsFileReader::loadMaterials(s);
          materials.insert(mats.begin(), mats.end());
        }

        // Read material specification
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
    } catch (helios::filems::EndOfReadingException& eorex) {
      // This exception is okay, since it means the reader has finished
    } catch (std::exception& e) {
      ss << "Error reading primitives.\nEXCEPTION: " << e.what();
      logging::WARN(ss.str());
      ss.str("");
    }

    return loadedObj;
  } catch (std::exception& ex) {
    ss << "Unexpected exception when trying to read:\n\"" << pathString
       << "\"\n\t" << ex.what();
    logging::ERR(ss.str());
    ss.str("");
    throw ex;
  }
}

// ***  ASSIST METHODS  *** //
// ************************ //
void
WavefrontObjFileLoader::buildPrimitiveVertex(
  Vertex& dstVert,
  Vertex const& srcVert,
  int const texIdx,
  int const normalIdx,
  std::vector<glm::dvec2> const& texcoords,
  std::vector<glm::dvec3> const& normals)
{
  dstVert = srcVert.copy();
  if (texIdx >= 0)
    dstVert.texcoords = texcoords[texIdx];
  if (normalIdx >= 0)
    dstVert.normal = normals[normalIdx];
}
