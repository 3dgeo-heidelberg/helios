#pragma once

#include "AbstractGeometryFilter.h"
#include <glm/glm.hpp>
using namespace glm;

/**
 * @brief OBJ file loader filter
 */
class WavefrontObjFileLoader : public AbstractGeometryFilter {
  /**
   * @brief Path to the OBJ file
   */
  std::string filePathString = "";

public:
  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Constructor for OBJ file loader filter
   * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
   */
  WavefrontObjFileLoader() : AbstractGeometryFilter(new ScenePart()) {}

  // ***  MAIN METHODS *** //
  // ********************* //
  /**
   * @see AbstractGeometryFilter::run
   */
  ScenePart *run();

  /**
   * @brief Load a vertice from a given line
   * @param lineParts Vector containing each vertex coordinate in string format
   * @param yIsUp
   * @return
   */
  Vertex readVertex(std::vector<std::string> &lineParts, bool yIsUp);

  /**
   * @brief Read a normal vector from a given line
   * @param lineParts Vector containing each normal coordinate in string format
   * @param yIsUp
   * @return
   */
  dvec3 readNormalVector(std::vector<std::string> &lineParts, bool yIsUp);

  /**
   * @brief Reads a face from a given line
   * @param lineParts Vector of strings containing the primitive indices
   * @param vertices Vertices used to build. They come from the set of vertices
   * in the obj file
   * @param texcoords Coordinates of the texture to be loaded in the face
   * @param normals Vector of normales of every face in the .obj file
   * @param currentMat Current material to be used in the primitive
   * @param pathString Path to the primitive
   */
  void readPrimitive(std::vector<std::string> &lineParts,
                     std::vector<Vertex> &vertices,
                     std::vector<dvec2> &texcoords, std::vector<dvec3> &normals,
                     std::string &currentMat, const std::string &pathString);

  /**
   * @brief Load an OBJ file
   */
  void loadObj(std::string const &pathString, bool yIsUp);

  // ***  ASSIST METHODS  *** //
  // ************************ //
  /**
   * @brief Build dstVertex considering data of srcVert.
   * @param dstVert Vertex to be built. It belongs to a primitive.
   * @param srcVert Vertex used to built. It comes from the set of vertices
   * in obj file.
   * @param texIdx Index of texture to be used
   * @param normalIdx Index of normal to be used.
   * @param texcoords Vector of texture coordinates.
   * @param normals Vector of normals. It contains the normal for each
   * vertex.
   */
  static void buildPrimitiveVertex(Vertex &dstVert, Vertex &srcVert, int texIdx,
                                   int normalIdx,
                                   std::vector<dvec2> const &texcoords,
                                   std::vector<dvec3> const &normals);
};