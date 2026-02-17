#pragma once

#include <glm/glm.hpp>
#include <helios/assetloading/geometryfilter/AbstractGeometryFilter.h>
#include <helios/assetloading/geometryfilter/WavefrontObj.h>

/**
 * @brief OBJ file loader filter
 */
class WavefrontObjFileLoader : public AbstractGeometryFilter
{
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
  WavefrontObjFileLoader()
    : AbstractGeometryFilter(new ScenePart())
  {
  }

  // ***  MAIN METHODS *** //
  // ********************* //
  /**
   * @see AbstractGeometryFilter::run
   */
  ScenePart* run() override;

  /**
   * @brief Load a vertice from a given line
   * @param lineParts Vector containing each vertex coordinate in string format
   */
  Vertex readVertex(std::vector<std::string> const& lineParts,
                    bool const yIsUp);

  /**
   * @brief Read a normal vector from a given line
   * @param lineParts Vector containing each normal coordinate in string format
   */
  glm::dvec3 readNormalVector(std::vector<std::string> const& lineParts,
                              bool const yIsUp);

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
  void readPrimitive(WavefrontObj* loadedObj,
                     std::vector<std::string> const& lineParts,
                     std::vector<Vertex> const& vertices,
                     std::vector<glm::dvec2> const& texcoords,
                     std::vector<glm::dvec3> const& normals,
                     std::string const& currentMat,
                     std::string const& pathString);

  /**
   * @brief Load an OBJ file
   */
  WavefrontObj* loadObj(std::string const& pathString, bool const yIsUp);

  // ***  ASSIST METHODS  *** //
  // ************************ //
  /**
   * @brief Build dstVertex considering data of srcVert.
   * @param dstVert Vertex to be built. It belongs to a primitive.
   * @param[in] srcVert Vertex used to built. It comes from the set of vertices
   * in obj file.
   * @param texIdx Index of texture to be used
   * @param normalIdx Index of normal to be used.
   * @param texcoords Vector of texture coordinates.
   * @param normals Vector of normals. It contains the normal for each
   * vertex.
   */
  static void buildPrimitiveVertex(Vertex& dstVert,
                                   Vertex const& srcVert,
                                   int const texIdx,
                                   int const normalIdx,
                                   std::vector<glm::dvec2> const& texcoords,
                                   std::vector<glm::dvec3> const& normals);
};
