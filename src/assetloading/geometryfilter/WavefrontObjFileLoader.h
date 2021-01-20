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
	ScenePart* run();

	/**
	 * @brief Load an OBJ file
	 */
    void loadObj(std::string const & pathString, bool yIsUp);

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
    static void buildPrimitiveVertex(
        Vertex &dstVert,
        Vertex & srcVert,
        int texIdx,
        int normalIdx,
        std::vector<dvec2> const & texcoords,
        std::vector<dvec3> const & normals
    );
};