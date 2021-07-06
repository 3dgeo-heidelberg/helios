#pragma once

#include <vector>

#include "Vertex.h"
class Primitive;
class WavefrontObj;

#include "maths/Rotation.h"
#include <ogr_geometry.h>
#include "LadLut.h"
#include <iostream>

/**
 * @brief Class representing a scene part
 */
class ScenePart {
    // ***  SERIALIZATION  *** //
    // *********************** //
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & mPrimitives;
		ar & mId;
		ar & mOrigin;
		ar & mRotation;
		ar & mScale;
		ar & onRayIntersectionMode;
		ar & onRayIntersectionArgument;
		ar & randomShift;
		ar & ladlut;
	}
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Vector of pointers to primitives used by this scene part
     */
	std::vector<Primitive*> mPrimitives;
	/**
	 * @brief Identifier for the scene part
	 */
	std::string mId = "";
	/**
	 * @brief Vector specifying the limit of a subpart as the index of first
	 *  element of next subpart.
	 *
	 * This vector makes sense when a scene part is built from multiple
	 *  objects, so each one must be placed on its own scene part.
	 *
	 * The ith subpart is defined inside interval \f$[u[i-1], u[i])\f$, where
	 *  \f$u\f$ is the subpartLimit vector. For the first case, the interval
	 *  is defined as \f$[0, u[i])\f$
	 * Having \f$|u| = 1\f$ means there is only one scene part
	 */
	std::vector<size_t> subpartLimit;
	/**
	 * @brief Specify the handling mode for ray intersections
	 */
	std::string onRayIntersectionMode = "";
	/**
	 * @brief Specify the extra value to be used for ray intersection handling
	 * computation, when needed (depends on mode).
	 */
	double onRayIntersectionArgument = 0.0;
	/**
	 * @brief Specify if apply random shift to the scene part (true)
	 * or not (false, by default)
	 */
	bool randomShift = false;

	/**
	 * @brief Look-up table for leaf angle distribution
	 */
	std::shared_ptr<LadLut> ladlut = nullptr;

	/**
	 * @brief Specify the origin for the scene part
	 */
	glm::dvec3 mOrigin = glm::dvec3(0, 0, 0);
	/**
	 * @brief Specify the rotation for the scene part
	 */
	Rotation mRotation = Rotation(glm::dvec3(1, 0, 0), 0);
	/**
	 * @brief Specify the scale for the scene part
	 */
	double mScale = 1;

	OGRSpatialReference *mCrs = nullptr;
	OGREnvelope *mEnv = nullptr;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Default constructor for a scene part
	 */
	ScenePart() = default;
	ScenePart(ScenePart &sp);
	virtual ~ScenePart(){}

	// ***  M E T H O D S  *** //
	// *********************** //

        /**
         * @brief Add the primitives of a WavefrontObj to the ScenePart
         * @param obj Pointer to a loaded OBJ
         */
        void addObj(WavefrontObj * obj);
	/**
	 * @brief Obtain all vertices in the scene part
	 * @return All vertices in the scene part
	 */
	std::vector<Vertex*> getAllVertices();

	/**
	 * @brief Smooth normals for each vertex computing the mean for each
	 * triangle using it
	 */
	void smoothVertexNormals();

	/**
	 * @brief Split each subpart into a different scene part, with the first
	 *  one corresponding to this scene part
	 * @see subpartLimit
	 * @return True when split was successfully performed, false otherwise
	 */
        bool splitSubparts();

        /**
         * @brief Copy assigment operator.
         */
         ScenePart & operator=(const ScenePart & rhs);
};