#pragma once

#include <RaySceneIntersection.h>

#include <glm/glm.hpp>

#include <map>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Raycaster interface declaring raycasting operations
 */
class Raycaster{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~Raycaster() = default;

    // ***  RAYCASTING METHODS  *** //
    // **************************** //
    /**
	 * @brief Search all intersections for specified ray
	 * @param rayOrigin Ray origin 3D coordinates
	 * @param rayDir Ray 3D director vector
	 * @param tmin Minimum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) before this time during
	 *  the recursive search process
	 * @param tmax Maximum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) after this time during
	 *  the recursive search process
	 * @param groundOnly Flag to specify if only ground primitives must
	 *  be considered (true) or not (false)
	 * @return Return map of collected primitives, each identified by its
	 *  distance with respect to ray origin
	 */
    virtual std::map<double, Primitive*> searchAll(
        glm::dvec3 rayOrigin,
        glm::dvec3 rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) = 0;
    /**
	 * @brief Search first intersection for specified ray
	 * @param rayOrigin Ray origin 3D coordinates
	 * @param rayDir Ray 3D director vector
	 * @param tmin Minimum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) before this time during
	 *  the recursive search process
	 * @param tmax Maximum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) after this time during
	 *  the recursive search process
	 * @param groundOnly Flag to specify if only ground primitives must
	 *  be considered (true) or not (false)
	 * @return Return first found intersection
	 */
    virtual RaySceneIntersection * search(
        glm::dvec3 rayOrigin,
        glm::dvec3 rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) = 0;
};