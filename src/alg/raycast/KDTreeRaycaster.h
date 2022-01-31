#pragma once

// TODO 4: Merge search_recursive() and search_all_recursive?

#include <Raycaster.h>
#include <Primitive.h>
#include <Material.h>
#include <RaySceneIntersection.h>
#include <LightKDTreeNode.h>

/**
 * @brief Class representing a KDTree ray caster
 *
 * @see Raycaster
 */
class KDTreeRaycaster : public Raycaster {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for the ray caster
     */
	double epsilon = 0.0001;
	/**
	 * @brief Flag to specify if only ground points must be considered (true)
	 *  or not (false)
	 */
	bool groundOnly = false;
	/**
	 * @brief Ray 3D director vector
	 */
	glm::dvec3 rayDir;
	/**
	 * @brief Ray origin 3D coordinates
	 */
	glm::dvec3 rayOrigin;
	/**
	 * @brief Vector containing components of ray director vector. It is filled
	 *  at the start of a search operation
     * @see KDTreeRaycaster::searchAll
     * @see KDTreeRaycaster::search
	 */
	std::vector<double> rayDirArray;
	/**
	 * @brief Vector containing components of ray origin. It is filled at the
	 *  start of a search operation
     * @see KDTreeRaycaster::searchAll
     * @see KDTreeRaycaster::search
	 */
	std::vector<double> rayOriginArray;
	/**
	 * @brief Shared pointer to the root node of the KDTree
	 */
	std::shared_ptr<LightKDTreeNode> root;
	/**
	 * @brief Distance of closest hit. It is numeric_limits<double>::max() by
	 *  default
	 */
	double closestHitDistance = std::numeric_limits<double>::max();
	/**
	 * @brief Map of primitives identified by its distance with respect to
	 *  ray origin. Only primitives which intersect with the ray are considered
	 *  (i.e. those which distance with respect to ray origin is greater than
	 *  0)
	 */
	std::map<double, Primitive*> collectedPoints;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief KDTree ray caster constructor
	 * @param root Root node of the KDTree
	 */
	KDTreeRaycaster(std::shared_ptr<LightKDTreeNode> root) : root(root) {}
    virtual ~KDTreeRaycaster() = default;

	// ***  RAYCASTING METHODS  *** //
	// **************************** //
	/**
	 * @see Raycaster::searchAll
	 */
    std::map<double, Primitive*> searchAll(
        glm::dvec3 _rayOrigin,
        glm::dvec3 _rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) override;
    /**
     * @see Raycaster::search
     */
    RaySceneIntersection * search(
        glm::dvec3 _rayOrigin,
        glm::dvec3 _rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) override;

protected:
    // ***  RAYCASTING UTILS  *** //
    // ************************** //
    /**
     * @brief Recursive search function to assist searchAll function
     * @param node KDTree node to be recursively explored
     * @param tmin Minimum time to intersection. It is used to prevent
     *  considering intersections (capturing points) before this time during
     *  the recursive search process
     * @param tmax Maximum time to intersection. It is used to prevent
     *  considering intersections (capturing points) after this time during
     *  the recursive search process
     * @see KDTreeRaycaster::searchAll
     */
	void searchAll_recursive(LightKDTreeNode* node, double tmin, double tmax);
	/**
	 * @brief Recursive search function to assist search function
	 * @param node KDTree node to be recursively explored
	 * @param tmin Minimum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) before this time during
	 *  the recursive search process
	 * @param tmax Maximum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) after this time during
	 *  the recursive search process
	 * @return Return first found intersection
	 * @see KDTreeRaycaster::search
	 */
	Primitive* search_recursive(
	    LightKDTreeNode* node,
	    double tmin,
	    double tmax
    );
};