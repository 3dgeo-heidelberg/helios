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








// ***  NESTED DATASTRUCTURE-LIKE CLASS  *** //
// ***************************************** //
/**
 * @brief Data for search operations of KDTree raycaster
 */
class KDTreeRaycasterSearch{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief Ray 3D director vector
	 */
    glm::dvec3 const rayDir;
    /**
	 * @brief Ray origin 3D coordinates
	 */
    glm::dvec3 const rayOrigin;
    /**
	 * @brief Flag to specify if only ground points must be considered (true)
	 *  or not (false)
	 */
    bool const groundOnly;
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
	 * @brief Distance of closest hit. It is numeric_limits<double>::max() by
	 *  default
	 */
    double closestHitDistance;
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
     * @brief Default constructo for KDTreeRaycasterSearch
     */
    KDTreeRaycasterSearch(
        glm::dvec3 const rayDir,
        glm::dvec3 const rayOrigin,
        bool const groundOnly=false
    ):
        rayDir(rayDir),
        rayOrigin(rayOrigin),
        groundOnly(groundOnly),
        closestHitDistance(std::numeric_limits<double>::max())
    {}
    virtual ~KDTreeRaycasterSearch() = default;
};








public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for the ray caster
     */
	double epsilon = 0.0001;
	/**
	 * @brief Shared pointer to the root node of the KDTree
	 */
	std::shared_ptr<LightKDTreeNode> root;

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
	 * @see KDTreeRaycaster::searchAll_recursive
	 */
    std::map<double, Primitive*> searchAll(
        glm::dvec3 const rayOrigin,
        glm::dvec3 const rayDir,
        double const tmin,
        double const tmax,
        bool const groundOnly
    ) override;
    /**
     * @see Raycaster::search
     * @see KDTreeRaycaster::search_recursive
     */
    RaySceneIntersection * search(
        glm::dvec3 const rayOrigin,
        glm::dvec3 const rayDir,
        double const tmin,
        double const tmax,
        bool const groundOnly
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
	 * @param search The KDTreeRaycasterSearch data structure for current
	 *  search process
     * @see KDTreeRaycaster::searchAll
     */
	void searchAll_recursive(
	    LightKDTreeNode* node,
	    double const tmin,
	    double const tmax,
	    KDTreeRaycasterSearch &search
    );
	/**
	 * @brief Recursive search function to assist search function
	 * @param node KDTree node to be recursively explored
	 * @param tmin Minimum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) before this time during
	 *  the recursive search process
	 * @param tmax Maximum time to intersection. It is used to prevent
	 *  considering intersections (capturing points) after this time during
	 *  the recursive search process
	 * @param search The KDTreeRaycasterSearch data structure for current
	 *  search process
	 * @return Return first found intersection
	 * @see KDTreeRaycaster::search
	 */
	Primitive* search_recursive(
	    LightKDTreeNode* node,
	    double const tmin,
	    double const tmax,
	    KDTreeRaycasterSearch &search
    ) const;
};