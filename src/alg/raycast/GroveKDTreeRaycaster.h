#pragma once

#include <LightKDTreeNode.h>
#include <DynObject.h>
#include <KDTreeRaycaster.h>
#include <RaycasterGroveTree.h>
#include <KDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Grove KDTree Raycaster extends KDTreeRaycaster to make it compatible
 *  with groves by implementing the RaycasterGroveTree interface
 * @see RaycasterGroveTree
 * @see KDTreeRaycaster
 */
class GroveKDTreeRaycaster :
    public RaycasterGroveTree<DynObject>,
    public KDTreeRaycaster
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The KDTreeFactory to be used to rebuild the KDTree if necessary
     */
    std::shared_ptr<KDTreeFactory> kdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default Grove KDTree ray caster constructor
     * @param root Root node of the KDTree
     */
    GroveKDTreeRaycaster(
        std::shared_ptr<LightKDTreeNode> root,
        std::shared_ptr<KDTreeFactory> kdtf=nullptr
    ) :
        KDTreeRaycaster(root),
        kdtf(kdtf)
    {}
    virtual ~GroveKDTreeRaycaster() = default;

    // ***  RAYCASTING METHODS  *** //
    // **************************** //
    /**
	 * @see Raycaster::searchAll
	 */
    std::map<double, Primitive*> searchAll(
        glm::dvec3 rayOrigin,
        glm::dvec3 rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) override {return KDTreeRaycaster::searchAll(
        rayOrigin, rayDir, tmin, tmax, groundOnly
    );}
    /**
     * @see Raycaster::search
     */
    RaySceneIntersection * search(
        glm::dvec3 rayOrigin,
        glm::dvec3 rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) override {return KDTreeRaycaster::search(
        rayOrigin, rayDir, tmin, tmax, groundOnly
    );}

    // ***  GROVE DYNAMIC TREE METHODS  *** //
    // ************************************ //
    /**
     * @brief Method to handle callbacks from updated dynamic objects
     * @param dynObj The updated dynamic object
     * @see RaycasterGroveTree::update
     */
    void update(DynObject &dynObj) override;

    /**
     * @brief Make a temporal clone of the GroveKDTreeRaycaster
     *
     * The temporal clone is meant to produce a temporal copy of the tree. If
     *  the original tree is updated, then the temporal copy should not be
     *  updated.
     *
     * @return Temporal clone of the GroveKDTreeRaycaster
     * @see KDGrove::makeTemporalClone
     */
    virtual std::shared_ptr<GroveKDTreeRaycaster> makeTemporalClone() const;
};