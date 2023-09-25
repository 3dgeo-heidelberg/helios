#pragma once

#include <LightKDTreeNode.h>
#include <DynObject.h>
#include <KDTreeRaycaster.h>
#include <RaycasterGroveTree.h>
#include <KDTreeFactory.h>
#include <adt/custom/PointerVector.h>

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

    // CACHE ATTRIBUTES
    /**
     * @brief The cache of primitives defining the last state for the root node
     *  of the raycasting process
     */
    std::shared_ptr<PointerVector<Primitive>> cache_prims;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default Grove KDTree ray caster constructor
     * @param root Root node of the KDTree
     */
    GroveKDTreeRaycaster(
        std::shared_ptr<LightKDTreeNode> root,
        std::shared_ptr<KDTreeFactory> kdtf=nullptr,
        std::shared_ptr<PointerVector<Primitive>> cache_prims=nullptr
    ) :
        KDTreeRaycaster(root),
        kdtf(kdtf),
        cache_prims(cache_prims)
    {}
    /**
     * @brief The destructor of Grove KDTree must destroy any cache-related
     *  resource that doesnt make sense after the time-of-live of the raycaster
     *  has finished
     */
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
#ifdef DATA_ANALYTICS
       ,std::vector<double> &subraySimRecord,
        bool const isSubray=false
#endif
    ) override {return KDTreeRaycaster::search(
        rayOrigin, rayDir, tmin, tmax, groundOnly
#ifdef DATA_ANALYTICS
       ,subraySimRecord,
        isSubray
#endif
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

    /**
     * @brief Generate a shared pointer to a copy of the given vector of
     *  primitives. Copy implies that primitives are cloned. Thus, deleting
     *  copied primitives will not delete source primitives.
     * @param src The source primitives to be copied
     * @return Shared pointer to a vector of cloned primitives
     */
    std::shared_ptr<PointerVector<Primitive>> sharedCopy(
        std::vector<Primitive *> const &src
    ) const;
};