#pragma once

#include <Raycaster.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Define a Raycaster derived interface to make it compatible with
 *  groves
 *
 * This interface is meant to support dynamic trees behavior,
 *  thus it assures that an update method exists for any of its instances.
 *
 * @see StaticGrove
 * @see DynGrove
 * @see BasicStaticGrove
 * @see BasicDynGrove
 * @see KDGrove
 */
template <typename Subject>
class RaycasterGroveTree : Raycaster{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~RaycasterGroveTree() = default;


    // ***  GROVE DYNAMIC TREE METHODS  *** //
    // ************************************ //
    /**
     * @brief Method to handle callbacks from updated subjects
     * @param s The updated subject
     */
    virtual void update(Subject &s) = 0;
};