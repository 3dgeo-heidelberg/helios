#pragma once

#include <Raycaster.h>
#include <KDGrove.h>
#include <Primitive.h>

#include <glm/glm.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a KDGrove ray caster
 *
 * @see Raycaster
 * @see KDTreeRaycaster
 * @see GroveKDTreeRaycaster
 * @see KDGrove
 */
class KDGroveRaycaster : public Raycaster{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The KDGrove for the ray casting
     */
    std::shared_ptr<KDGrove> grove;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @bried KDGrove ray caster constructor
     * @param grove The KDGrove for the ray casting
     */
    KDGroveRaycaster(std::shared_ptr<KDGrove> grove) : grove(grove) {}
    virtual ~KDGroveRaycaster() = default;

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
    ) override;
    /**
     * @see Raycaster::search
     */
    RaySceneIntersection * search(
        glm::dvec3 rayOrigin,
        glm::dvec3 rayDir,
        double tmin,
        double tmax,
        bool groundOnly
    ) override;
};