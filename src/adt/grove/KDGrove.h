#pragma once

#include <BasicDynGrove.h>
#include <GroveKDTreeRaycaster.h>
#include <DynMovingObject.h>
#include <KDGroveStats.h>

#include <string>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Grove of KDTrees. It supports both static and dynamic KDTrees,
 *  handling each accordingly.
 * @see BasicDynGrove
 */
class KDGrove : public BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Stats of the KDGgrove
     * @see KDGroveStats
     */
    std::shared_ptr<KDGroveStats> stats;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for KDGrove
     */
    KDGrove() :
        BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>(),
        stats(nullptr)
    {}
    virtual ~KDGrove() = default;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the KDGrove stats
     * @return KDGrove stats
     */
    virtual std::shared_ptr<KDGroveStats> getStats()
    {return stats;}
    /**
     * @brief Set the KDGrove stats
     * @param stats New KDGrove stats
     */
    virtual void setStats(std::shared_ptr<KDGroveStats> stats)
    {this->stats = stats;}
};