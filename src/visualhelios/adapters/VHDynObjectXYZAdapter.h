#ifdef PCL_BINDING

#pragma once

#include <visualhelios/adapters/VHDynObjectAdapter.h>
#include <visualhelios/adapters/VHStaticObjectXYZAdapter.h>

namespace visualhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing concrete implementation of a VHDynObjectAdapter for
 *  a simple XYZ visualization with no color nor intensity
 *
 * @see visualhelios::VHDynObjectAdapter
 */
class VHDynObjectXYZAdapter :
    public VHStaticObjectXYZAdapter,
    public VHDynObjectAdapter
{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the visual Helios dynamic object adapter
     *  providing XYZ visualization
     * @see visualhelios::VHDynObjectAdapter::VHDynObjectAdapter(DynObject &)
     */
    VHDynObjectXYZAdapter(DynObject &dynObj) :
        VHStaticObjectAdapter(static_cast<ScenePart &>(dynObj)),
        VHStaticObjectXYZAdapter(static_cast<ScenePart &>(dynObj)),
        VHDynObjectAdapter(dynObj)
    {}
    virtual ~VHDynObjectXYZAdapter() = default;
};

}

#endif