#ifdef PCL_BINDING

#pragma once

#include <visualhelios/adapters/VHStaticObjectAdapter.h>
#include <scene/dynamic/DynObject.h>


namespace visualhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class defining core mechanisms to adapt dynamic objects
 *  to the visual Helios context based on PCL and VTK libraries
 */
class VHDynObjectAdapter : virtual public VHStaticObjectAdapter {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the visual Helios dynamic object adapter
     * @param dynObj Dynamic object to be adapted for visual Helios
     */
    VHDynObjectAdapter(DynObject &dynObj) :
        VHStaticObjectAdapter(static_cast<ScenePart&>(dynObj))
    {}
    virtual ~VHDynObjectAdapter() = default;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Method to adapt dynamic object computations over time to visual
     *  Helios
     *
     * @param forceStep True to specify that underlying dynamic object must be
     *  forced to execute its doStep method, false otherwise. Forcing step
     *  might be useful to force dynamic object step computation when it does
     *  not belong to a dynamic scene which handles doStep calls.
     * @param forceRebuild True to specify the polygon mesh must be rebuilt,
     *  false otherwise.
     *
     * @return True if the dynamic object was modified, false otherwise
     */
    bool doStep(bool const forceStep=false, bool const forceRebuild=false);

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the dynamic object
     *
     * <b><span style="color: red;">WARNING</span></b> this getter returns the
     *  dynamic object reference allowing modifications. Use with caution.
     *
     * @return Dynamic object
     */
    inline DynObject & getDynObj()
    {return static_cast<DynObject &>(staticObj);}

};


}

#endif