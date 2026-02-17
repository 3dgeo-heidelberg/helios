#ifdef PCL_BINDING

#pragma once

#include <helios/visualhelios/adapters/VHDynObjectAdapter.h>
#include <helios/visualhelios/adapters/VHStaticObjectXYZRGBAdapter.h>

namespace visualhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing concrete implementation of a VHDynObjectAdapter for
 *  a simple XYZ visualization with RGB color
 */
class VHDynObjectXYZRGBAdapter
  : public VHStaticObjectXYZRGBAdapter
  , public VHDynObjectAdapter
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios dynamic object adapter
   *  providing XYZ visualization with RGB color
   * @see visualhelios::VHDynObjectAdapter::VHDynObjectAdapter(DynObject &)
   */
  VHDynObjectXYZRGBAdapter(DynObject& dynObj)
    : VHStaticObjectAdapter(static_cast<ScenePart&>(dynObj))
    , VHStaticObjectXYZRGBAdapter(static_cast<ScenePart&>(dynObj))
    , VHDynObjectAdapter(dynObj)
  {
  }
  virtual ~VHDynObjectXYZRGBAdapter() = default;
};

}

#endif
