#ifdef PCL_BINDING
#pragma once

#include <helios/demo/BaseDemo.h>
#include <helios/scene/primitives/Triangle.h>
#include <helios/visualhelios/adapters/VHDynObjectXYZAdapter.h>

#include <memory>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common_headers.h>
#include <vector>

namespace HeliosDemos {

using visualhelios::VHDynObjectXYZAdapter;

using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Simple primitives demo
 *
 * This demo implements the rendering of simple objects performing different
 *  motions
 */
class SimplePrimitivesDemo : public BaseDemo
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple primitives demo constructor
   */
  SimplePrimitivesDemo()
    : BaseDemo("Simple primitives demo")
  {
  }
  virtual ~SimplePrimitivesDemo() = default;

  // ***  R U N  *** //
  // *************** //
  /**
   * @see BaseDemo::run
   */
  void run() override;

  // ***  U T I L  *** //
  // ***************** //
  /**
   * @brief Build the dynamic object representing the mobile structure
   * @return Dynamic object representing the mobile structure
   */
  shared_ptr<DynObject> buildMobileStructure();
  /**
   * @brief Build the dynamic object representing the fixed structure
   * @return Dynamic object representing the fixed structure
   */
  shared_ptr<DynObject> buildFixedStructure();
  /**
   * @brief Build the dynamic object representing the helical structure
   * @return Dynamic object representing the helical structure
   */
  shared_ptr<DynObject> buildHelicalStructure();
  /**
   * @brief Build the dynamic object representing the static structure
   * @return Dynamic object representing the static structure
   */
  shared_ptr<DynObject> buildStaticStructure();
  /**
   * @brief Build the dynamic object representing the ground structure
   * @return Dynamic object representing the ground structure
   */
  shared_ptr<DynObject> buildGroundStructure();
};

}
#endif
