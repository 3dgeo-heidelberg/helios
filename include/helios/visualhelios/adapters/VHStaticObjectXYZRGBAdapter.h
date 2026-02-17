#ifdef PCL_BINDING

#pragma once

#include <helios/visualhelios/adapters/VHStaticObjectAdapter.h>

namespace visualhelios {

class VHStaticObjectXYZRGBAdapter : virtual public VHStaticObjectAdapter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Polygon mesh representing the static object in a
   *  \f$\mathbb{R}^{3}\f$ space with RGB color
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr polymesh;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios static object adapter
   *  providing XYZ visualization with RGB color
   * @see VHStaticObjectAdapter::VHStaticObjectAdapter(ScenePart &)
   */
  VHStaticObjectXYZRGBAdapter(ScenePart& staticObj)
    : VHStaticObjectAdapter(staticObj)
  {
  }
  virtual ~VHStaticObjectXYZRGBAdapter() = default;

  // ***  BUILDING  *** //
  // ****************** //
  /**
   * @see visualhelios::VHStaticObjectAdapter::constructPolymesh
   */
  void constructPolymesh() override;
  /**
   * @see visualhelios::VHStaticObjectAdapter::vertexToMesh
   */
  void vertexToMesh(Vertex const& vertex) override;

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the polygon mesh representing the static object
   *  in a \f$\mathbb{R}^{3}\f$ space with RGB color
   * @return Polygon mesh representing the static object in a
   *  \f$\mathbb{R}^{3}\f$ space with RGB color
   */
  inline pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getPolymesh() const
  {
    return polymesh;
  }
};
}

#endif
