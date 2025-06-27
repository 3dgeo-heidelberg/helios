#ifdef PCL_BINDING

#pragma once

#include <visualhelios/adapters/VHStaticObjectAdapter.h>

namespace visualhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing concrete implementation of a VHStaticObjectAdapter
 *  for a simple XYZ visualization with no color nor intensity
 *
 * @see visualhelios::VHStaticObjectAdapter
 */
class VHStaticObjectXYZAdapter : virtual public VHStaticObjectAdapter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Polygon mesh representing the static object in a
   *  \f$\mathbb{R}^{3}\f$ space with no color nor intensity
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr polymesh;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios static object adapter
   *  providing XYZ visualization
   * @see VHStaticObjectAdapter::VHStaticObjectAdapter(ScenePart &)
   */
  VHStaticObjectXYZAdapter(ScenePart& staticObj)
    : VHStaticObjectAdapter(staticObj)
  {
  }
  virtual ~VHStaticObjectXYZAdapter() = default;

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
   *  in a \f$\mathbb{R}^{3}\f$ space with no color nor intensity
   * @return Polygon mesh representing the static object in a
   *  \f$\mathbb{R}^{3}\f$ space with no color nor intensity
   */
  inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPolymesh() const
  {
    return polymesh;
  }
};

}

#endif
