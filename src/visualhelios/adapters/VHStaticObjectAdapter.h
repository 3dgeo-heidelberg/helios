#ifdef PCL_BINDING

#pragma once

#include <assetloading/ScenePart.h>

#include <pcl/PolygonMesh.h>
#include <pcl/common/common_headers.h>

#include <string>
#include <vector>

namespace visualhelios {

using std::string;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class defining core mechanisms to adapt static objects
 *  to the visual Helios context based on PCL and VTK libraries
 */
class VHStaticObjectAdapter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The adapted static object
   * @see ScenePart
   */
  ScenePart& staticObj;
  /**
   * @brief Static object vertices connection specification through ordered
   *  indices for visualization purposes.
   *
   * It is, how the vertices must be connected to render each primitive in
   *  the polygon mesh.
   */
  vector<pcl::Vertices> vertices;
  /**
   * @brief Specify if the static object normals must be rendered (true) or
   *  not (false)
   */
  bool renderingNormals;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios static object adapter
   * @param staticObj Static object to be adapted for visual Helios
   */
  VHStaticObjectAdapter(ScenePart& staticObj)
    : staticObj(staticObj)
    , renderingNormals(true)
  {
  }
  virtual ~VHStaticObjectAdapter() = default;

  // ***  BUILDING  *** //
  // ****************** //
  /**
   * @brief Build the polygon mesh from static object primitives. This
   *  implies building corresponding vertices vector too.
   *
   * @see VHStaticObjectAdapter::polymesh
   * @see VHStaticObjectAdapter::vertices
   */
  virtual void buildPolymesh();
  /**
   * @brief Instantiate the polymesh object.
   *
   * The buildPolymesh method requires to call this function to instantiate
   *  a new mesh replacing the old one if any. Therefore, any concrete
   *  implementation of a VHStaticObjectAdapter must provide its own
   *  definition for this method.
   *
   * @see VHStaticObjectAdapter::buildPolymesh
   * @see VHStaticObjectAdapter::vertexToMesh(Vertex const &)
   */
  virtual void constructPolymesh() = 0;
  /**
   * @brief Add a vertex to the polymesh.
   *
   * The buildPolymesh method requires to call this function to generate
   *  points/vertices defining the mesh from static object primitives.
   *  Therefore, any concrete implementation of a VHStaticObjectAdapter must
   *  provide its own definition for this method.
   *
   * @param vertex Vertex from the static object that must be added to
   *  the mesh
   * @see VHStaticObjectAdapter::buildPolymesh
   * @see VHStaticObjectAdapter::constructPolymesh
   */
  virtual void vertexToMesh(Vertex const& vertex) = 0;

  // ***  UTILS  *** //
  // *************** //
  /**
   * @brief Function to add triangle primitives to the polymesh during
   *  building time
   * @param primitive The triangle primitive to be added
   * @param offset The offset for vertex index at current iteration
   * @see VHStaticObjectAdapter::buildPolymesh
   * @see VHStaticObjectAdapter::addVoxelToPolymesh
   */
  virtual void addTriangleToPolymesh(Primitive* primitive, int& offset);
  /**
   * @brief Function to add voxel primitives to the polymesh during
   *  building time
   * @param primitive The triangle primitive to be added
   * @param offset The offset for vertex index at current iteration
   * @see VHStaticObjectAdapter::buildPolymesh
   * @see VHStaticObjectAdapter::addTriangleToPolymesh
   */
  virtual void addVoxelToPolymesh(Primitive* primitive, int& offset);

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the static object
   *
   * <b><span style="color: red;">WARNING</span></b> this getter returns the
   *  static object (scene part) reference allowing modifications.
   *  Use with caution.
   *
   * @return Static object
   */
  inline ScenePart& getStaticObj() { return staticObj; }
  /**
   * @brief Obtain the ordered vertices indices representing the static
   *  object
   * @return Ordered vertices indices representing the static object
   */
  inline vector<pcl::Vertices> const& getVertices() const { return vertices; }
  /**
   * @brief Obtain the ID of the static object
   * @return Static object ID
   */
  inline string const& getId() const { return staticObj.mId; }
  /**
   * @brief Check whether the static object normals must be rendered or not
   * @return True if static objects normals must be rendered,
   *  false otherwise
   * @see VHStaticObjectAdapter::renderingNormals
   */
  inline bool isRenderingNormals() const { return renderingNormals; }
  /**
   * @brief Enable or disable normals rendering for the static object
   * @param renderingNormals True to enable rendering normals, false to
   *  disable it
   * @see VHStaticObjectAdapter::renderingNormals
   */
  inline void setRenderingNormals(bool const renderingNormals)
  {
    this->renderingNormals = renderingNormals;
  }
};
}

#endif
