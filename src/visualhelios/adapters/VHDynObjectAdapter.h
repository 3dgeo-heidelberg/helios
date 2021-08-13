#ifdef PCL_BINDING

#pragma once

#include <scene/dynamic/DynObject.h>

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>

namespace visualhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class defining core mechanisms to adapt dynamic objects
 *  to the visual Helios context based on PCL and VTK libraries
 */
class VHDynObjectAdapter {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The adapted dynamic object
     * @see DynObject
     */
    DynObject &dynObj;
    /**
     * @brief Dynamic object vertices connection specification through ordered
     *  indices for visualization purposes.
     *
     * It is, how the vertices must be connected to render each primitive in
     *  the polygon mesh
     */
    vector<pcl::Vertices> vertices;
    /**
     * @brief Specify if the dynamic object normals must be rendered (true) or
     *  not (false)
     */
    bool renderingNormals;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the visual Helios dynamic object adapter
     * @param dynObj Dynamic object to be adapted for visual Helios
     */
    VHDynObjectAdapter(DynObject &dynObj) : dynObj(dynObj) {}
    virtual ~VHDynObjectAdapter() = default;

    // ***  BUILDING  *** //
    // ****************** //
    /**
     * @brief Build the polygon mesh from dynamic object primitives. This
     *  implies building corresponding vertices vector too.
     *
     * @see VHDynObjectAdapter::polymesh
     * @see VHDynObjectAdapter::vertices
     */
    void buildPolymesh();
    /**
     * @brief Instantiate the polymesh object.
     *
     * The buildPolymesh method requires to call this function to instantiate
     *  a new mesh replacing the old one if any. Therefore, any concrete
     *  implementation of a VHDynObjectAdapter must provide its own definition
     *  for this method.
     *
     * @see VHDynObjectAdapter::buildPolymesh
     * @see VHDynObjectAdapter::vertexToMesh(Vertex const &)
     */
    virtual void constructPolymesh() = 0;
    /**
     * @brief Add a vertex to the polymesh.
     *
     * The buildPolymesh method requires to call this function to generate
     *  points/vertices defining the mesh from dynamic object primitives.
     *  Therefore, any concrete implementation of a VHDynObjectAdapter must
     *  provide its own definition for this method.
     *
     * @param vertex Vertex from the dynamic object that must be added to
     *  the mesh
     * @see VHDynObjectAdapter::buildPolymesh
     * @see VHDynObjectAdapter::constructPolymesh
     */
    virtual void vertexToMesh(Vertex const & vertex) = 0;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Method to adapt dynamic object computations over time to visual
     *  Helios
     *
     * @return True if the dynamic object was modified, false otherwise
     */
    bool doStep();

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the adapted dynamic object
     *
     * <b><span style="color: red;">WARNING</span></b> this getter returns the
     *  dynamic object reference allowing modifications. Use with caution.
     *
     * @return Adapted dynamic object
     */
    inline DynObject & getDynObj() {return dynObj;}
    /**
     * @brief Obtain the ordered vertices indices representing the dynamic
     *  object
     * @return Ordered vertices indices representing the dynamic object
     */
    inline vector<pcl::Vertices> const & getVertices() const
    {return vertices;}
    /**
     * @brief Obtain the ID of the dynamic object
     * @return Dynamic object ID
     */
    inline string const & getId() const {return dynObj.getId();}
    /**
     * @brief Check whether the dynamic object normals must be rendered or not
     * @return True if dynamic objects normals must be rendered, false otherwise
     * @see VHDynObjectAdapter::renderingNormals
     */
    inline bool isRenderingNormals() const {return renderingNormals;}
    /**
     * @brief Enable or disable normals rendering for the dynamic object
     * @param renderingNormals True to enable rendering normals, false to
     *  disable it
     * @see VHDynObjectAdapter::renderingNormals
     */
    inline void setRenderingNormals(bool const renderingNormals)
    {this->renderingNormals = renderingNormals;}

};


}

#endif