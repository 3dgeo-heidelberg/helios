#ifdef PCL_BINDING

#pragma once

#include <scene/dynamic/DynObject.h>

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>

namespace visualhelios{

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
     * @brief Polygon mesh representing the dynamic object for visualization
     *  purposes
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr polymesh;
    /**
     * @brief Dynamic object vertices connection specification through ordered
     *  indices for visualization purposes.
     *
     * It is, how the vertices must be connected to render each primitive in
     *  the polygon mesh
     */
    vector<pcl::Vertices> vertices;
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

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Method to adapt dynamic object computations over time to visual
     *  Helios
     */
    void doStep();

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
     * @brief Obtain the polygon mesh representing the dynamic object
     * @return Polygon mesh representing the dynamic object
     */
    inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPolymesh() const
    {return polymesh;}
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
};

}

#endif