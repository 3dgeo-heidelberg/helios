#ifdef PCL_BINDING

#pragma once

#include <visualhelios/adapters/VHDynObjectAdapter.h>

namespace visualhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing concrete implementation of a VHDynObjectAdapter for
 *  a simple XYZ visualization with RGB color
 */
class VHDynObjectXYZRGBAdapter : public VHDynObjectAdapter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Polygon mesh representing the dynamic object in a
     *  \f$\mathbb{R}^{3}\f$ space with RGB color
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr polymesh;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the visual Helios dynamic object adapter
     *  providing XYZ visualization with RGB color
     * @see visualhelios::VHDynObjectAdapter::VHDynObjectAdapter(DynObject &)
     */
    VHDynObjectXYZRGBAdapter(DynObject &dynObj) : VHDynObjectAdapter(dynObj) {}
    virtual ~VHDynObjectXYZRGBAdapter() = default;

    // ***  BUILDING  *** //
    // ****************** //
    /**
     * @see visualhelios::VHDynObjectAdapter:::buildPolymesh
     */
    void constructPolymesh() override;
    /**
     * @see visualhelios::VHDynObjectAdapter::vertexToMesh
     */
    void vertexToMesh(Vertex const & vertex) override;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the polygon mesh representing the dynamic object
     *  in a \f$\mathbb{R}^{3}\f$ space with RGB color
     * @return Polygon mesh representing the dynamic object in a
     *  \f$\mathbb{R}^{3}\f$ space with RGB color
     */
    inline pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getPolymesh() const
    {return polymesh;}
};

}

#endif