#ifdef PCL_BINDING

#pragma once

#include <visualhelios/adapters/VHDynObjectAdapter.h>

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
class VHDynObjectXYZAdapter : public VHDynObjectAdapter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Polygon mesh representing the dynamic object in a
     *  \f$\mathbb{R}^{3}\f$ space with no color nor intensity
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr polymesh;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the visual Helios dynamic object adapter
     *  providing XYZ visualization
     * @see visualhelios::VHDynObjectAdapter::VHDynObjectAdapter(DynObject &)
     */
    VHDynObjectXYZAdapter(DynObject &dynObj) : VHDynObjectAdapter(dynObj) {}
    virtual ~VHDynObjectXYZAdapter() = default;

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
     *  in a \f$\mathbb{R}^{3}\f$ space with no color nor intensity
     * @return Polygon mesh representing the dynamic object in a
     *  \f$\mathbb{R}^{3}\f$ space with no color nor intensity
     */
    inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPolymesh() const
    {return polymesh;}
};

}

#endif