#ifdef PCL_BINDING
#pragma once

#include <demo/BaseDemo.h>
#include <scene/primitives/Triangle.h>

#include <vector>
#include <memory>
#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>

namespace HeliosDemos{

using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Simple primitives demo
 *
 * This demo implements simple objects
 */
class SimplePrimitivesDemo : public BaseDemo{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple primitives demo constructor
     */
    SimplePrimitivesDemo() : BaseDemo("Simple primitives demo"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseDemo::run
     */
    void run() override;

    // ***  U T I L  *** //
    // ***************** //
    /**
     * @brief Build a polygon mesh from given set of helios triangle primitives
     * @param[in] triangles Triangle primitives
     * @param[out] vertices Where the indices of vertices defining the mesh
     *  will be stored
     * @return Built point cloud from given triangles
     * @see Triangle
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr buildPolymesh(
        vector<shared_ptr<Triangle>> const triangles,
        std::vector<pcl::Vertices> &vertices
    );
    /**
     * @brief Build the triangles representing the mobile structure
     * @return Triangles representing the mobile structure
     */
    vector<shared_ptr<Triangle>> buildMobileStructure();
    /**
     * @brief Build the triangles representing the fixed structure
     * @return Triangles representing the fixed structure
     */
    vector<shared_ptr<Triangle>> buildFixedStructure();
    /**
     * @brief Build the triangles representing the static structure
     * @return Triangles representing the static structure
     */
    vector<shared_ptr<Triangle>> buildStaticStructure();
};


}
#endif