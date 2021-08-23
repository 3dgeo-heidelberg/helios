#ifdef PCL_BINDING

#include <visualhelios/adapters/VHStaticObjectXYZRGBAdapter.h>

using visualhelios::VHStaticObjectXYZRGBAdapter;

// ***  BUILDING  *** //
// ****************** //
void VHStaticObjectXYZRGBAdapter::constructPolymesh(){
    // Instantiate a new polymesh replacing the old one, if any
    polymesh = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>()
    );
}

void VHStaticObjectXYZRGBAdapter::vertexToMesh(Vertex const & vertex){
    polymesh->push_back(pcl::PointXYZRGB(  // Add vertex to the polymesh
        (float) vertex.getX(),
        (float) vertex.getY(),
        (float) vertex.getZ(),
        (uint8_t) (vertex.color.x*255),
        (uint8_t) (vertex.color.y*255),
        (uint8_t) (vertex.color.z*255)
    ));

}

#endif