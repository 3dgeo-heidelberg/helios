#ifdef PCL_BINDING

#include <helios/visualhelios/adapters/VHStaticObjectXYZAdapter.h>

using visualhelios::VHStaticObjectXYZAdapter;

// ***  BUILDING  *** //
// ****************** //
void
VHStaticObjectXYZAdapter::constructPolymesh()
{
  // Instantiate a new polymesh replacing the old one, if any
  polymesh =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

void
VHStaticObjectXYZAdapter::vertexToMesh(Vertex const& vertex)
{
  polymesh->push_back(pcl::PointXYZ( // Add vertex to the polymesh
    (float)vertex.getX(),
    (float)vertex.getY(),
    (float)vertex.getZ()));
}

#endif
