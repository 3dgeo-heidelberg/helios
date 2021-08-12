#ifdef PCL_BINDING

#include <VHDynObjectAdapter.h>

using visualhelios::VHDynObjectAdapter;

// ***  BUILDING  *** //
// ****************** //
void VHDynObjectAdapter::buildPolymesh(){
    // Instantiate a new polymesh replacing the old one, if any
    polymesh = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>()
    );
    vertices.clear();

    // Add each primitive to the polymesh
    int offset = 0;  // To handle vertex indices
    vector<Primitive *> const primitives = dynObj.getPrimitives();
    for(Primitive * primitive : primitives){ // For each primitive in dynobj
        pcl::Vertices verts;  // Vertex connection order for the primitive
        int const n = (int) primitive->getNumVertices();
        Vertex *primitiveVerts = primitive->getVertices();
        for(int i = 0 ; i < n ; ++i){
            pcl::PointXYZ p(  // Instantiate the vertex for the polymesh
                (float) primitiveVerts[i].getX(),
                (float) primitiveVerts[i].getY(),
                (float) primitiveVerts[i].getZ()
            );
            polymesh->push_back(p);  // Add vertex to the polymesh
            verts.vertices.push_back(offset+i);  // Register vertex order
        }
        vertices.push_back(verts);  // Register primitive order of vertices
        offset += n;  // Update vertex start index for next primitive
    }
}

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
void VHDynObjectAdapter::doStep(){
    if(dynObj.doStep()) buildPolymesh();
}

#endif