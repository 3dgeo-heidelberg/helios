#ifdef PCL_BINDING

#include <VHStaticObjectAdapter.h>
#include <Primitive.h>

using visualhelios::VHStaticObjectAdapter;

// ***  BUILDING  *** //
// ****************** //
void VHStaticObjectAdapter::buildPolymesh(){
    // Instantiate a new polymesh replacing the old one, if any
    constructPolymesh();
    vertices.clear();

    // Add each primitive to the polymesh
    int offset = 0;  // To handle vertex indices
    vector<Primitive *> const &primitives = staticObj.mPrimitives;
    for(Primitive * primitive : primitives){ // For each primitive in object
        pcl::Vertices verts;  // Vertex connection order for the primitive
        int const n = (int) primitive->getNumVertices();
        Vertex *primitiveVerts = primitive->getVertices();
        for(int i = 0 ; i < n ; ++i){
            vertexToMesh(primitiveVerts[i]);
            verts.vertices.push_back(offset+i);  // Register vertex order
        }
        vertices.push_back(verts);  // Register primitive order of vertices
        offset += n;  // Update vertex start index for next primitive
    }
}

#endif
