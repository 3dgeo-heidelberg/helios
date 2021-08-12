#ifdef PCL_BINDING

#include <visualhelios/VHSimpleCanvas.h>

using visualhelios::VHSimpleCanvas;


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHSimpleCanvas::VHSimpleCanvas(string const title) : VHCanvas(title) {}

// ***  CANVAS METHODS  *** //
// ************************ //
void VHSimpleCanvas::start(){
    // Build polygon meshes and add them to viewer
    for(shared_ptr<VHDynObjectAdapter> & dynObj : dynObjs){
        dynObj->buildPolymesh();
        viewer->addPolygonMesh<pcl::PointXYZ>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
    }

    // Apply initial transformations
    for(shared_ptr<VHDynObjectAdapter> & dynObj : dynObjs){
        dynObj->doStep();
        viewer->updatePolygonMesh<pcl::PointXYZ>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
    }
}

void VHSimpleCanvas::update(){
    if(dynamicUpdateFunction) dynamicUpdateFunction(dynObjs);
    for(shared_ptr<VHDynObjectAdapter> & dynObj : dynObjs){
        dynObj->doStep();
        viewer->updatePolygonMesh<pcl::PointXYZ>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
    }
}

#endif