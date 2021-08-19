#ifdef PCL_BINDING

#include <visualhelios/VHSimpleCanvas.h>

#include <sstream>

using visualhelios::VHSimpleCanvas;


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHSimpleCanvas::VHSimpleCanvas(string const title) :
    VHCanvas(title),
    renderingNormals(false),
    normalMagnitude(2.0f)
{}

// ***  CANVAS METHODS  *** //
// ************************ //
void VHSimpleCanvas::configure(){
    // Configure base canvas
    VHCanvas::configure();

    // Configure camera
    viewer->setCameraPosition(0, -75.0, 35.0, 0.0, 1.0, 0.0);

    // Register keyboard callbacks for simple canvas
    viewer->registerKeyboardCallback(
        [&](pcl::visualization::KeyboardEvent const &ev) -> void {
            if(!ev.keyDown()) return; // Consider only key down cases

            // Handle normal toggling by key
            if(ev.getKeyCode() == 'n' || ev.getKeyCode() == 'N'){
                this->setRenderingNormals(!this->isRenderingNormals());
                if(this->isRenderingNormals()) needUpdate = true;
                else this->unrenderAllNormals();
                std::cout << "Rendering normals modified by keyboard: "
                    << this->isRenderingNormals() << std::endl;
            }
        }
    );
}

void VHSimpleCanvas::start(){
    // Build polygon meshes and add them to viewer
    for(shared_ptr<VHDynObjectXYZRGBAdapter> & dynObj : dynObjs){
        dynObj->buildPolymesh();
        viewer->addPolygonMesh<pcl::PointXYZRGB>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
    }

    // Apply initial transformations
    for(shared_ptr<VHDynObjectXYZRGBAdapter> & dynObj : dynObjs){
        dynObj->doStep();
        viewer->updatePolygonMesh<pcl::PointXYZRGB>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
        // Render initial normals if requested
        if(renderingNormals) renderNormals(*dynObj);
    }

    // Add keyboard usage text
    viewer->addText(
        "Press N to enable/disable normals rendering",
        10, 20,
        "keyboardUsageText"
    );

}

void VHSimpleCanvas::update(){
    // Apply dynamic update function to primitives
    if(dynamicUpdateFunction) dynamicUpdateFunction(dynObjs);

    // Update the polygon mesh after applying dynamic function
    for(shared_ptr<VHDynObjectXYZRGBAdapter> & dynObj : dynObjs){
        // Continue to next iteration if no updates are needed for this
        if(!dynObj->doStep() && !needUpdate) continue;
        // Update the polygon mesh itself
        viewer->updatePolygonMesh<pcl::PointXYZRGB>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
        // Update normals if requested
        if(renderingNormals) renderNormals(*dynObj);
    }

    // After update no updates are needed from canvas side until modified
    needUpdate = false;
}

// ***  UTIL METHODS  ***  //
// ********************** //
void VHSimpleCanvas::renderNormals(VHDynObjectAdapter & dynObj){
    if(!dynObj.isRenderingNormals()) return;
    vector<Primitive *> const primitives = dynObj.getDynObj().getPrimitives();
    for(size_t i = 0 ; i < primitives.size() ; ++i){
        Primitive * primitive = primitives[i];
        float nx = 0;
        float ny = 0;
        float nz = 0;
        pcl::PointXYZ p, q;
        Vertex * vertices = primitive->getVertices();
        float const numVerticesf = (float) primitive->getNumVertices();
        for(size_t j = 0 ; j < primitive->getNumVertices() ; ++j){
            Vertex &vertex = vertices[j];
            nx += vertex.normal.x;
            ny += vertex.normal.y;
            nz += vertex.normal.z;
            p.x += vertex.pos.x;
            p.y += vertex.pos.y;
            p.z += vertex.pos.z;
        }
        nx /= numVerticesf;
        ny /= numVerticesf;
        nz /= numVerticesf;
        p.x /= numVerticesf;
        p.y /= numVerticesf;
        p.z /= numVerticesf;
        q.x = p.x + normalMagnitude * nx;
        q.y = p.y + normalMagnitude * ny;
        q.z = p.z + normalMagnitude * nz;
        std::stringstream ss;
        ss << dynObj.getId() << "_normal" << i;
        viewer->removeShape(ss.str());
        viewer->addLine(p, q, 1.0, 1.0, 0.0, ss.str());
    }
}

void VHSimpleCanvas::unrenderAllNormals(){
    for(shared_ptr<VHDynObjectXYZRGBAdapter> dynObj : dynObjs){
        if(!dynObj->isRenderingNormals()) continue; // Skip, nothing to remove
        vector<Primitive *> const primitives = \
            dynObj->getDynObj().getPrimitives();
        for(size_t i = 0 ; i < primitives.size() ; ++i){
            std::stringstream ss;
            ss << dynObj->getId() << "_normal" << i;
            viewer->removeShape(ss.str());
        }
    }
}

#endif