#ifdef PCL_BINDING

#include <visualhelios/VHSceneCanvas.h>
#include <glm/detail/type_vec3.hpp>

using namespace visualhelios;

using std::make_shared;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHSceneCanvas::VHSceneCanvas(
    DynScene &ds,
    string const title,
    bool const normalsKeyboardCallbackEnabled,
    bool const normalsUsageTextEnabled,
    bool const renderingNormals,
    float const normalMagnitude
) :
    VHNormalsCanvas(
        title,
        normalsKeyboardCallbackEnabled,
        normalsUsageTextEnabled,
        renderingNormals,
        normalMagnitude
    )
{
    dynScene = make_shared<VHDynSceneAdapter<VHDynObjectXYZRGBAdapter>>(ds);
}


// ***  CANVAS METHODS  *** //
// ************************ //
void VHSceneCanvas::configure(){
    // Configure base canvas
    VHNormalsCanvas::configure();

    // Configure camera
    glm::dvec3 const centerP = dynScene->getDynScene().getAABB()->getCentroid();
    glm::dvec3 const maxP = dynScene->getDynScene().getAABB()->getMax();
    viewer->setCameraPosition(
        centerP.x, centerP.y, maxP.z+centerP.z,
        0.0, 0.0, -1.0
    );
}

void VHSceneCanvas::start(){
    // Start base canvas
    VHNormalsCanvas::start();

    // Build polygon meshes and add them to viewer
    size_t const n = dynScene->numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        shared_ptr<VHDynObjectXYZRGBAdapter> dynObj =
            dynScene->getAdaptedDynObj(i);
        dynObj->buildPolymesh();
        viewer->addPolygonMesh<pcl::PointXYZRGB>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
        // Render initial normals if requested
        if(isRenderingNormals()) renderNormals(*dynObj);
    }
}

void VHSceneCanvas::update(){
    // Base canvas update
    VHNormalsCanvas::update();

    // Update polygon meshes which need an update
    size_t const n = dynScene->numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        shared_ptr<VHDynObjectXYZRGBAdapter> dynObj =
            dynScene->getAdaptedDynObj(i);
        // Continue to next iteration if no updates are needed for this
        if(dynObj->doStep() && !isNeedingUpdate()) continue;
        // Update the polygon mesh itself
        viewer->updatePolygonMesh<pcl::PointXYZRGB>(
            dynObj->getPolymesh(),
            dynObj->getVertices(),
            dynObj->getId()
        );
        // Update normals if requested
        if(isRenderingNormals()) renderNormals(*dynObj);
    }
}


// ***  NORMALS RENDERING METHODS  ***  //
// ************************************ //
void VHSceneCanvas::renderNormals(VHDynObjectAdapter & dynObj){
    if(!dynObj.isRenderingNormals()) return;
    // TODO Rethink : Implement
}

void VHSceneCanvas::unrenderAllNormals(){
    // TODO Rethink : Implement
}

#endif