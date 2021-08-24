#ifdef PCL_BINDING

#include <visualhelios/VHSceneCanvas.h>
#include <MathConstants.h>

#include <glm/detail/type_vec3.hpp>


using namespace visualhelios;

using std::make_shared;

// ***  CONSTANTS  *** //
// ******************* //
double const VHSceneCanvas::cosPIeighth = std::cos(PI_EIGHTH);
double const VHSceneCanvas::camCoef = cosPIeighth/
    std::sqrt(1.0-cosPIeighth*cosPIeighth);

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
    dynScene = make_shared<VHDynSceneAdapter<
        VHStaticObjectXYZRGBAdapter, VHDynObjectXYZRGBAdapter
    >>(ds);
}


// ***  CANVAS METHODS  *** //
// ************************ //
void VHSceneCanvas::configure(){
    // Configure base canvas
    VHNormalsCanvas::configure();

    // Configure camera
    cameraFromScene();
}

void VHSceneCanvas::start(){
    // Start base canvas
    VHNormalsCanvas::start();

    // TODO Rethink : Externalize both objects to viewer regions to method

    // Build polygon meshes for non dynamic objects and add them to viewer
    size_t const m = dynScene->numStaticObjects();
    for(size_t i = 0 ; i < m ; ++i){
        shared_ptr<VHStaticObjectXYZRGBAdapter> staticObj =
            dynScene->getAdaptedStaticObj(i);
        staticObj->buildPolymesh();
        viewer->addPolygonMesh<pcl::PointXYZRGB>(
            staticObj->getPolymesh(),
            staticObj->getVertices(),
            staticObj->getId()
        );
        // Render initial normals if requested
        if(isRenderingNormals()) renderNormals(*staticObj);
    }

    // Build polygon meshes for dynamic objects and add them to viewer
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

    // Compute next step
    dynScene->doStep();

    // Update polygon meshes for dynamic objects which need an update
    size_t const n = dynScene->numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        shared_ptr<VHDynObjectXYZRGBAdapter> dynObj =
            dynScene->getAdaptedDynObj(i);
        // Continue to next iteration if no updates are needed for this
        if(!dynScene->isDynObjectUpdated(i) && !isNeedingUpdate()) continue;
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
void VHSceneCanvas::renderNormals(VHStaticObjectAdapter & staticObj){
    if(!staticObj.isRenderingNormals()) return;
    // TODO Rethink : Implement
}

void VHSceneCanvas::unrenderAllNormals(){
    // TODO Rethink : Implement
}


// ***   U T I L S   *** //
// ********************* //
void VHSceneCanvas::cameraFromScene(){
    // Configure camera
    glm::dvec3 const c = dynScene->getDynScene().getAABB()->getCentroid();
    glm::dvec3 const p = dynScene->getDynScene().getAABB()->getMin();
    double const camZ = (p.z-camCoef*(p.x-2*c.x))/2.0;
    viewer->setCameraPosition(
        c.x, c.y, camZ,
        c.x, c.y, p.z,
        0.0, 1.0, 0.0
    );
}
#endif