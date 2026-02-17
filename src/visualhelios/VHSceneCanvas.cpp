#ifdef PCL_BINDING

#include <helios/maths/MathConstants.h>
#include <helios/visualhelios/VHSceneCanvas.h>

#include <glm/detail/type_vec3.hpp>

using namespace visualhelios;

using std::make_shared;

// ***  CONSTANTS  *** //
// ******************* //
double const VHSceneCanvas::cosPIeighth = std::cos(PI_EIGHTH);
double const VHSceneCanvas::camCoef =
  cosPIeighth / std::sqrt(1.0 - cosPIeighth * cosPIeighth);

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHSceneCanvas::VHSceneCanvas(DynScene& ds,
                             string const title,
                             bool const normalsKeyboardCallbackEnabled,
                             bool const normalsUsageTextEnabled,
                             bool const renderingNormals,
                             float const normalMagnitude)
  : VHNormalsCanvas(title,
                    normalsKeyboardCallbackEnabled,
                    normalsUsageTextEnabled,
                    renderingNormals,
                    normalMagnitude)
{
  dynScene = make_shared<
    VHDynSceneAdapter<VHStaticObjectXYZRGBAdapter, VHDynObjectXYZRGBAdapter>>(
    ds);
}

// ***  CANVAS METHODS  *** //
// ************************ //
void
VHSceneCanvas::configure()
{
  // Configure base canvas
  VHNormalsCanvas::configure();

  // Configure camera
  cameraFromScene();
}

void
VHSceneCanvas::start()
{
  // Start base canvas
  VHNormalsCanvas::start();

  // Build polygon meshes for non dynamic objects and add them to viewer
  size_t const m = dynScene->numStaticObjects();
  for (size_t i = 0; i < m; ++i) {
    addObjectToViewer(*dynScene->getAdaptedStaticObj(i));
  }

  // Build polygon meshes for dynamic objects and add them to viewer
  size_t const n = dynScene->numDynObjects();
  for (size_t i = 0; i < n; ++i) {
    addObjectToViewer(*dynScene->getAdaptedDynObj(i));
  }
}

void
VHSceneCanvas::update()
{
  // Base canvas update
  VHNormalsCanvas::update();

  // Compute next step
  bool sceneUpdated = dynScene->doStep();

  // Handle normals rendering for static objects meshes if must update
  if (isNeedingUpdate()) {
    size_t const m = dynScene->numStaticObjects();
    for (size_t i = 0; i < m; ++i) {
      shared_ptr<VHStaticObjectXYZRGBAdapter> staticObj =
        dynScene->getAdaptedStaticObj(i);
      // Render normals if requested
      if (isRenderingNormals())
        renderNormals(*staticObj);
    }
  }

  // Update polygon meshes for dynamic objects which need an update
  if (sceneUpdated) {
    size_t const n = dynScene->numDynObjects();
    for (size_t i = 0; i < n; ++i) {
      shared_ptr<VHDynObjectXYZRGBAdapter> dynObj =
        dynScene->getAdaptedDynObj(i);
      // Continue to next iteration if no updates are needed for this
      if (!dynScene->isDynObjectUpdated(i) && !isNeedingUpdate())
        continue;
      // Update the polygon mesh itself
      viewer->updatePolygonMesh<pcl::PointXYZRGB>(
        dynObj->getPolymesh(), dynObj->getVertices(), dynObj->getId());
      // Update normals if requested
      if (isRenderingNormals())
        renderNormals(*dynObj);
    }
  }
}

// ***  NORMALS RENDERING METHODS  ***  //
// ************************************ //
void
VHSceneCanvas::renderNormals(VHStaticObjectAdapter& staticObj)
{
  // Check normals must be rendered
  if (!staticObj.isRenderingNormals())
    return;

  // Build id
  std::stringstream ss;
  ss << "normalsCloud_" << staticObj.getId();
  string id = ss.str();

  // Build pcpn (Point Cloud with Position and Normals)
  pcl::PointCloud<pcl::PointNormal>::Ptr pcpn(
    new pcl::PointCloud<pcl::PointNormal>());
  vector<Vertex*> vertices = staticObj.getStaticObj().getAllVertices();
  for (Vertex* vertex : vertices) {
    pcpn->push_back(pcl::PointNormal(vertex->pos.x,
                                     vertex->pos.y,
                                     vertex->pos.z,
                                     vertex->normal.x,
                                     vertex->normal.y,
                                     vertex->normal.z));
  }

  // Remove previous pcpn from viewer
  viewer->removePointCloud(id);

  // Add pcpn to viewer
  viewer->addPointCloudNormals<pcl::PointNormal>(pcpn, 1, normalMagnitude, id);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    normalDefColor[0],
    normalDefColor[1],
    normalDefColor[2],
    id);
}

void
VHSceneCanvas::unrenderAllNormals()
{
  // Remove normals for static objects
  unrenderNormals(dynScene->numStaticObjects(),
                  [&](size_t const index) -> ScenePart& {
                    return dynScene->getStaticObj(index);
                  });

  // Remove normals for dynamic objects
  unrenderNormals(dynScene->numDynObjects(),
                  [&](size_t const index) -> ScenePart& {
                    return dynScene->getDynObj(index);
                  });
}

// ***   U T I L S   *** //
// ********************* //
void
VHSceneCanvas::cameraFromScene()
{
  // Configure camera
  double const sceneMaxZ = dynScene->getDynScene().getAABB()->getMax().z;
  glm::dvec3 const c = dynScene->getDynScene().getAABB()->getCentroid();
  glm::dvec3 const p = dynScene->getDynScene().getAABB()->getMin();
  double const camZ =
    std::max((p.z - camCoef * (p.x - 2 * c.x)) / 2.0, sceneMaxZ);
  viewer->setCameraPosition(c.x, c.y, camZ, c.x, c.y, p.z, 0.0, 1.0, 0.0);
}

void
VHSceneCanvas::unrenderNormals(size_t const m,
                               std::function<ScenePart&(size_t const)> get)
{
  for (size_t i = 0; i < m; ++i) {
    // Build id
    std::stringstream ss;
    ss << "normalsCloud_" << get(i).getId();
    // Remove normals cloud from viewer
    viewer->removePointCloud(ss.str());
  }
}

void
VHSceneCanvas::addObjectToViewer(VHStaticObjectXYZRGBAdapter& obj)
{
  // Add object to viewer
  obj.buildPolymesh();
  viewer->addPolygonMesh<pcl::PointXYZRGB>(
    obj.getPolymesh(), obj.getVertices(), obj.getId());

  // Render initial normals if requested
  if (isRenderingNormals())
    renderNormals(obj);
}

#endif
