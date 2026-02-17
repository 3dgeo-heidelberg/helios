#ifdef PCL_BINDING

#include <helios/visualhelios/VHRaycastingCanvas.h>

#include <pcl/impl/point_types.hpp>

using namespace visualhelios;

using std::make_shared;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHRaycastingCanvas::VHRaycastingCanvas(
  DynScene& ds,
  Scanner& sc,
  Survey& sv,
  string const title,
  bool const normalsKeyboardCallbackEnabled,
  bool const normalsUsageTextEnabled,
  bool const renderingNormals,
  float const normalMagnitude)
  : VHSceneCanvas(ds,
                  title,
                  normalsKeyboardCallbackEnabled,
                  normalsUsageTextEnabled,
                  renderingNormals,
                  normalMagnitude)
{
  scanner = make_shared<VHScannerAdapter>(sc, sv);
}

// ***  CANVAS METHODS  *** //
// ************************ //
void
VHRaycastingCanvas::configure()
{
  // Configure base canvas
  VHSceneCanvas::configure();

  // Configure the scanner adapter
  scanner->setOriginRadius(5.0);
  scanner->setOriginColor(0.6, 0.1, 0.1);
  scanner->setRayLength(500.0);
  scanner->setRayColor(0.9, 0.1, 0.15);
  scanner->setNonReturningRayColor(0.3, 0.2, 0.9);
}
void
VHRaycastingCanvas::start()
{
  // Start base canvas
  VHSceneCanvas::start();

  // Start the scanner adapter
  scanner->start();
}
void
VHRaycastingCanvas::update()
{
  // Base canvas update
  VHSceneCanvas::update();

  // Update the scanner adapter
  glm::dvec3 const& p = scanner->getRayOrigin();
  glm::dvec3 const& v = scanner->getRayDir();
  glm::dvec3 const& q = p + scanner->getRayLength() * v;
  double const* rayColor = scanner->getCurrentRayColor();
  if (!rayRendered) { // Initial rendering
    viewer->addSphere(pcl::PointXYZ(p.x, p.y, p.z),
                      scanner->getOriginRadius(),
                      scanner->getOriginColorRed(),
                      scanner->getOriginColorGreen(),
                      scanner->getOriginColorBlue(),
                      rayOriginId);
    viewer->addLine(pcl::PointXYZ(p.x, p.y, p.z),
                    pcl::PointXYZ(q.x, q.y, q.z),
                    rayColor[0],
                    rayColor[1],
                    rayColor[2],
                    rayId);
    rayRendered = true;
  } else { // Non-initial rendering
    viewer->updateSphere(pcl::PointXYZ(p.x, p.y, p.z),
                         scanner->getOriginRadius(),
                         scanner->getOriginColorRed(),
                         scanner->getOriginColorGreen(),
                         scanner->getOriginColorBlue(),
                         rayOriginId);
    viewer->removeShape(rayId);
    viewer->addLine(pcl::PointXYZ(p.x, p.y, p.z),
                    pcl::PointXYZ(q.x, q.y, q.z),
                    rayColor[0],
                    rayColor[1],
                    rayColor[2],
                    rayId);
  }
  scanner->nextStep();
}

#endif
