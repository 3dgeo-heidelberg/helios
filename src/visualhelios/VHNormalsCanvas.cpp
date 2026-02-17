#ifdef PCL_BINDING

#include <helios/visualhelios/VHNormalsCanvas.h>

using namespace visualhelios;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHNormalsCanvas::VHNormalsCanvas(string const title,
                                 bool const normalsKeyboardCallbackEnabled,
                                 bool const normalsUsageTextEnabled,
                                 bool const renderingNormals,
                                 float const normalMagnitude)
  : VHDynCanvas(title)
  , normalsKeyboardCallbackEnabled(normalsKeyboardCallbackEnabled)
  , normalsUsageTextEnabled(normalsUsageTextEnabled)
  , renderingNormals(renderingNormals)
  , normalMagnitude(normalMagnitude)
  , normalDefColor{ 1.0, 1.0, 0 }
{
}

// ***  CANVAS METHODS  *** //
// ************************ //
void
VHNormalsCanvas::configure()
{
  // Configure base canvas
  VHDynCanvas::configure();

  // Register keyboard callbacks for simple canvas
  registerNormalsKeyboardCallback();
}

void
VHNormalsCanvas::start()
{
  // Start base canvas
  VHDynCanvas::start();

  // Add keyboard usage text
  addKeyboardCallbackUsageText();
}

// ***  NORMALS RENDERING METHODS  *** //
// *********************************** //
void
VHNormalsCanvas::registerNormalsKeyboardCallback()
{
  // Check whether normals toggling keyboard callback must be enabled or not
  if (!normalsKeyboardCallbackEnabled)
    return;

  // Register keyboard callbacks for simple canvas
  viewer->registerKeyboardCallback(
    [&](pcl::visualization::KeyboardEvent const& ev) -> void {
      if (!ev.keyDown())
        return; // Consider only key down cases

      // Handle normal toggling by key
      if (ev.getKeyCode() == 'n' || ev.getKeyCode() == 'N') {
        this->setRenderingNormals(!this->isRenderingNormals());
        if (this->isRenderingNormals())
          setNeedsUpdate(true);
        else
          this->unrenderAllNormals();
        std::cout << "Rendering normals modified by keyboard: "
                  << this->isRenderingNormals() << std::endl;
      }
    });
}

void
VHNormalsCanvas::addKeyboardCallbackUsageText()
{
  // Check whtether keyboard usage text must be added or not
  if (!normalsUsageTextEnabled)
    return;

  // Add keyboard usage text
  viewer->addText(
    "Press N to enable/disable normals rendering", 10, 20, "keyboardUsageText");
}

#endif
