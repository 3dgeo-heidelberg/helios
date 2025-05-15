#ifdef PCL_BINDING
#pragma once

#include <visualhelios/VHSceneCanvas.h>
#include <visualhelios/adapters/VHScannerAdapter.h>

#include <memory>

namespace visualhelios {

using std::shared_ptr;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Ray casting Canvas is a class which supports rendering
 *  a dynamic scene together with the rays emitted by the scanner
 */
class VHRaycastingCanvas : public VHSceneCanvas
{
protected:
  using VHSceneCanvas::dynScene;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The scanner which rays must be rendered
   */
  shared_ptr<VHScannerAdapter> scanner;
  /**
   * @brief Flag to control whether there is a rendered ray (true) or not
   *  (false)
   */
  bool rayRendered = false;
  /**
   * @brief The identifier for the rendered shape that represents the origin
   *  of the ray
   */
  string rayOriginId = "SCANNER_RAY_ORIGIN";
  /**
   * @brief The identifier for the rendered shape that represents the ray
   *  itself
   */
  string rayId = "SCANNER_RAY";

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the visual Helios ray casting canvas
   * @param ds The scene to be rendered
   * @param sc The scanner which emitted rays must be rendered
   * @param sv The survey to which the scanner is associated
   */
  VHRaycastingCanvas(DynScene& ds, Scanner& sc, Survey& sv)
    : VHRaycastingCanvas(ds, sc, sv, "Visual Helios ray casting canvas")
  {
  }
  /**
   * @brief Constructor for the visual Helios ray casting canvas which allows
   *  for title specification, flags configuration and normal magnitude
   *  specification
   * @see VHNormalsCanvas(string const, bool const, bool const, bool cont, float
   * const)
   * @see VHRaycastingCanvas::VHRaycastingCanvas(DynScene &, Scanner &)
   */
  VHRaycastingCanvas(DynScene& ds,
                     Scanner& sc,
                     Survey& sv,
                     string const title,
                     bool const normalsKeyboardCallbackEnabled = true,
                     bool const normalsUsageTextEnabled = true,
                     bool const renderingNormals = true,
                     float const normalMagnitude = 0.2);
  virtual ~VHRaycastingCanvas() = default;

  // ***  CANVAS METHODS  *** //
  // ************************ //
  /**
   * @see VHSceneCanvas::configure
   */
  void configure() override;
  /**
   * @see VHSceneCanvas::start
   */
  void start() override;
  /**
   * @see VHSceneCanvas::update
   */
  void update() override;
};
}

#endif
