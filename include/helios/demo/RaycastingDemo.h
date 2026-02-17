#ifdef PCL_BINDING
#pragma once

#include <helios/demo/DynamicSceneDemo.h>
#include <helios/scanner/Scanner.h>
#include <helios/visualhelios/VHRaycastingCanvas.h>

namespace HeliosDemos {

using visualhelios::VHRaycastingCanvas;

using std::shared_ptr;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Raycasting demo
 *
 * This demo extends the DynamicSceneDemo to also render the rays defining the
 *  scanning process simulation
 *
 * @see HeliosDemos::DynamicSceneDemo
 */
class RaycastingDemo : public DynamicSceneDemo
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The scanner emitting the rays
   */
  shared_ptr<Scanner> scanner = nullptr;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Ray casting demo constructor
   * @see HeliosDemos::DynamicSceneDemo::DynamicSceneDemo
   */
  RaycastingDemo(string const surveyPath, string const assetsPath)
    : DynamicSceneDemo("Raycasting demo", surveyPath, assetsPath)
  {
  }
  virtual ~RaycastingDemo() = default;

  // ***  R U N  *** //
  // *************** //
  /**
   * @see DynamicSceneDemo::run
   */
  void run() override;

  // ***   U T I L S   *** //
  // ********************* //
  /**
   * @brief Build a visual Helios ray casting canvas from given survey.
   *
   * A ray casting canvas is like a scene canvas but extending it to support
   *  the rendering of the scanner's rays. Both, dynamic and static scenes
   *  are supported.
   *
   * @param survey Survey which must contain a valid scene to build the
   *  visual Helios ray casting canvas
   * @return Built visual helios ray casting canvas
   */
  shared_ptr<VHRaycastingCanvas> buildCanvas(shared_ptr<Survey> survey);
};

}

#endif
