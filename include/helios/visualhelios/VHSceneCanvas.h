#ifdef PCL_BINDING

#pragma once

#include <helios/visualhelios/VHNormalsCanvas.h>
#include <helios/visualhelios/adapters/VHDynObjectXYZRGBAdapter.h>
#include <helios/visualhelios/adapters/VHDynSceneAdapter.h>

#include <memory>

namespace visualhelios {

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Scene Canvas is a class which supports rendering
 *  a dynamic scene
 */
class VHSceneCanvas : public VHNormalsCanvas
{
protected:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * \f[
   *  \cos\left(\frac{\pi}{8}\right)
   * \f]
   */
  static double const cosPIeighth;
  /**
   * \f[
   *  \frac{\cos\left(\frac{\pi}{8}\right)}
   *  {\sqrt{1 - \cos\left(\frac{\pi}{8}\right)^2}}
   * \f]
   */
  static double const camCoef;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The dynamic scene that must be rendered
   */
  shared_ptr<
    VHDynSceneAdapter<VHStaticObjectXYZRGBAdapter, VHDynObjectXYZRGBAdapter>>
    dynScene;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the visual Helios scene canvas
   * @see visualhelios::VHNormalsCanvas::VHNormalsCanvas
   */
  VHSceneCanvas(DynScene& ds)
    : VHSceneCanvas(ds, "Visual Helios scene canvas")
  {
  }
  /**
   * @brief Constructor for the visual Helios scene canvas which allows for
   *  title specification, flags configuration and normal magnitude
   *  specification
   * @see VHNormalsCanvas(string const, bool const, bool const, bool const,
   * float const)
   */
  VHSceneCanvas(DynScene& ds,
                string const title,
                bool const normalsKeyboardCallbackEnabled = true,
                bool const normalsUsageTextEnabled = true,
                bool const renderingNormals = true,
                float const normalMagnitude = 0.2);
  virtual ~VHSceneCanvas() = default;

  // ***  CANVAS METHODS  *** //
  // ************************ //
  /**
   * @see VHNormalsCanvas::configure
   */
  void configure() override;
  /**
   * @see VHNormalsCanvas::start
   */
  void start() override;
  /**
   * @see VHNormalsCanvas::update
   */
  void update() override;

  // ***  NORMALS RENDERING METHODS  ***  //
  // ************************************ //
  /**
   * @brief Render normals for each primitive of given static object
   * @see visualhelios::VHNormalsCanvas::renderNormals
   */
  void renderNormals(VHStaticObjectAdapter& staticObj) override;
  /**
   * @brief Remove all rendered normals
   * @see visualhelios::VHNormalsCanvas::unrenderAllNormals
   */
  void unrenderAllNormals() override;

protected:
  // ***   U T I L S   *** //
  // ********************* //
  /**
   * @brief Set the camera position from the scene.
   *
   * For this purpose let \f$O\f$ be the point where the camera is located,
   *  \f$P\f$ be the point with minimum coordinates from the scene and
   *  \f$C\f$ be the centroid of the scene. Now lets define the normalized
   *  director vector \f$\hat{v}\f$ that will be used to define the
   *  \f$\overrightarrow{OP}\f$ direction with a \f$\frac{\pi}{8}\f$ angle
   *  which is half of \f$\frac{\pi}{4}\f$ angle that is the approximated
   *  angle corresponding to camera field of view:
   *
   * \f[
   *  \hat{v} = \left(
   *      \sqrt{1-\cos\left(\frac{\pi}{8}\right)^2},
   *      0,
   *      \cos\left(\frac{\pi}{8}\right)
   *  \right)
   * \f]
   *
   * For the point \f$O = \left(O_x, O_y, O_z\right)\f$ its \f$O_x\f$ and
   *  \f$O_y\f$ components are known, but its \f$O_z\f$ component must be
   *  calculated from expression \f$\overrightarrow{OP} = O + t\hat{v}\f$
   *  as the required \f$z\f$-distance so the camera is guaranteed to view
   *  all the scene from above. Thus, following system must be solved:
   *
   * \f[
   *  \left\{\begin{array}{lll}
   *      P_x - O_x &=& O_x + t \hat{v}_x \\
   *      P_z - O_z &=& O_z + t \hat{v}_z
   *  \end{array}\right.
   * \f]
   *
   * It is easy to see that:
   *
   * \f[
   *  t = \frac{P_x - 2 O_x}{\hat{v}_x}
   * \f]
   *
   * In consequence:
   *
   * \f[
   *  O_z =
   *      \frac{P_z - t \hat{v}_z}{2} =
   *      \frac{1}{2}\left[
   *          P_z -
   *          \frac{\cos\left(\frac{\pi}{8}\right)}
   *              {\sqrt{1-\cos\left(\frac{\pi}{8}\right)^2}}
   *          \left(P_x - 2 O_x\right)
   *      \right]
   * \f]
   *
   * Finally, one last consideration must be taken into account. If \f$z^*\f$
   *  is the maximum value of the \f$z\f$ coordinate in the scene, then
   *  when \f$O_z < z^*\f$ the \f$z\f$ coordinate for the initial
   *  camera position will be \f$z^*\f$ instead of \f$O_z\f$. It is, the
   *  camera position will be \f$O = \left(O_x, O_y, z^*\right)\f$
   *
   */
  void cameraFromScene();
  /**
   * @brief Unrender normals for objects obtained through given get function
   *  ranging in \f$[0, m)\f$
   * @param m How many objects can be obtained
   * @param get Getter to obtain \f$i\f$-th object
   */
  void unrenderNormals(size_t const m,
                       std::function<ScenePart&(size_t const)> get);
  /**
   * @brief Add given object to the viewer and render its normals if
   *  scene canvas is requested to render normals
   * @param obj The object, whether dynamic or static, to be added to the
   *  viewer
   */
  void addObjectToViewer(VHStaticObjectXYZRGBAdapter& obj);
};

}

#endif
