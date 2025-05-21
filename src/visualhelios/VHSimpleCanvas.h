#ifdef PCL_BINDING

#include <memory>
#include <vector>

#include <visualhelios/VHNormalsCanvas.h>
#include <visualhelios/adapters/VHDynObjectXYZRGBAdapter.h>

namespace visualhelios {

using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Simple Canvas is a class which supports rendering
 *  polygon meshes which are updated over time
 */
class VHSimpleCanvas : public VHNormalsCanvas
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The dynamic objects that must be rendered by the simple canvas
   * @see VHSimpleCanvas::appendDynObj
   * @see VHSimpleCanvas::getDynObj
   * @see VHSimpleCanvas::setDynObj
   * @see VHSimpleCanvas::clearDynObjs
   */
  vector<shared_ptr<VHDynObjectXYZRGBAdapter>> dynObjs;
  /**
   * @brief Function to define dynamic objects behavior before updating the
   *  canvas
   *
   * It can be used to populate the motion queues or any other required
   *  manipulation over dynamic objects.
   *
   * Simple canvas does not rely on components which update the dynamic
   *  object outside the canvas scope, so any implementation to define the
   *  behavior of dynamic objects must be done through this ad-hoc function
   */
  std::function<void(vector<shared_ptr<VHDynObjectXYZRGBAdapter>>)>
    dynamicUpdateFunction;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the visual Helios simple canvas
   * @see visualhelios::VHNormalsCanvas::VHNormalsCanvas
   */
  VHSimpleCanvas()
    : VHSimpleCanvas("Visual Helios simple canvas")
  {
  }
  /**
   * @brief Constructor for the visual Helios simple canvas which allows for
   *  title specification
   * @param title Title for the visualizer
   * @see visualhelios::VHNormalCanvas::VHNormalCanvas(string const)
   */
  VHSimpleCanvas(string const title);
  virtual ~VHSimpleCanvas() = default;

protected:
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

public:
  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Append a dynamic object to the simple canvas
   * @param dynObj Dynamic object to be appended
   * @see VHSimpleCanvas::dynObjs
   */
  inline void appendDynObj(shared_ptr<VHDynObjectXYZRGBAdapter> dynObj)
  {
    dynObjs.push_back(dynObj);
  }
  /**
   * @brief Obtain a dynamic object from simple canvas
   * @param index Index of dynamic object to be obtained
   * @return Dynamic object from simple canvas
   */
  inline VHDynObjectAdapter const& getDynObj(size_t index) const
  {
    return *dynObjs[index];
  }
  /**
   * @brief Replace a dynamic object in simple canvas
   * @param index Index of dynamic object to be replaced
   * @param dynObj Dynamic object to replace with
   */
  inline void setDynObj(size_t index,
                        shared_ptr<VHDynObjectXYZRGBAdapter> dynObj)
  {
    dynObjs[index] = dynObj;
  }
  /**
   * @brief Remove all dynamic objects from simple canvas
   */
  inline void clearDynObjs() { dynObjs.clear(); }
  /**
   * @brief Set the dynamic update function
   * @param dynamicUpdateFunction  The dynamic update function
   * @see visualhelios::VHSimpleCanvas::dynamicUpdateFunction
   */
  inline void setDynamicUpdateFunction(
    std::function<void(vector<shared_ptr<VHDynObjectXYZRGBAdapter>>)> const
      dynamicUpdateFunction)
  {
    this->dynamicUpdateFunction = dynamicUpdateFunction;
  }
};

}

#endif
