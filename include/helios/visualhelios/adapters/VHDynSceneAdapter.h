#ifdef PCL_BINDING
#ifndef _VHDYNSCENEADAPTER_H_

#include <helios/scene/dynamic/DynScene.h>
#include <helios/visualhelios/adapters/VHDynObjectAdapter.h>
#include <helios/visualhelios/adapters/VHStaticObjectAdapter.h>

#include <memory>
#include <vector>

namespace visualhelios {

using std::shared_ptr;
using std::static_pointer_cast;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam ST Type of static object adapter to be used by the dynamic scene
 *  adapter
 * @tparam DT Type of dynamic object adapter to be used by the dynamic scene
 *  adapter
 * @brief Class defining core mechanisms to adapt dynamic scenes to the visual
 *  Helios context based on PCL and VTK libraries
 *
 * @see visualhelios::VHDynObjectAdapter
 */
template<typename ST, typename DT>
class VHDynSceneAdapter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The adapted dynamic scene
   * @see DynScene
   */
  DynScene& dynScene;
  /**
   * @brief Vector of adapted dynamic objects from the adapted dynamic scene
   * @see visualhelios::VHDynObjectAdapter
   */
  vector<shared_ptr<DT>> dynObjs;
  /**
   * @brief Vector of adapted static objects from the adapted dynamic scene
   *
   * @see visualhelios::VHStaticObjectAdapter
   */
  vector<shared_ptr<ST>> staticObjs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios dynamic scene adapter
   * @param dynScene Dynamic scene to be adapted for visual Helios
   */
  VHDynSceneAdapter(DynScene& dynScene);
  virtual ~VHDynSceneAdapter() = default;

  // ***  DYNAMIC BEHAVIOR  *** //
  // ************************** //
  /**
   * @brief Method to adapt dynamic scene computations over time to visual
   *  Helios
   *
   * @return True if the dynamic scene was modified, false otherwise
   */
  bool doStep();

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the adapted dynamic scene
   *
   * <b><span style="color: red;">WARNING</span></b> this getter returns the
   *  dynamic scene reference allowing modifications. Use with caution.
   *
   * @return Adapted dynamic scene
   */
  inline DynScene& getDynScene() { return dynScene; }
  /**
   * @brief Obtain the number of static objects composing the dynamic scene
   * @return Number of static objects composing the dynamic scene
   */
  inline size_t numStaticObjects() const { return staticObjs.size(); }
  /**
   * @brief Obtain the adapted static object at given index
   * @param index Index of the static object to be obtained
   * @return Adapted static object at given index
   * @see visualhelios::VHStaticObjectAdapter
   */
  inline shared_ptr<ST> getAdaptedStaticObj(size_t const index)
  {
    return staticObjs[index];
  }
  /**
   * @brief Obtain the static object at given index
   * @param index Index of static object to be obtained
   * @return Static object at given index
   * @see visualhelios::VHStaticObjectAdapter::getStaticObj
   * @see ScenePart
   */
  inline ScenePart& getStaticObj(size_t const index)
  {
    return static_pointer_cast<VHStaticObjectAdapter>(staticObjs[index])
      ->getStaticObj();
  }
  /**
   * @brief Check whether the normals of static object at given index must
   *  be rendered or not
   * @see visualhelios::VHStaticObjectAdapter::isRenderingNormals
   */
  inline bool isStaticObjectRenderingNormals(size_t const index) const
  {
    return static_pointer_cast<VHStaticObjectAdapter>(staticObjs[index])
      ->isRenderingNormals();
  }
  /**
   * @brief Enable or disable normals rendering for the static object at
   *  given index
   * @see VHStaticObjectAdapter::setRenderingNormals
   */
  inline void setStaticObjectRenderingNormals(size_t const index,
                                              bool const renderingNormals)
  {
    static_pointer_cast<VHStaticObjectAdapter>(staticObjs[index])
      ->setRenderingNormals(renderingNormals);
  }
  /**
   * @brief Obtain the number of dynamic objects composing the dynamic scene
   * @return Number of dynamic objects composing the dynamic scene
   */
  inline size_t numDynObjects() const { return dynObjs.size(); }
  /**
   * @brief Obtain the adapted dynamic object at given index
   * @param index Index of the dynamic object to be obtained
   * @return Adapted dynamic object at given index
   * @see visualhelios::VHDynObjectAdapter
   */
  inline shared_ptr<DT> getAdaptedDynObj(size_t const index)
  {
    return dynObjs[index];
  }
  /**
   * @brief Obtain the dynamic object at given index
   * @param index Index of dynamic object to be obtained
   * @return Dynanic object at given index
   * @see visualhelios::VHDynObjectAdapter::getDynObj
   * @see DynObject
   */
  inline DynObject& getDynObj(size_t const index)
  {
    return static_pointer_cast<VHDynObjectAdapter>(dynObjs[index])->getDynObj();
  }
  /**
   * @brief Check if the dynamic object at given index has been updated on
   *  last step (true) or not (false)
   * @param index Index of dynamic object to be checked
   * @return True if dynamic object at given index has been udpated on last
   *  step, false otherwise
   * @see DynScene::isUpdated
   */
  inline bool isDynObjectUpdated(size_t const index) const
  {
    return dynScene.isDynObjectUpdated(index);
  }
  /**
   * @brief Obtain the ordered vertices indices representing the dynamic
   *  object at given index
   * @see visualhelios::VHDynObjectAdapter::getVertices
   */
  /**
   * @brief Check whether the normals of dynamic object at given index must
   *  be rendered or not
   * @see visualhelios::VHDynObjectAdapter::isRenderingNormals
   */
  inline bool isDynObjectRenderingNormals(size_t const index) const
  {
    return static_pointer_cast<VHDynObjectAdapter>(dynObjs[index])
      ->isRenderingNormals();
  }
  /**
   * @brief Enable or disable normals rendering for the dynamic object at
   *  given index
   * @see VHDynObjectAdapter::setRenderingNormals
   */
  inline void setDynObjectRenderingNormals(size_t const index,
                                           bool const renderingNormals)
  {
    static_pointer_cast<VHDynObjectAdapter>(dynObjs[index])
      ->setRenderingNormals(renderingNormals);
  }
  inline vector<pcl::Vertices> const& getVertices(size_t const index) const
  {
    return static_pointer_cast<VHDynObjectAdapter>(dynObjs[index])
      ->getVertices();
  }
  /**
   * @brief Obtain the ID of the dynamic object at given index
   * @see visualhelios::VHDynObjectAdapter::getId
   */
  inline string const& getId(size_t const index) const
  {
    return static_pointer_cast<VHDynObjectAdapter>(dynObjs[index])->getId();
  }
};

#include <helios/visualhelios/adapters/VHDynSceneAdapter.tpp>
}
#define _VHDYNSCENEADAPTER_H_
#endif
#endif
