#pragma once

#include <memory>
#include <vector>

#include <assetloading/ScenePart.h>
#include <scene/Scene.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Static scene basic implementation
 *
 * A static scene is a simple scene which is aware of primitives composing
 *  each object in the scene.
 *
 * @see DynScene
 */
class StaticScene : public Scene
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Static objects composing the scene
   */
  std::vector<std::shared_ptr<ScenePart>> staticObjs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Static scene default constructor
   */
  StaticScene() = default;
  ~StaticScene() override {}
  /**
   * @brief Copy constructor for static scene
   * @param ss Static scene to be copied
   */
  StaticScene(StaticScene& ss);
  /**
   * @brief Build a static scene using given scene as basis
   * @param s Basis scene for static scene
   */
  StaticScene(Scene& s)
    : Scene(s)
  {
  }

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @see Scene::shutdown
   */
  void shutdown() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Append given static object to the scene
   * @param obj Static object to be appended to the scene
   * @see ScenePart
   */
  inline void appendStaticObject(std::shared_ptr<ScenePart> obj)
  {
    staticObjs.push_back(obj);
  }
  /**
   * @brief Obtain static object at given index
   * @param index Index of static object to be obtained
   * @return Static object at given index
   */
  inline std::shared_ptr<ScenePart> getStaticObject(std::size_t const index)
  {
    return staticObjs[index];
  }
  /**
   * @brief Set static object at given index
   * @param index Index of static object to be setted
   * @param obj New static object
   */
  inline void setStaticObject(std::size_t const index,
                              std::shared_ptr<ScenePart> obj)
  {
    staticObjs[index] = obj;
  }
  /**
   * @brief Remove static object at given index
   * @param index Index of static object to be removed
   */
  inline void removeStaticObject(std::size_t const index)
  {
    staticObjs.erase(staticObjs.begin() + index);
  }
  /**
   * @brief Remove all static objects from the static scene
   */
  inline void clearStaticObjects() { staticObjs.clear(); }
  /**
   * @brief Obtain the number of static objects in the scene
   * @return Number of static objects in the scene
   */
  inline size_t numStaticObjects() { return staticObjs.size(); }
};
