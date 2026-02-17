#pragma once

#include <helios/assetloading/Asset.h>
#include <helios/scene/Scene.h>
#include <helios/scene/StaticScene.h>
#include <helios/scene/dynamic/DynScene.h>

// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif
#include <boost/serialization/export.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for scenes which assists read and write serialization
 *  operations depending on the type of scene (Scene, StaticScene, DynScene)
 *
 * @see Scene
 * @see StaticScene
 * @see DynScene
 */
class SerialSceneWrapper : public Asset
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a SerialSceneWrapper to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the SerialSceneWrapper
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<SerialSceneWrapper, Asset>();
    ar& boost::serialization::base_object<Asset>(*this);
    ar & sceneType;
    switch (sceneType) {
      case SCENE:
        ar & scene;
        break;
      case STATIC_SCENE: {
        StaticScene* _scene = (StaticScene*)scene;
        ar & _scene;
        scene = (Scene*)_scene;
        break;
      }
      case DYNAMIC_SCENE: {
        DynScene* _scene = (DynScene*)scene;
        ar & _scene;
        scene = (Scene*)_scene;
        break;
      }
    }
  }

public:
  // ***  SCENE TYPE ENUMERATION  *** //
  // ******************************** //
  /**
   * @brief Types of scene that can be handled by the SerialSceneWrapper
   */
  enum SceneType
  {
    SCENE,
    STATIC_SCENE,
    DYNAMIC_SCENE
  };

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Type of the scene being wrapped
   */
  SceneType sceneType;
  /**
   * @brief Pointer to the scene being wrapped
   */
  Scene* scene;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for serial scene wrapper
   */
  SerialSceneWrapper()
    : Asset()
    , sceneType(SCENE)
    , scene(nullptr)
  {
  }
  SerialSceneWrapper(SceneType sceneType, Scene* scene)
    : Asset()
    , sceneType(sceneType)
    , scene(scene)
  {
  }
  ~SerialSceneWrapper() override = default;

  // ***  READ / WRITE  *** //
  // ********************** //
  /**
   * @brief Serialize the scene writing it to given output file.
   * @param path Path to output file where serialized scene shall be stored.
   */
  void writeScene(std::string const& path);
  /**
   * @brief Read serialized scene from given file, automatically handling
   *  the scene type to be read
   */
  static SerialSceneWrapper* readScene(std::string const& path);

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the type of scene being wrapped
   * @return Type of scene being wrapped
   * @see SerialSceneWrapper::sceneType
   * @see SerialSceneWrapper::setSceneType
   */
  SceneType getSceneType() const { return sceneType; }
  /**
   * @brief Set the type of scene being wrapped
   * @param sceneType Type of scene to be wrapped
   * @see SerialSceneWrapper::sceneType
   * @see SerialSceneWrapper::getSceneType
   */
  void setSceneType(SceneType const& sceneType) { this->sceneType = sceneType; }
  /**
   * @brief Obtain the scene being wrapped
   * @return Scene being wrapped
   * @see SerialSceneWrapper::scene
   * @see SerialSceneWrapper::setScene
   * @see SerialSceneWrapper::getStaticScene
   * @see SerialSceneWrapper::getDynScene
   */
  Scene* getScene() const { return scene; }
  /**
   * @brief Set the scene being wrapped
   * @param scene New scene to be wrapped
   * @see SerialSceneWrapper::scene
   * @see SerialSceneWrapper::getScene
   */
  void setScene(Scene* scene) { this->scene = scene; }
  /**
   * @brief Obtain the scene being wrapped as a StaticScene
   * @return Scene being wrapped as a StaticScene
   * @see StaticScene
   * @see SerialSceneWrapper::scene
   * @see SerialSceneWrapper::getScene
   */
  StaticScene* getStaticScene() const { return (StaticScene*)scene; }
  /**
   * @brief Obtain the scene being wrapped as a DynScene
   * @return Scene being wrapped as a DynScene
   * @see DynScene
   * @see SerialSceneWrapper::scene
   * @see SerialSceneWrapper::getScene
   */
  DynScene* getDynScene() const { return (DynScene*)scene; }
};
