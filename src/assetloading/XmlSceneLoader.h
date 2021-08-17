#pragma once

#include <tinyxml2.h>

#include <Scene.h>
#include <SceneLoadingSpecification.h>

/**
 * @brief Class for scene loading from XML file.
 *
 * Whenever possible, it is preferred to load XML defined components by
 *  XmlAssetsLoader or XmlSurveyLoader classes which are the main classes for
 *  this purpose.
 *
 * @see XmlAssetsLoader
 */
class XmlSceneLoader {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief Scene loading specification
	 * @see SceneLoadingSpecification
	 */
    SceneLoadingSpecification sceneSpec;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    XmlSceneLoader() = default;
    virtual ~XmlSceneLoader() {}

    // ***  SCENE CREATION  *** //
    // ************************ //
    /**
	 * @brief Create scene from given XML element (node)
	 * @param sceneNode XML element (node) containing scene data
	 * @param path Path to scene file
	 * @return Shared pointer to created scene
	 * @see Scene
	 */
    std::shared_ptr<Scene> createSceneFromXml(
        tinyxml2::XMLElement* sceneNode,
        std::string path
    );
};