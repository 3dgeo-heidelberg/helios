#pragma once

#include <tinyxml2.h>

#include <Scene.h>
#include <StaticScene.h>
#include <DynScene.h>
#include <SerialSceneWrapper.h>
#include <SceneLoadingSpecification.h>
#include <scene/dynamic/DynSequentiableMovingObject.h>
#include <KDTreeFactory.h>
#include <KDGroveFactory.h>

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
    /**
     * @brief Type of KDTree factory to be used to build scene
     * @see KDTreeFactory
     */
    int kdtFactoryType = 1;
    /**
     * @brief How many threads must be used to build KDTree
     * @see KDTreeFactory
     * @see SimpleKDTreeFactory
     * @see MultiThreadKDTreeFactory
     * @see SAHKDTreeFactory
     * @see MultiThreadSAHKDTreeFactory
     */
    size_t kdtNumJobs = 1;
    /**
     * @brief How many threads must be used to build upper nodes of KDTree
     * @see KDTreeFactory
     * @see SimpleKDTreeFactory
     * @see MultiThreadKDTreeFactory
     * @see SAHKDTreeFactory
     * @see MultiThreadSAHKDTreeFactory
     */
    size_t kdtGeomJobs = 1;
    /**
     * @brief How many loss nodes for the Surface Area Heuristic if using a SAH
     *  like KDTree factory to build the scene
     */
    size_t kdtSAHLossNodes = 21;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for XML scene loader
     */
    XmlSceneLoader() :
        kdtFactoryType(1),
        kdtNumJobs(1),
        kdtGeomJobs(1),
        kdtSAHLossNodes(21)
    {}
    virtual ~XmlSceneLoader() {}

    // ***  SCENE CREATION  *** //
    // ************************ //
    /**
	 * @brief Create scene from given XML element (node)
	 * @param sceneNode XML element (node) containing scene data
	 * @param path Path to scene file
     * @param sceneType[out] When it is not null, it will be used to store the
     *  type of created scene
	 * @return Shared pointer to created scene
	 * @see Scene
	 */
    std::shared_ptr<Scene> createSceneFromXml(
        tinyxml2::XMLElement* sceneNode,
        std::string path,
        SerialSceneWrapper::SceneType *sceneType = nullptr
    );

    /**
     * @brief Load filters defining the scene part.
     *
     * NOTICE a scene part requires at least one primitives loading filter to
     *  be instantiated, otherwise it will be nullptr
     *
     * @param scenePartNode XML part node defining the scene part
     * @param[out] holistic Used to specify if all vertices defining each
     *  primitive must be considered as a whole (true) or not
     * @return Built scene part if any, nullptr otherwise
     */
    shared_ptr<ScenePart> loadFilters(
        tinyxml2::XMLElement *scenePartNode,
        bool &holistic
    );

    /**
     * @brief Load the scene part identifier
     * @param scenePartNode XML part node where the identifier might be
     *  explicitly specified
     * @param partIndex Index of scene part according to current loop
     *  iteration. It will be used if no specific identifier is provided
     *  through XML
     * @param scenePart The scene part object to which identifier must be
     *  assigned
     * @return True if scene part must be splitted, false otherwise. A scene
     *  part can only be splitted when a part identifier is explicitly provided
     */
    bool loadScenePartId(
        tinyxml2::XMLElement *scenePartNode,
        int partIndex,
        shared_ptr<ScenePart> scenePart
    );

    /**
     * @brief Apply final processings to the built scene part so it is fully
     *  integrated in the scene and totally configured
     * @param scenePart The scene part object to be digested
     * @param scene The scene where the scene part belongs
     * @param holistic Flag used to specify if all vertices defining each
     *  primitive must be considered as a whole (true) or not (false)
     * @param splitPart Flag to specify if scene part must be splitted into
     *  subparts (true) or not (false)
     * @param dynObject Flag to specify if the scene part corresponds to a
     *  dynamic object (true) or to a static one (false)
     * @param[out] partIndex If the subpart is splitted, then partIndex will
     *  be opportunely updated
     * @see ScenePart::splitSubparts
     */
    void digestScenePart(
        shared_ptr<ScenePart> &scenePart,
        std::shared_ptr<StaticScene> &scene,
        bool holistic,
        bool splitPart,
        bool dynObject,
        int &partIndex
    );

    /**
     * @brief Build the KDTree factory from loader's kdtFactoryType and
     *  kdtSAHLossNodes attributes
     * @return Built KDTree factory
     * @see XmlSceneLoader::kdtFactoryType
     * @see XmlSceneLoader::kdtSAHLossNodes
     * @see KDTreeFactory
     */
    shared_ptr<KDTreeFactory> makeKDTreeFactory();

    /**
     * @brief Build the KDGrove factory from loader's KDTree factory
     *  specification
     * @return Build KDGrove factory
     * @see XmlSceneLoader::makeKDTreeFactory
     * @see KDGroveFactory
     */
    shared_ptr<KDGroveFactory> makeKDGroveFactory();

    // ***  DYNAMIC SCENE LOADING METHODS  *** //
    // *************************************** //
    /**
     * @brief Build a dynamic sequentiable moving object which is composed of
     *  dynamic motions.
     *
     * It is mandatory that dmotion elements contained in the part element
     *  also contain an id attribute specifying the unique identifier for the
     *  sequence in its context. The loop attribute is also mandatory, where
     *  \f$0\f$ means infinity loop and \f$n > 0\f$ specifies how many times
     *  the sequence will be repeated until proceeding to next sequence. Next
     *  sequence can be specified through the next attribute, which may contain
     *  the identifier of the next sequence. If no next attribute is given,
     *  then it is assumed that there is no next sequence.
     *
     * @param scenePartNode XML part node defining the scene part
     * @param scenePart The scene part object where the dynamic sequentiable
     *  moving object belongs to
     * @return Built dynamic sequentiable moving object composed of dynamic
     *  motions specified in the XML
     * @see DynSequencer
     * @see DynSequence
     * @see rigidmotion::RigidMotion
     * @see DynMotion
     * @see XmlUtils::createDynMotionsVector
     */
    shared_ptr<DynSequentiableMovingObject> loadDynMotions(
        tinyxml2::XMLElement *scenePartNode,
        shared_ptr<ScenePart> scenePart
    );

    /**
     * @brief Build a dynamic scene based on given static scene.
     *
     * NOTICE for this method to work properly given scene MUST be of
     *  StaticScene type or unexpected behaviors might occur. Use with caution.
     *
     * @param scene Static scene to be used to build a dynamic scene
     * @return Built dynamic scene based on given static scene
     * @see StaticScene
     * @see DynScene
     */
    shared_ptr<StaticScene> makeSceneDynamic(shared_ptr<StaticScene> scene);

    /**
     * @brief Handle the loading of dynamic scene attributes
     * @param sceneNode The XML node defining the dynamic scene
     * @param scene Dynamic scene which attributes must be configured
     */
    void handleDynamicSceneAttributes(
        tinyxml2::XMLElement *sceneNode,
        shared_ptr<DynScene> scene
    );
};