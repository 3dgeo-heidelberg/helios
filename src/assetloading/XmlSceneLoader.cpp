#include <XmlSceneLoader.h>
#include <XmlUtils.h>
#include <TimeWatcher.h>
#include <GeoTiffFileLoader.h>
#include <WavefrontObjFileLoader.h>
#include <XYZPointCloudFileLoader.h>
#include <DetailedVoxelLoader.h>
#include <scene/dynamic/DynScene.h>

#include <logging.hpp>

#include <boost/lexical_cast.hpp>
#include <memory>


// ***  SCENE CREATION  *** //
// ************************ //

std::shared_ptr<Scene>
XmlSceneLoader::createSceneFromXml(
    tinyxml2::XMLElement *sceneNode,
    std::string path
){
    std::shared_ptr<StaticScene> scene = std::make_shared<StaticScene>();
    bool dynScene = false;
    scene->sourceFilePath = path;

    TimeWatcher tw;

    // Loop over all part nodes
    int partIndex = -1;
    tinyxml2::XMLElement *scenePartNode = sceneNode->FirstChildElement("part");
    size_t scenePartCounter = 0;
    tw.start();
    while (scenePartNode != nullptr) {
        partIndex++;
        bool holistic = false;
        bool dynObject = false;

        // Load filter nodes, if any
        shared_ptr<ScenePart> scenePart = loadFilters(scenePartNode, holistic);

        // Read and set scene part ID
        bool splitPart = loadScenePartId(scenePartNode, partIndex, scenePart);

        // Load rigid motions if any
        shared_ptr<DynSequentiableMovingObject> dsmo =
            loadRigidMotions(scenePartNode, scenePart);
        if(!dynScene && dsmo != nullptr){
            scene = makeSceneDynamic(scene);
            dynScene = true;
        }
        if(dsmo != nullptr){ // Append to scene, replace scene part, flag dyn
            std::static_pointer_cast<DynScene>(scene)->appendDynObject(dsmo);
            scenePart = std::static_pointer_cast<ScenePart>(dsmo);
            dynObject = true;
        }

        // Consider scene loading specification
        scenePartCounter++;
        sceneSpec.apply(scenePart);

        // Digest scene part
        digestScenePart(
            scenePart, scene, holistic, splitPart, dynObject, partIndex
        );

        // Read next scene part from XML
        scenePartNode = scenePartNode->NextSiblingElement("part");
    }

    // Report elapsed time
    tw.stop();
    std::stringstream ss;
    ss  << std::to_string(scenePartCounter) << " sceneparts loaded in "
        << tw.getElapsedDecimalSeconds() << "s\n";
    logging::INFO(ss.str());


    // Finish scene loading
    bool success = scene->finalizeLoading();
    if (!success) {
        logging::ERR("Finalizing the scene failed.");
        exit(-1);
    }

    // Return built scene
    return scene;
}

shared_ptr<ScenePart> XmlSceneLoader::loadFilters(
    tinyxml2::XMLElement *scenePartNode,
    bool &holistic
){
    ScenePart *scenePart = nullptr;
    tinyxml2::XMLElement *filterNodes =
        scenePartNode->FirstChildElement("filter");
    while (filterNodes != nullptr) {
        std::string filterType = filterNodes->Attribute("type");
        std::transform(
            filterType.begin(),
            filterType.end(),
            filterType.begin(),
            ::tolower
        );
        AbstractGeometryFilter *filter = nullptr;

        // ################### BEGIN Set up filter ##################

        // Apply scale transformation:
        if (filterType == "scale") {
            filter = new ScaleFilter(scenePart);
        }

            // Read GeoTiff file:
        else if (filterType == "geotiffloader") {
            filter = new GeoTiffFileLoader();
        }

            // Read Wavefront Object file:
        else if (filterType == "objloader") {
            filter = new WavefrontObjFileLoader();
        }

            // Apply rotation filter:
        else if (filterType == "rotate") {
            filter = new RotateFilter(scenePart);
        }

            // Apply translate transformation:
        else if (filterType == "translate") {
            filter = new TranslateFilter(scenePart);
        }

            // Read xyz ASCII point cloud file:
        else if (filterType == "xyzloader") {
            filter = new XYZPointCloudFileLoader();
            holistic = true;
        }

            // Read detailed voxels file
        else if (filterType == "detailedvoxels") {
            filter = new DetailedVoxelLoader();
        }

        // ################### END Set up filter ##################

        // Finally, apply the filter:
        if (filter != nullptr) {
            // Set params:
            filter->params = XmlUtils::createParamsFromXml(filterNodes);
            logging::DEBUG("Applying filter: " + filterType);
            scenePart = filter->run();
            if (scenePart == filter->primsOut)
                filter->primsOut = nullptr;
            delete filter;
        }

        filterNodes = filterNodes->NextSiblingElement("filter");
    }
    // ############## END Loop over filter nodes ##################
    return shared_ptr<ScenePart>(scenePart);
}

shared_ptr<DynSequentiableMovingObject> XmlSceneLoader::loadRigidMotions(
    tinyxml2::XMLElement *scenePartNode,
    shared_ptr<ScenePart> scenePart
){
    // Find first rmotion node
    tinyxml2::XMLElement *rmotionNode =
        scenePartNode->FirstChildElement("rmotion");
    if(rmotionNode == nullptr) return nullptr; // No rmotion found

    // Build dynamic sequential moving object from XML
    shared_ptr<DynSequentiableMovingObject> dsmo =
        make_shared<DynSequentiableMovingObject>(*scenePart);
    while(rmotionNode != nullptr){
        // Optional attributes
        std::string nextId = "";
        tinyxml2::XMLAttribute const *nextIdAttr =
            rmotionNode->FindAttribute("next");
        if(nextIdAttr != nullptr) nextId = nextIdAttr->Value();

        // Mandatory attributes
        tinyxml2::XMLAttribute const *idAttr =
            rmotionNode->FindAttribute("id");
        if(idAttr == nullptr) throw HeliosException(
            "XmlSceneLoader::loadRigidMotions found a rmotion element with "
            "no id"
        );
        tinyxml2::XMLAttribute const *loopAttr =
            rmotionNode->FindAttribute("loop");
        if(loopAttr == nullptr) throw HeliosException(
            "XmlSceneLoader::loadRigidMotions found a rmotion element with "
            "no loop"
        );

        // Add rigid motion sequence
        shared_ptr<DynSequence<RigidMotion>> rmSequence =
            make_shared<DynSequence<RigidMotion>>(
                idAttr->Value(),
                nextId,
                loopAttr->Int64Value()
            );
        rmSequence->append(XmlUtils::createRigidMotionsVector(rmotionNode));
        if(rmSequence == nullptr){  // Check rmSequence is not null
            throw HeliosException(
                "XmlSceneLoader::loadRigidMotions found a rmotion element with"
                " no motions. This is not allowed."
            );
        }
        dsmo->addSequence(rmSequence);

        // Next rigid motion sequence
        rmotionNode = rmotionNode->NextSiblingElement("rmotion");
    }

    for(Primitive *primitive : dsmo->mPrimitives){
        primitive->part = dsmo;
    }

    // Use scene part ID to build dynamic dynamic sequentiable moving object ID
    std::stringstream ss;
    ss << "DSMO_" << scenePart->mId;
    dsmo->setId(ss.str());

    // Return
    return dsmo;
}

bool XmlSceneLoader::loadScenePartId(
    tinyxml2::XMLElement *scenePartNode,
    int partIndex,
    shared_ptr<ScenePart> scenePart
){
    std::string partId = "";
    tinyxml2::XMLAttribute const *partIdAttr =
        scenePartNode->FindAttribute("id");
    char const *str = nullptr;
    if(partIdAttr!=nullptr) str = scenePartNode->FindAttribute("id")->Value();
    if (str != nullptr) {
        partId = std::string(str);
        try {
            boost::lexical_cast<int>(partId);
        } catch (boost::bad_lexical_cast &blcex) {
            std::stringstream exss;
            exss << "You have provided a scene part id \"" << partId
                 << "\" which is non numerical.\n"
                 << "Caution! "
                 << "This is not compatible with LAS format specification"
                 << std::endl;
            logging::INFO(exss.str());
        }
    }

    bool splitPart = true;
    if (partId.empty()) {
        scenePart->mId = std::to_string(partIndex);
    } else {
        scenePart->mId = partId;
        splitPart = false;
    }

    return splitPart;
}

void XmlSceneLoader::digestScenePart(
    shared_ptr<ScenePart> scenePart,
    shared_ptr<StaticScene> scene,
    bool holistic,
    bool splitPart,
    bool dynObject,
    int &partIndex
){
    // For all primitives, set reference to their scene part and transform:
    for (Primitive *p : scenePart->mPrimitives) {
        p->part = scenePart;
        p->rotate(scenePart->mRotation);
        if (holistic) {
            for (size_t i = 0; i < p->getNumVertices(); i++) {
                p->getVertices()[i].pos.x *= scenePart->mScale;
                p->getVertices()[i].pos.y *= scenePart->mScale;
                p->getVertices()[i].pos.z *= scenePart->mScale;
            }
        }
        p->scale(scenePart->mScale);
        p->translate(scenePart->mOrigin);
    }

    // Append as static object if it is not dynamic
    // If it is dynamic, it must have been appended before
    if(!dynObject) scene->appendStaticObject(scenePart);

    // Add scene part primitives to the scene
    scene->primitives.insert(
        scene->primitives.end(),
        scenePart->mPrimitives.begin(),
        scenePart->mPrimitives.end()
    );

    // Split subparts into different scene parts
    if (splitPart) {
        size_t partIndexOffset = scenePart->subpartLimit.size() - 1;
        if (scenePart->splitSubparts()) partIndex += partIndexOffset;
    }
}

shared_ptr<StaticScene> XmlSceneLoader::makeSceneDynamic(
    shared_ptr<StaticScene> scene
){
    // Upgrade static scene to dynamic scene
    std::shared_ptr<StaticScene> newScene =
        std::static_pointer_cast<StaticScene>(make_shared<DynScene>(*scene));

    // Primitives were automatically updated at scene level
    // Now, update them at static object level
    // It is as simple as rebuilding the static objects because at this point
    // there is no dynamic object in the scene yet, since it didnt support them
    newScene->clearStaticObjects();
    std::set<shared_ptr<ScenePart>> parts; // Scene parts with no repeats
    std::vector<Primitive *> const primitives = newScene->primitives;
    for(Primitive *primitive : primitives)
        if(primitive->part != nullptr) parts.insert(primitive->part);
    for(shared_ptr<ScenePart> part : parts)
        newScene->appendStaticObject(part);

    // Return upgraded scene
    return newScene;
}
