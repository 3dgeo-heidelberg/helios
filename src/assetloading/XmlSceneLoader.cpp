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
    std::shared_ptr<Scene> scene = std::make_shared<Scene>();
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

        // Load filter nodes, if any
        ScenePart *scenePart = loadFilters(scenePartNode, holistic);

        // Load rigid motions if any
        shared_ptr<DynSequentiableMovingObject> dsmo =
            loadRigidMotions(scenePartNode, scenePart);
        if(!dynScene && dsmo != nullptr){
            scene = makeSceneDynamic(scene);
            dynScene = true;
        }
        if(dsmo != nullptr){
            std::static_pointer_cast<DynScene>(scene)->appendDynObject(dsmo);
        }

        // Read and set scene part ID
        bool splitPart = loadScenePartId(scenePartNode, partIndex, scenePart);

        // Consider scene loading specification
        scenePartCounter++;
        sceneSpec.apply(scenePart);

        // Digest scene part
        digestScenePart(scenePart, scene, holistic, splitPart, partIndex);

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

ScenePart * XmlSceneLoader::loadFilters(
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
    return scenePart;
}

shared_ptr<DynSequentiableMovingObject> XmlSceneLoader::loadRigidMotions(
    tinyxml2::XMLElement *scenePartNode,
    ScenePart *scenePart
){
    // Find first rmotion node
    tinyxml2::XMLElement *rmotionNode =
        scenePartNode->FirstChildElement("rmotion");
    if(rmotionNode == nullptr) return nullptr; // No rmotion found

    // Build dynamic sequential moving object from XML
    shared_ptr<DynSequentiableMovingObject> dsmo =
        make_shared<DynSequentiableMovingObject>();
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

    // Return
    return dsmo;
}

bool XmlSceneLoader::loadScenePartId(
    tinyxml2::XMLElement *scenePartNode,
    int partIndex,
    ScenePart *scenePart
){
    std::string partId = "";
    tinyxml2::XMLAttribute const *partIdAttr =
        scenePartNode->FindAttribute("id");
    char const *str = NULL;
    if (partIdAttr != NULL)
        str = scenePartNode->FindAttribute("id")->Value();
    if (str != NULL) {
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
    ScenePart *scenePart,
    std::shared_ptr<Scene> scene,
    bool holistic,
    bool splitPart,
    int &partIndex
){
    // For all primitives, set reference to their scene part and transform:
    std::shared_ptr<ScenePart> sharedScenePart(scenePart);
    for (Primitive *p : scenePart->mPrimitives) {
        p->part = sharedScenePart;
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

    // Add scene part to the scene:
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

shared_ptr<Scene> XmlSceneLoader::makeSceneDynamic(shared_ptr<Scene> scene){
    return std::static_pointer_cast<Scene>(make_shared<DynScene>(*scene));
}
