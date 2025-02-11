#include <PyXMLReader.h>
#include <XmlSurveyLoader.h>


std::shared_ptr<Survey> readSurveyFromXml(
    std::string surveyPath,
    std::vector<std::string> assetsPath,
    bool legNoiseDisabled,
    bool rebuildScene
){
    XmlSurveyLoader xmlreader(surveyPath, assetsPath);
    xmlreader.sceneLoader.kdtFactoryType = 4;
    xmlreader.sceneLoader.kdtNumJobs = 0;
    xmlreader.sceneLoader.kdtSAHLossNodes = 32;
    return xmlreader.load(legNoiseDisabled, rebuildScene);
}

std::shared_ptr<Scanner> readScannerFromXml(
    std::string scannerPath,
    std::vector<std::string> assetsPath,
    std::string scannerId
)
{
    XmlAssetsLoader xmlreader(scannerPath, assetsPath);
    std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
        xmlreader.getAssetById("scanner", scannerId, nullptr)
    );

    return scanner;
}

std::shared_ptr<Platform> readPlatformFromXml(
    std::string platformPath,
    std::vector<std::string> assetsPath,
    std::string platformId
)
{
    XmlAssetsLoader xmlreader(platformPath, assetsPath);


    std::shared_ptr<Platform> platform = std::static_pointer_cast<Platform>(
        xmlreader.getAssetById("platform", platformId, nullptr)
    );

    return platform;
}

std::shared_ptr<Scene> readSceneFromXml(
    std::string filePath,
    std::vector<std::string> assetsPath,
    bool legNoiseDisabled,
    bool rebuildScene
) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filePath.c_str()) != tinyxml2::XML_SUCCESS) {
        logging::ERR("ERROR: Failed to load XML file " + filePath);
        return nullptr;
    }

    tinyxml2::XMLNode* rootNode = doc.FirstChild();
    if (!rootNode) {
        logging::ERR("ERROR: XML root not found in file " + filePath);
        return nullptr;
    }

    tinyxml2::XMLElement* documentElement = doc.FirstChildElement("document");

    for (tinyxml2::XMLElement* element = documentElement->FirstChildElement(); element != nullptr; element = element->NextSiblingElement()) {
        std::string elementName = element->Name();
        if (elementName == "survey") {

            std::string sceneString = element->Attribute("scene");
            XmlSurveyLoader xmlSurveyLoader(filePath, assetsPath);
            xmlSurveyLoader.sceneLoader.kdtFactoryType = 4;
            xmlSurveyLoader.sceneLoader.kdtNumJobs = 0;
            xmlSurveyLoader.sceneLoader.kdtSAHLossNodes = 32;
            return xmlSurveyLoader.loadScene(sceneString, rebuildScene);
        } else if (elementName == "scene") {
            // Load the scene directly from a scene node
            XmlSceneLoader xmlSceneLoader(assetsPath);

            SerialSceneWrapper::SceneType sceneType;
            std::shared_ptr<Scene> scene = xmlSceneLoader.createSceneFromXml(
                element, filePath, &sceneType);

            if (scene) {
                std::string sceneObjPath = filePath + ".scene";
                SerialSceneWrapper(sceneType, scene.get()).writeScene(sceneObjPath);

                scene->buildKDGroveWithLog();
            } else {
                logging::ERR("Error: Failed to create scene from XML");
                return nullptr;
            }

            return scene;
        }
    }
    return nullptr;
}

std::shared_ptr<ScenePart> readScenePartFromXml(
    std::string filePath,
    std::vector<std::string> assetsPath,
    int id
) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filePath.c_str()) != tinyxml2::XML_SUCCESS) {
        logging::ERR("Failed to load file: " + filePath);
        return nullptr;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("document");
    if (!root) {
        logging::ERR("Invalid XML structure: Missing <document> root");
        return nullptr;
    }

    tinyxml2::XMLElement* scene = root->FirstChildElement("scene");
    if (!scene) {

        logging::ERR("Invalid XML structure: Missing <scene> element");
        return nullptr;
    }

    tinyxml2::XMLElement* part = scene->FirstChildElement("part");
    int currentIndex = 0; 
    std::string finalId = "";
    bool splitPart = true;
    bool holistic = false;

    while (part) {
        const char* partId = part->Attribute("id");
        if (partId) {
      
            try{
                int parsedId = std::stoi(partId);
                if (parsedId == id) {
                    finalId = partId;
                    splitPart = false;
                    break;
                }
            } catch (std::invalid_argument& e) {
                logging::ERR("Error: Invalid ID format in XML for part: " + std::string(partId));
                return nullptr;
            }
        } else {
        
            if (currentIndex == id) {
                finalId = std::to_string(currentIndex);
                break;
            }
        }

        part = part->NextSiblingElement("part");
        currentIndex++;
    }
    
    if (finalId.empty()) {
        logging::ERR("Error: No matching part found for id: " + std::to_string(id));
        return nullptr;
    }
    XmlSceneLoader xmlSceneLoader(assetsPath);

    shared_ptr<ScenePart> scenePart = xmlSceneLoader.loadFilters(part, holistic);
    scenePart->mId = finalId;

    // For all primitives, set reference to their scene part and transform:
    ScenePart::computeTransformations(scenePart, holistic);

    // Infer type of primitive for the scene part
    auto numVertices = scenePart->mPrimitives[0]->getNumVertices();
    if(numVertices == 3)
        scenePart->primitiveType = ScenePart::TRIANGLE;
    else 
        scenePart->primitiveType = ScenePart::VOXEL;

    if(!xmlSceneLoader.validateScenePart(scenePart, part)){
        logging::ERR("Error: Invalid scene part");
        return nullptr;
    }

    return scenePart;
}


std::shared_ptr<KDTreeFactory> makeKDTreeFactory(int kdtFactoryType, int kdtNumJobs, int kdtGeomJobs, int kdtSAHLossNodes){
    if(kdtFactoryType == 1){ // Simple
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeSimpleMultiThread(
                kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeSimple();
    }
    else if(kdtFactoryType == 2){ // SAH
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeSAHMultiThread(
                kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeSAH(kdtSAHLossNodes);
    }
    else if(kdtFactoryType == 3){ // Axis SAH
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeAxisSAHMultiThread(
                kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeAxisSAH(kdtSAHLossNodes);
    }
    
    if(kdtNumJobs > 1){
        return KDTreeFactoryMaker::makeFastSAHMultiThread(
            kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
        );
    }
    return KDTreeFactoryMaker::makeFastSAH(kdtSAHLossNodes);
}


void finalizeStaticScene(std::shared_ptr<StaticScene> scene, int kdtFactoryType, int kdtNumJobs, int kdtGeomJobs, int kdtSAHLossNodes) {
    // Loop over all scene parts and perform their final processing
    for (auto& sp : scene->parts) {
        // Append as a static object
        scene->appendStaticObject(sp);

        // Add scene part primitives to the scene
        scene->primitives.insert(
            scene->primitives.end(),
            sp->mPrimitives.begin(),
            sp->mPrimitives.end()
        );
    }

    // Call scene finalization
    if (!scene->finalizeLoading()) {
        throw std::runtime_error("Finalizing the scene failed.");
    }

    // Build KDGroveFactory
    scene->setKDGroveFactory(std::make_shared<KDGroveFactory>(makeKDTreeFactory(kdtFactoryType, kdtNumJobs, kdtGeomJobs, kdtSAHLossNodes)));
}


void invalidateStaticScene(std::shared_ptr<StaticScene> scene) {
    scene->clearStaticObjects();
    scene->setKDGroveFactory(nullptr);
    scene->primitives.clear();
}
