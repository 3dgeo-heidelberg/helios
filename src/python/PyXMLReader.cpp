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
