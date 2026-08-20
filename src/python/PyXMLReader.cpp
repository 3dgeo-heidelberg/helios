#include <HeliosException.h>
#include <PyXMLReader.h>
#include <SpectralLibrary.h>
#include <XmlSurveyLoader.h>

std::shared_ptr<Survey>
readSurveyFromXml(std::string surveyPath,
                  std::vector<std::string> assetsPath,
                  bool legNoiseDisabled)
{
  XmlSurveyLoader xmlreader(surveyPath, assetsPath);
  xmlreader.sceneLoader.kdtFactoryType = 4;
  xmlreader.sceneLoader.kdtNumJobs = 0;
  xmlreader.sceneLoader.kdtSAHLossNodes = 32;
  return xmlreader.load(legNoiseDisabled);
}

std::shared_ptr<Scanner>
readScannerFromXml(std::string scannerPath,
                   std::vector<std::string> assetsPath,
                   std::string scannerId)
{
  XmlAssetsLoader xmlreader(scannerPath, assetsPath);
  std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
    xmlreader.getAssetById("scanner", scannerId, nullptr));
  scanner->initializeSequentialGenerators();

  return scanner;
}

std::shared_ptr<Platform>
readPlatformFromXml(std::string platformPath,
                    std::vector<std::string> assetsPath,
                    std::string platformId)
{
  XmlAssetsLoader xmlreader(platformPath, assetsPath);

  std::shared_ptr<Platform> platform = std::static_pointer_cast<Platform>(
    xmlreader.getAssetById("platform", platformId, nullptr));

  return platform;
}

std::shared_ptr<Scene>
readSceneFromXml(std::string filePath,
                 std::vector<std::string> assetsPath,
                 bool buildKDGrove)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filePath.c_str()) != tinyxml2::XML_SUCCESS) {
    std::stringstream ss;
    ss << "Failed to load XML file '" << filePath << "'";

    if (doc.ErrorStr() != nullptr)
      ss << ": " << doc.ErrorStr();

    if (doc.ErrorLineNum() > 0)
      ss << " at line " << doc.ErrorLineNum();

    throw HeliosException(ss.str());
  }

  tinyxml2::XMLElement* documentElement = doc.FirstChildElement("document");
  if (documentElement == nullptr) {
    throw HeliosException("XML <document> root not found in file: " + filePath);
  }

  for (tinyxml2::XMLElement* element = documentElement->FirstChildElement();
       element != nullptr;
       element = element->NextSiblingElement()) {
    std::string elementName = element->Name();
    if (elementName == "survey") {

      const char* sceneStringAttr = element->Attribute("scene");
      if (sceneStringAttr == nullptr || *sceneStringAttr == '\0') {
        throw HeliosException("Survey is missing required 'scene' attribute.");
      }
      std::string sceneString(sceneStringAttr);
      XmlSurveyLoader xmlSurveyLoader(filePath, assetsPath);
      xmlSurveyLoader.sceneLoader.kdtFactoryType = 4;
      xmlSurveyLoader.sceneLoader.kdtNumJobs = 0;
      xmlSurveyLoader.sceneLoader.kdtSAHLossNodes = 32;
      return xmlSurveyLoader.loadScene(sceneString);
    }
    if (elementName == "scene") {
      // Load the scene directly from a scene node
      XmlSceneLoader xmlSceneLoader(assetsPath);
      std::shared_ptr<Scene> scene =
        xmlSceneLoader.createSceneFromXml(element, filePath);
      if (buildKDGrove)
        scene->buildKDGroveWithLog();
      return scene;
    }
  }
  throw HeliosException("Failed to create scene from XML '" + filePath +
                        "': no <survey> or <scene> element found.");
}

std::shared_ptr<DynScene>
readDynamicSceneFromXml(std::string filePath,
                        std::vector<std::string> assetsPath,
                        bool buildKDGrove)
{
  std::shared_ptr<Scene> scene =
    readSceneFromXml(filePath, assetsPath, buildKDGrove);
  if (!scene) {
    throw HeliosException(
      "read_dynamic_scene_from_xml() failed to load a scene from XML file '" +
      filePath + "'.");
  }

  std::shared_ptr<DynScene> dynamicScene =
    std::dynamic_pointer_cast<DynScene>(scene);
  if (!dynamicScene) {
    throw HeliosException(
      "read_dynamic_scene_from_xml() requires XML that produces a dynamic "
      "scene, but '" +
      filePath +
      "' produced a static scene. Use read_scene_from_xml() instead.");
  }

  return dynamicScene;
}

std::shared_ptr<ScenePart>
readScenePartFromXml(std::string filePath,
                     std::vector<std::string> assetsPath,
                     int id)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filePath.c_str()) != tinyxml2::XML_SUCCESS) {
    throw HeliosException("Failed to load XML file '" + filePath +
                          "': " + doc.ErrorStr());
  }

  tinyxml2::XMLElement* root = doc.FirstChildElement("document");
  if (!root) {
    throw HeliosException("Invalid XML structure: Missing <document> root");
  }

  tinyxml2::XMLElement* scene = root->FirstChildElement("scene");
  if (!scene) {
    throw HeliosException("Invalid XML structure in '" + filePath +
                          "': Missing <scene> element");
  }

  tinyxml2::XMLElement* part = scene->FirstChildElement("part");
  int currentIndex = 0;
  std::string finalId = "";
  bool holistic = false;

  while (part) {
    const char* partId = part->Attribute("id");
    if (partId) {
      int parsedId = 0;

      if (part->QueryIntAttribute("id", &parsedId) != tinyxml2::XML_SUCCESS) {
        throw HeliosException("Invalid scene part ID: '" + std::string(partId) +
                              "'.");
      }
      if (parsedId == id) {
        finalId = partId;
        break;
      }
    } else if (currentIndex == id) {
      finalId = std::to_string(currentIndex);
      break;
    }

    part = part->NextSiblingElement("part");
    currentIndex++;
  }

  if (part == nullptr || finalId.empty()) {
    throw HeliosException("No scene part found with id " + std::to_string(id) +
                          ".");
  }
  XmlSceneLoader xmlSceneLoader(assetsPath);

  std::shared_ptr<ScenePart> scenePart =
    xmlSceneLoader.loadFilters(part, holistic);

  scenePart->mId = finalId;
  if (scenePart->mPrimitives.empty()) {
    throw HeliosException("Scene part with id '" + finalId +
                          "' contains no primitives.");
  }
  // For all primitives, set reference to their scene part and transform:
  ScenePart::computeTransformations(scenePart, holistic);

  // Infer type of primitive for the scene part
  auto numVertices = scenePart->mPrimitives[0]->getNumVertices();
  if (numVertices == 3)
    scenePart->primitiveType = ScenePart::TRIANGLE;
  else
    scenePart->primitiveType = ScenePart::VOXEL;

  return scenePart;
}
