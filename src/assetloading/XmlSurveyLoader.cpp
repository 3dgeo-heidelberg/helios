#include <chrono>
using namespace std::chrono;

#include <boost/algorithm/string.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "typedef.h"
#include <XmlUtils.h>
#include <logging.hpp>

#include "TimeWatcher.h"
#include "XmlSurveyLoader.h"
#include <RandomnessGenerator.h>
#include <SerialSceneWrapper.h>

#include <unordered_set>


using namespace glm;
using namespace std;

shared_ptr<Survey> XmlSurveyLoader::load(
    bool legNoiseDisabled,
    bool rebuildScene
){
  tinyxml2::XMLNode *pRoot = doc.FirstChild();
  if (pRoot == nullptr) {
    logging::ERR("ERROR: xml root not found");
    return nullptr;
  }
  tinyxml2::XMLElement *surveyNodes =
      pRoot->NextSibling()->FirstChildElement("survey");
  if (surveyNodes == nullptr) {
    stringstream ss;
    ss << "XML Survey playback loader: "
       << "ERROR: No survey elements found in file " << this->xmlDocFilename;
    logging::WARN(ss.str());
    return NULL;
  }

  return createSurveyFromXml(surveyNodes, legNoiseDisabled, rebuildScene);
}

shared_ptr<Survey>
XmlSurveyLoader::createSurveyFromXml(
    tinyxml2::XMLElement *surveyNode,
    bool legNoiseDisabled,
    bool rebuildScene
) {
  // Prepare survey loading
  reinitLoader();
  shared_ptr<Survey> survey = make_shared<Survey>();

  // Load survey core
  loadSurveyCore(surveyNode, survey);

  // Load legs
  loadLegs(
    surveyNode->FirstChildElement("leg"),
    survey->scanner->retrieveCurrentSettings(),
    survey->scanner->platform->retrieveCurrentSettings(),
    survey->legs
  );

  // NOTE:
  // The scene is loaded as the last step, since it takes the longest time.
  // In the case that something goes wrong during the parsing
  // of the survey description, the user is noticed immediately and does not
  // need to wait until the scene is loaded, only to learn that something has
  // failed.

  // Load scene
  string sceneString = surveyNode->Attribute("scene");
  survey->scanner->platform->scene = loadScene(sceneString, rebuildScene);
  SpectralLibrary spectralLibrary = SpectralLibrary(
      (float)survey->scanner->getWavelength(), assetsDir + "spectra");
  spectralLibrary.readReflectances();
  spectralLibrary.setReflectances(survey->scanner->platform->scene.get());

  // Apply scene geometry shift to platform waypoints
  applySceneShift(surveyNode, legNoiseDisabled, survey);

  // Configure default randmoness generator
  configureDefaultRandomnessGenerator(surveyNode);

  // Load platform noise
  loadPlatformNoise(surveyNode, survey->scanner->platform);

  // Initialize scanner randomness generators
  survey->scanner->initializeSequentialGenerators();

  // Return created survey
  return survey;
}

shared_ptr<Leg>
XmlSurveyLoader::createLegFromXML(
    tinyxml2::XMLElement *legNode,
    std::unordered_set<std::string> *scannerFields,
    std::unordered_set<std::string> *platformFields

){
  shared_ptr<Leg> leg = make_shared<Leg>();

  // Leg serial ID
  ++lastLegSerialId;
  leg->setSerialId(lastLegSerialId);

  // Strip ID
  std::string stripId = boost::get<string>(XmlUtils::getAttribute(
      legNode, "stripId", "string", ScanningStrip::NULL_STRIP_ID
  ));
  if(stripId != ScanningStrip::NULL_STRIP_ID){ // Handle strip
      std::shared_ptr<ScanningStrip> strip = nullptr;
      if(strips.find(stripId) != strips.end()) strip = strips[stripId];
      else{
          strip = std::make_shared<ScanningStrip>(stripId);
          strips.emplace(stripId, strip);
      }
      leg->setStrip(strip);
      strip->emplace(leg.get());
  }

  // Platform settings
  tinyxml2::XMLElement *platformSettingsNode =
      legNode->FirstChildElement("platformSettings");
  if (platformSettingsNode != nullptr) {
      leg->mPlatformSettings =
          createPlatformSettingsFromXml(platformSettingsNode);
  }

  // Scanner settings
  tinyxml2::XMLElement *scannerSettingsNode =
      legNode->FirstChildElement("scannerSettings");
  if (scannerSettingsNode != nullptr) {
    leg->mScannerSettings = createScannerSettingsFromXml(
        scannerSettingsNode, scannerFields
    );
  }
  else {
    leg->mScannerSettings = shared_ptr<ScannerSettings>(new ScannerSettings());
  }

  // Return built leg
  return leg;
}


// ***  UTIL METHODS  *** //
// ********************** //
void XmlSurveyLoader::reinitLoader(){
    XmlAssetsLoader::reinitLoader();
    lastLegSerialId = -1;
    strips.clear();
}
shared_ptr<Scene> XmlSurveyLoader::loadScene(
    string sceneString,
    bool rebuildScene
) {
  logging::INFO("Loading Scene...");

  shared_ptr<Scene> scene;
  TimeWatcher tw;
  tw.start();

  string sceneFullPath;
  try {
    vector<string> paths;
    boost::split(paths, sceneString, boost::is_any_of("#"));
    sceneFullPath = paths.at(0);
    sceneFullPath = sceneFullPath.substr(0, sceneFullPath.length() - 3);
  } catch (...) { // Case for having Survey and Scene in the same XML
    sceneFullPath = xmlDocFilePath.substr(0, xmlDocFilePath.length() - 3);
  }
  string sceneObjPath = sceneFullPath + "scene";
  try {
    fs::path sceneObj(sceneObjPath);
    fs::path sceneXml(sceneFullPath + "xml");
    if (fs::is_regular_file(sceneObj) &&
        fs::last_write_time(sceneObj) > fs::last_write_time(sceneXml) &&
        !rebuildScene
    ){
      SerialSceneWrapper *ssw = SerialSceneWrapper::readScene(sceneObjPath);
      scene = shared_ptr<Scene>(ssw->getScene());
      delete ssw;
    } else {
      SerialSceneWrapper::SceneType sceneType;
      scene = dynamic_pointer_cast<Scene>(
          getAssetByLocation("scene", sceneString, &sceneType)
      );
      SerialSceneWrapper(sceneType, scene.get()).writeScene(sceneObjPath);
      /*
       * Build KDGrove for Scene after exporting it.
       * This way memory issues coming from tracking of pointers at
       *    serialization are avoided, as there is more available memory
       */
      scene->buildKDGroveWithLog();
    }
  } catch (exception &e) {
    stringstream ss;
    ss << "EXCEPTION at XmlSurveyLoader::loadScene:\n\t" << e.what();
    logging::WARN(ss.str());
  }

  if (scene == nullptr) {
    logging::ERR("Error: Cannot load scene");
    exit(1);
  }

  tw.stop();
  stringstream ss;
  ss << "Scene loaded in " << tw.getElapsedDecimalSeconds() << "s";
  logging::TIME(ss.str());

  return scene;
}

void XmlSurveyLoader::loadSurveyCore(
    tinyxml2::XMLElement *surveyNode,
    std::shared_ptr<Survey> survey
){
    // Load survey fields
    survey->name = boost::get<string>(
        XmlUtils::getAttribute(surveyNode, "name", "string", xmlDocFilename));
    survey->sourceFilePath = xmlDocFilePath;
    // Load scanner
    string scannerAssetLocation = boost::get<string>(
        XmlUtils::getAttribute(surveyNode, "scanner", "string", string("")));
    survey->scanner = dynamic_pointer_cast<Scanner>(
        getAssetByLocation("scanner", scannerAssetLocation));
    // Load platform
    string platformAssetLocation = boost::get<string>(
        XmlUtils::getAttribute(surveyNode, "platform", "string", string("")));
    survey->scanner->platform = dynamic_pointer_cast<Platform>(
        getAssetByLocation("platform", platformAssetLocation));
    // Load fullwave form
    tinyxml2::XMLElement *scannerFWFSettingsNode =
        surveyNode->FirstChildElement("FWFSettings");
    survey->scanner->applySettingsFWF(*createFWFSettingsFromXml(
        scannerFWFSettingsNode, std::make_shared<FWFSettings>(
            FWFSettings(survey->scanner->FWF_settings)
        )
    ));
    // Read number of runs
    survey->numRuns = boost::get<int>(
        XmlUtils::getAttribute(surveyNode, "numRuns", "int", 1)
    );
    // Load initial simulation speed factor
    double speed = boost::get<double>(XmlUtils::getAttribute(
        surveyNode, "simSpeed", "double", 1.0)
    );
    if (speed <= 0) {
        std::stringstream ss;
        ss << "XMLSurveyLoader::loadSurveyCore "
           << "ERROR: Sim speed can't be <= 0. Setting it to 1.";
        logging::WARN(ss.str());
        speed = 1.0;
    }
    survey->simSpeedFactor = 1.0 / speed;

    // Handle overloads of loaded survey core
    handleCoreOverloading(surveyNode, survey);
}

void XmlSurveyLoader::handleCoreOverloading(
    tinyxml2::XMLElement *surveyNode,
    std::shared_ptr<Survey> survey
){
    // Detector overloading
    AbstractDetector &detector = *(survey->scanner->detector);
    tinyxml2::XMLElement *dsNode = \
        surveyNode->FirstChildElement("detectorSettings");
    if(dsNode!=nullptr){ // If a detector overload is specified, apply it
        detector.cfg_device_accuracy_m = boost::get<double>(
            XmlUtils::getAttribute(
                dsNode, "accuracy_m", "double", detector.cfg_device_accuracy_m
            )
        );
        detector.cfg_device_rangeMin_m = boost::get<double>(
            XmlUtils::getAttribute(
                dsNode, "rangeMin_m", "double", detector.cfg_device_rangeMin_m
            )
        );
        detector.cfg_device_rangeMax_m = boost::get<double>(
            XmlUtils::getAttribute(
                dsNode, "rangeMax_m", "double", detector.cfg_device_rangeMax_m
            )
        );
    }

}

void XmlSurveyLoader::loadLegs(
    tinyxml2::XMLElement *legNodes,
    std::shared_ptr<ScannerSettings> scannerSettings,
    std::shared_ptr<PlatformSettings> platformSettings,
    std::vector<std::shared_ptr<Leg>> &legs
){
    // Iterate over XML sibling leg elements
    while (legNodes != nullptr) {
        // Prepare leg loading
        glm::dvec3 origin = glm::dvec3(0, 0, 0);
        std::unordered_set<std::string> scannerFields;
        std::unordered_set<std::string> platformFields;
        shared_ptr<Leg> leg(createLegFromXML(
            legNodes, &scannerFields, &platformFields
        ));
        // Add originWaypoint shift to waypoint coordinates:
        if (leg->mPlatformSettings != nullptr /* && originWaypoint != null*/) {
            leg->mPlatformSettings->setPosition(
                leg->mPlatformSettings->getPosition() + origin
            );
        }
        // Cherry-picking of ScannerSettings
        std::unordered_set<std::string> templateFields;
        if(leg->mScannerSettings->baseTemplate != nullptr){
            templateFields = scannerTemplatesFields[
                leg->mScannerSettings->baseTemplate->id
            ];
        }
        leg->mScannerSettings = scannerSettings->cherryPick(
            leg->mScannerSettings,
            scannerFields,
            &templateFields
        );
        // Insert leg and iterate to next ony, if any
        legs.push_back(leg);
        legNodes = legNodes->NextSiblingElement("leg");
    }
}

void XmlSurveyLoader::applySceneShift(
    tinyxml2::XMLElement *surveyNode,
    bool const legNoiseDisabled,
    std::shared_ptr<Survey> survey
){
    // Prepare normal distribution if necessary
    RandomnessGenerator<double> rg(*DEFAULT_RG);
    bool legRandomOffset = surveyNode->BoolAttribute("legRandomOffset", false);
    if(legRandomOffset && !legNoiseDisabled) {
        rg.computeNormalDistribution(
          surveyNode->DoubleAttribute("legRandomOffsetMean", 0.0),
          surveyNode->DoubleAttribute("legRandomOffsetStdev", 0.1)
        );
    }
    // Apply scene shift to each leg
    for(std::shared_ptr<Leg> leg : survey->legs) {
      // Shift platform settings, if any
      if (leg->mPlatformSettings != nullptr) {
          glm::dvec3 shift = survey->scanner->platform->scene->getShift();
          glm::dvec3 platformPos = leg->mPlatformSettings->getPosition();
          leg->mPlatformSettings->setPosition(platformPos - shift);

          // If specified, move waypoint z coordinate to ground level
          if (leg->mPlatformSettings->onGround) {
              glm::dvec3 pos = leg->mPlatformSettings->getPosition();
              glm::dvec3 ground = \
                survey->scanner->platform->scene->getGroundPointAt(pos);
              leg->mPlatformSettings->setPosition(glm::dvec3(
                  pos.x, pos.y, ground.z
              ));
          }

          // Noise -> add a random offset in x,y,z to the measurements
          if (legRandomOffset && !legNoiseDisabled) {
              leg->mPlatformSettings->setPosition(
                  leg->mPlatformSettings->getPosition() +
                    glm::dvec3(rg.normalDistributionNext(),
                  rg.normalDistributionNext(),
                  rg.normalDistributionNext())
              );
          }
      }
   }
}

void XmlSurveyLoader::configureDefaultRandomnessGenerator(
    tinyxml2::XMLElement *surveyNode
){
  // Handle seed for deafult randomness generator
  if(!DEFAULT_RG_MODIFIED_FLAG){
    string seed = boost::get<string>(
        XmlUtils::getAttribute(surveyNode, "seed", "string", string("AUTO")));
    if (seed != "AUTO") {
        stringstream ss;
        ss << "survey seed: " << seed;
        logging::INFO(ss.str());
        DEFAULT_RG = std::unique_ptr<RandomnessGenerator<double>>(
            new RandomnessGenerator<double>(seed));
        DEFAULT_RG_MODIFIED_FLAG = true;
      }
  }
}
void XmlSurveyLoader::loadPlatformNoise(
    tinyxml2::XMLElement *surveyNode,
    std::shared_ptr<Platform> platform
){
    // Position noise
    tinyxml2::XMLElement *positionXNoise =
        surveyNode->FirstChildElement("positionXNoise");
    if(positionXNoise != nullptr) platform->positionXNoiseSource =
        XmlUtils::createNoiseSource(positionXNoise);
    tinyxml2::XMLElement *positionYNoise =
        surveyNode->FirstChildElement("positionYNoise");
    if(positionYNoise != nullptr) platform->positionYNoiseSource =
        XmlUtils::createNoiseSource(positionYNoise);
    tinyxml2::XMLElement *positionZNoise =
        surveyNode->FirstChildElement("positionZNoise");
    if(positionZNoise != nullptr) platform->positionZNoiseSource =
        XmlUtils::createNoiseSource(positionZNoise);

    // Attitude noise
    tinyxml2::XMLElement *attitudeXNoise =
        surveyNode->FirstChildElement("attitudeXNoise");
    if(attitudeXNoise != nullptr) platform->attitudeXNoiseSource =
        XmlUtils::createNoiseSource(attitudeXNoise);
    tinyxml2::XMLElement *attitudeYNoise =
        surveyNode->FirstChildElement("attitudeYNoise");
    if(attitudeYNoise != nullptr) platform->attitudeYNoiseSource =
        XmlUtils::createNoiseSource(attitudeYNoise);
    tinyxml2::XMLElement *attitudeZNoise =
       surveyNode->FirstChildElement("attitudeZNoise");
    if(attitudeZNoise != nullptr) platform->attitudeZNoiseSource =
        XmlUtils::createNoiseSource(attitudeZNoise);
}
