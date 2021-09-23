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
    bool legNoiseDisabled, bool rebuildScene
) {
  Survey *survey = new Survey();

  survey->name = boost::get<string>(
      XmlUtils::getAttribute(surveyNode, "name", "string", xmlDocFilename));
  survey->sourceFilePath = xmlDocFilePath;

  string scannerAssetLocation = boost::get<string>(
      XmlUtils::getAttribute(surveyNode, "scanner", "string", string("")));
  survey->scanner = dynamic_pointer_cast<Scanner>(
      getAssetByLocation("scanner", scannerAssetLocation));

  string platformAssetLocation = boost::get<string>(
      XmlUtils::getAttribute(surveyNode, "platform", "string", string("")));
  survey->scanner->platform = dynamic_pointer_cast<Platform>(
      getAssetByLocation("platform", platformAssetLocation));

  // FWF info
  tinyxml2::XMLElement *scannerFWFSettingsNode =
      surveyNode->FirstChildElement("FWFSettings");
  survey->scanner->applySettingsFWF(*createFWFSettingsFromXml(
      scannerFWFSettingsNode, std::make_shared<FWFSettings>(
        FWFSettings(survey->scanner->FWF_settings)
      )
  ));

  // ##################### BEGIN Read misc parameters ##################
  // Read number of runs:
  survey->numRuns =
      boost::get<int>(XmlUtils::getAttribute(surveyNode, "numRuns", "int", 1));

  // ######### BEGIN Set initial sim speed factor ##########
  double speed = boost::get<double>(XmlUtils::getAttribute(
      surveyNode, "simSpeed", "double", 1.0));
  if (speed <= 0) {
    std::stringstream ss;
    ss << "XML Survey Playback Loader: "
       << "ERROR: Sim speed can't be <= 0. Setting it to 1.";
    logging::WARN(ss.str());
    speed = 1.0;
  }

  survey->simSpeedFactor = 1.0 / speed;
  // ######### END Set initial sim speed factor ##########
  tinyxml2::XMLElement *legNodes = surveyNode->FirstChildElement("leg");
  while (legNodes != nullptr) {
    glm::dvec3 origin = glm::dvec3(0, 0, 0);
    shared_ptr<Leg> leg(createLegFromXML(legNodes));
    // Add originWaypoint shift to waypoint coordinates:
    if (leg->mPlatformSettings != NULL /* && originWaypoint != null*/) {
      leg->mPlatformSettings->setPosition(
          leg->mPlatformSettings->getPosition() + origin);
    }
    survey->legs.push_back(leg);
    legNodes = legNodes->NextSiblingElement("leg");
  }

  // ############################## END Read waypoints
  // ###########################

  // NOTE:
  // The scene is loaded as the last step, since it takes the longest time.
  // In the case that something goes wrong during the parsing
  // of the survey description, the user is noticed immediately and does not
  // need to wait until the scene is loaded, only to learn that something has
  // failed.

  // ########### BEGIN Load scene ############
  string sceneString = surveyNode->Attribute("scene");
  survey->scanner->platform->scene = loadScene(sceneString, rebuildScene);

  SpectralLibrary spectralLibrary = SpectralLibrary(
      (float)survey->scanner->getWavelength(), assetsDir + "spectra");
  spectralLibrary.readReflectances();
  spectralLibrary.setReflectances(survey->scanner->platform->scene.get());

  // ########### END Load scene ############

  // ######## BEGIN Apply scene geometry shift to platform waypoint coordinates
  // ###########
  RandomnessGenerator<double> rg(*DEFAULT_RG);
  bool legRandomOffset = surveyNode->BoolAttribute("legRandomOffset", false);
  if (legRandomOffset && !legNoiseDisabled) {
    rg.computeNormalDistribution(
        surveyNode->DoubleAttribute("legRandomOffsetMean", 0.0),
        surveyNode->DoubleAttribute("legRandomOffsetStdev", 0.1));
  }
  for (std::shared_ptr<Leg> leg : survey->legs) {
    if (leg->mPlatformSettings != NULL) {
      glm::dvec3 shift = survey->scanner->platform->scene->getShift();
      glm::dvec3 platformPos = leg->mPlatformSettings->getPosition();
      leg->mPlatformSettings->setPosition(platformPos - shift);

      // ############ BEGIN If specified, move waypoint z coordinate to ground
      // level ###############
      if (leg->mPlatformSettings->onGround) {
        glm::dvec3 pos = leg->mPlatformSettings->getPosition();
        glm::dvec3 ground = survey->scanner->platform->scene->getGroundPointAt(
            pos
        );
        leg->mPlatformSettings->setPosition(glm::dvec3(
            pos.x, pos.y, ground.z
        ));
      }
      // ############ END If specified, move waypoint z coordinate to ground
      // level ###############

      // Noise -> add a random offset in x,y,z to the measurements
      if (legRandomOffset && !legNoiseDisabled) {
        leg->mPlatformSettings->setPosition(
            leg->mPlatformSettings->getPosition() +
            glm::dvec3(rg.normalDistributionNext(), rg.normalDistributionNext(),
                       rg.normalDistributionNext()));
      }
    }
  }
  // ######## END Apply scene geometry shift to platform waypoint coordinates
  // ###########

  if (!DEFAULT_RG_MODIFIED_FLAG) {
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

  // ### BEGIN platform noise (overriding platform.xml spec if necessary) ###
  tinyxml2::XMLElement *positionXNoise =
      surveyNode->FirstChildElement("positionXNoise");
  if (positionXNoise != NULL)
    survey->scanner->platform->positionXNoiseSource =
        XmlUtils::createNoiseSource(positionXNoise);
  tinyxml2::XMLElement *positionYNoise =
      surveyNode->FirstChildElement("positionYNoise");
  if (positionYNoise != NULL)
    survey->scanner->platform->positionYNoiseSource =
        XmlUtils::createNoiseSource(positionYNoise);
  tinyxml2::XMLElement *positionZNoise =
      surveyNode->FirstChildElement("positionZNoise");
  if (positionZNoise != NULL)
    survey->scanner->platform->positionZNoiseSource =
        XmlUtils::createNoiseSource(positionZNoise);

  tinyxml2::XMLElement *attitudeXNoise =
      surveyNode->FirstChildElement("attitudeXNoise");
  if (attitudeXNoise != nullptr)
    survey->scanner->platform->attitudeXNoiseSource =
        XmlUtils::createNoiseSource(attitudeXNoise);
  tinyxml2::XMLElement *attitudeYNoise =
      surveyNode->FirstChildElement("attitudeYNoise");
  if (attitudeYNoise != nullptr)
    survey->scanner->platform->attitudeYNoiseSource =
        XmlUtils::createNoiseSource(attitudeYNoise);
  tinyxml2::XMLElement *attitudeZNoise =
      surveyNode->FirstChildElement("attitudeZNoise");
  if (attitudeZNoise != nullptr)
    survey->scanner->platform->attitudeZNoiseSource =
        XmlUtils::createNoiseSource(attitudeZNoise);
  // ### END platform noise (overriding platform.xml spec if necessary) ###

  return shared_ptr<Survey>(survey);
}

shared_ptr<Leg>
XmlSurveyLoader::createLegFromXML(tinyxml2::XMLElement *legNode) {
  Leg *leg = new Leg();

  tinyxml2::XMLElement *platformSettingsNode =
      legNode->FirstChildElement("platformSettings");

  if (platformSettingsNode != nullptr) {
    leg->mPlatformSettings =
        createPlatformSettingsFromXml(platformSettingsNode);
  }

  tinyxml2::XMLElement *scannerSettingsNode =
      legNode->FirstChildElement("scannerSettings");

  if (scannerSettingsNode != nullptr) {
    leg->mScannerSettings = createScannerSettingsFromXml(scannerSettingsNode);
  } else {
    leg->mScannerSettings = shared_ptr<ScannerSettings>(new ScannerSettings());
  }

  return shared_ptr<Leg>(leg);
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
  logging::INFO(ss.str());

  return scene;
}