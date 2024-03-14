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
#include <platform/InterpolatedMovingPlatformEgg.h>
#include <fluxionum/ParametricLinearPiecesFunction.h>
#include <fluxionum/DiffDesignMatrixInterpolator.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>
#include <adt/exprtree/UnivarExprTreeStringFactory.h>

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
      pRoot->NextSiblingElement()->FirstChildElement("survey");
  if (surveyNodes == nullptr) {
    stringstream ss;
    ss << "XML Survey playback loader: "
       << "ERROR: No survey elements found in file " << this->xmlDocFilename;
    logging::WARN(ss.str());
    return nullptr;
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
    survey->scanner->platform,
    survey->legs
  );

  // Fit survey to legs and legs to survey
  integrateSurveyAndLegs(survey);

  // Validate survey
  validateSurvey(survey);


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

  // Update materials for all swap on repeat handlers
  for(std::shared_ptr<ScenePart> sp : survey->scanner->platform->scene->parts){
      // Ignore scene parts with no swap on repeat
      if(sp->sorh == nullptr) continue;
      // Update material for each primitive
      size_t const numPrimitives = sp->mPrimitives.size();
      std::vector<Primitive *> & baselinePrimitives =
          sp->sorh->getBaselinePrimitives();
      for(size_t i = 0 ; i < numPrimitives ; ++i){
          baselinePrimitives[i]->material = sp->mPrimitives[i]->material;
      }
  }

  // Apply scene geometry shift to platform waypoints
  applySceneShift(surveyNode, legNoiseDisabled, survey);

  // Configure default randomness generator
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
    leg->mScannerSettings = make_shared<ScannerSettings>();
  }

  // Trajectory settings
  platformSettingsNode = legNode->FirstChildElement("platformSettings");
  if(
      platformSettingsNode != nullptr &&
      XmlUtils::hasAttribute(platformSettingsNode, "trajectory")
  ){
    leg->mTrajectorySettings = createTrajectorySettingsFromXml(legNode);
  }
  else leg->mTrajectorySettings = nullptr;

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
      if(writeScene) {
          SerialSceneWrapper(sceneType, scene.get()).writeScene(sceneObjPath);
      }
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
            FWFSettings(survey->scanner->getFWFSettings())
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
    AbstractDetector &detector = *(survey->scanner->getDetector());
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
        if(XmlUtils::hasAttribute(dsNode, "distanceMeasurementError")){
            char const * expr = dsNode->Attribute("distanceMeasurementError");
            std::string exprStr(expr);
            detector.errorDistanceExpr = static_pointer_cast<
                UnivarExprTreeNode<double>
            >(
                UnivarExprTreeStringFactory<double>().makeShared(exprStr)
            );
        }
    }

}

void XmlSurveyLoader::loadLegs(
    tinyxml2::XMLElement *legNodes,
    std::shared_ptr<ScannerSettings> scannerSettings,
    std::shared_ptr<Platform> platform,
    std::vector<std::shared_ptr<Leg>> &legs
){
    // Obtain platform settings
    std::shared_ptr<PlatformSettings> platformSettings =
        platform->retrieveCurrentSettings();
    size_t xIdx = 3, yIdx = 4, zIdx = 5;
    // Obtain trajectory interpolator, if any
    std::shared_ptr<ParametricLinearPiecesFunction<double, double>>
        trajInterp = nullptr;
    std::shared_ptr<InterpolatedMovingPlatformEgg> ip = nullptr;
    try{
        ip = dynamic_pointer_cast<InterpolatedMovingPlatformEgg>(platform);
        if(ip!=nullptr){
            trajInterp =
            std::make_shared<ParametricLinearPiecesFunction<double, double>>(
            DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction(
                *(ip->ddm), *(ip->tdm)
            ));
            if(
                ip->scope ==
                InterpolatedMovingPlatform::InterpolationScope::POSITION
            ){
                xIdx = 0;       yIdx = 1;       zIdx = 2;
            }
        }
    }catch(std::exception &ex){}

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
        // Translate TrajectorySettings to simulation time
        if(leg->mTrajectorySettings != nullptr){
            if(leg->mTrajectorySettings->hasStartTime()){ // Translate tStart
                leg->mTrajectorySettings->tStart -= ip->startTime;
            }
            if(leg->mTrajectorySettings->hasEndTime()){ // Translate tEnd
                leg->mTrajectorySettings->tEnd -= ip->startTime;
            }
        }
        // Insert leg
        legs.push_back(leg);
        // Configure waypoints for interpolated legs
        if(trajInterp != nullptr){
            // Validate leg
            if(leg->mTrajectorySettings == nullptr){
                logging::ERR(
                    "XmlSurveyLoader::loadLegs failed because a leg without "
                    "trajectory settings could not be interpolated"
                );
                std::exit(-1);
            }
            // Configure start
            arma::Col<double> xStart;
            if(leg->mTrajectorySettings->hasStartTime()){
                xStart = (*trajInterp)(leg->mTrajectorySettings->tStart);
            }
            else{
                xStart = (*trajInterp)(0);
                leg->mTrajectorySettings->tStart =
                    ip->tdm->getTimeVector().front();
            }
            leg->mPlatformSettings->x = xStart[xIdx];
            leg->mPlatformSettings->y = xStart[yIdx];
            leg->mPlatformSettings->z = xStart[zIdx];
            // Configure end
            arma::Col<double> xEnd;
            if(leg->mTrajectorySettings->hasEndTime()){
                xEnd = (*trajInterp)(leg->mTrajectorySettings->tEnd);
            }
            else{
                xEnd = (*trajInterp)(arma::max(ip->tdm->getTimeVector()));
                leg->mTrajectorySettings->tEnd =
                    ip->tdm->getTimeVector().back();
            }
            // Insert stop leg
            std::shared_ptr<Leg> stopLeg = std::make_shared<Leg>(*leg);
            stopLeg->mScannerSettings = std::make_shared<ScannerSettings>(
                *leg->mScannerSettings
            );
            stopLeg->mScannerSettings->active = false;
            stopLeg->mPlatformSettings = std::make_shared<PlatformSettings>(
                *leg->mPlatformSettings
            );
            stopLeg->mPlatformSettings->x = xEnd[xIdx];
            stopLeg->mPlatformSettings->y = xEnd[yIdx];
            stopLeg->mPlatformSettings->z = xEnd[zIdx];
            stopLeg->mTrajectorySettings = make_shared<TrajectorySettings>();
            legs.push_back(stopLeg);
            // Insert teleport to start leg (after stop leg), if requested
            if(leg->mTrajectorySettings->teleportToStart){
                std::shared_ptr<Leg> startLeg = make_shared<Leg>(*stopLeg);
                startLeg->mPlatformSettings->x = leg->mPlatformSettings->x;
                startLeg->mPlatformSettings->y = leg->mPlatformSettings->y;
                startLeg->mPlatformSettings->z = leg->mPlatformSettings->z;
                startLeg->mTrajectorySettings->teleportToStart = true;
                leg->mTrajectorySettings->teleportToStart = false;
                legs.insert(legs.end()-2, startLeg);
            }
        }
        // Iterate to next leg
        legNodes = legNodes->NextSiblingElement("leg");
    }
}

void XmlSurveyLoader::applySceneShift(
    tinyxml2::XMLElement *surveyNode,
    bool const legNoiseDisabled,
    std::shared_ptr<Survey> survey
){
    // Obtain scene shift
    glm::dvec3 shift = survey->scanner->platform->scene->getShift();
    // Prepare normal distribution if necessary
    RandomnessGenerator<double> rg(*DEFAULT_RG);
    bool legRandomOffset = surveyNode->BoolAttribute("legRandomOffset", false);
    if(legRandomOffset && !legNoiseDisabled) {
        rg.computeNormalDistribution(
          surveyNode->DoubleAttribute("legRandomOffsetMean", 0.0),
          surveyNode->DoubleAttribute("legRandomOffsetStdev", 0.1)
        );
    }
    // Apply scene shift to interpolated trajectory, if any
    try{
        std::shared_ptr<InterpolatedMovingPlatformEgg> pe =
            dynamic_pointer_cast<InterpolatedMovingPlatformEgg>(
                survey->scanner->platform
            );
        if(pe != nullptr){
            size_t xIdx = 3, yIdx = 4, zIdx = 5;
            if(
                pe->scope ==
                InterpolatedMovingPlatform::InterpolationScope::POSITION
                ){
                xIdx = 0;       yIdx = 1;       zIdx = 2;
            }
            pe->tdm->addToColumn(xIdx, -shift.x);
            pe->tdm->addToColumn(yIdx, -shift.y);
            pe->tdm->addToColumn(zIdx, -shift.z);
        }
    }
    catch(...) {}
    // Apply scene shift to each leg
    for(std::shared_ptr<Leg> leg : survey->legs) {
      // Shift platform settings, if any
      if (leg->mPlatformSettings != nullptr) {
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

void XmlSurveyLoader::integrateSurveyAndLegs(std::shared_ptr<Survey> survey){
    // Obtain legs
    std::vector<std::shared_ptr<Leg>> &legs = survey->legs;

    // Handle scanAngleMax/scanAngleEffectiveMax != 1
    std::shared_ptr<PolygonMirrorBeamDeflector> pmbd =
        std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
            survey->scanner->getBeamDeflector()
        );
    if(pmbd != nullptr){
        for(std::shared_ptr<Leg> leg : legs){
            if(!leg->mScannerSettings->hasDefaultResolution()){
                std::stringstream ss;
                ss << "Scanner settings of leg " << leg->getSerialId() << " "
                   << "have been updated to consider the ratio between max "
                   << "effective scan angle and max scan angle.\n"
                   << "\tConsequently, the old scanFreq_Hz = "
                   << leg->mScannerSettings->scanFreq_Hz << " and "
                   << "headRotatePerSec_rad = "
                   << leg->mScannerSettings->headRotatePerSec_rad << " have "
                   << "been updated.\n";
                leg->mScannerSettings->fitToResolution(
                    pmbd->cfg_device_scanAngleMax_rad
                );
                ss << "\tThe new values are scanFreq_Hz = "
                   << leg->mScannerSettings->scanFreq_Hz << " and "
                   << "headRotatePerSec_rad = "
                   << leg->mScannerSettings->headRotatePerSec_rad << ".";
                logging::INFO(ss.str());
            }
        }
    }
}

void XmlSurveyLoader::validateSurvey(std::shared_ptr<Survey> survey){
    int const FORCED_EXIT_STATUS = 3; // Exit code for forced exits
    for(std::shared_ptr<Leg> leg : survey->legs){
        int const legId = leg->getSerialId();
        ScannerSettings const &ss = leg->getScannerSettings();
        std::shared_ptr<AbstractBeamDeflector> delf =
            survey->scanner->getBeamDeflector();
        // Check scanFreq_Hz is not below the minimum threshold
        if( ss.scanFreq_Hz < delf->cfg_device_scanFreqMin_Hz){
            std::stringstream s;
            s   << "Scanning frequency for leg " << legId << " is "
                << ss.scanFreq_Hz << "Hz but "
                << "min scanning frequency is set to "
                << delf->cfg_device_scanFreqMin_Hz << "Hz\n"
                << "The requested scanning frequency cannot be achieved by "
                << "this scanner.\n"
                << "Please update either the requested scanning "
                << "frequency (potentially via the requested scan resolution) "
                << "or the scanner specification.";
            logging::ERR(s.str());
            std::exit(FORCED_EXIT_STATUS);
        }
        // Check scanFreq_Hz is not above the maximum threshold
        if(
            ss.scanFreq_Hz > delf->cfg_device_scanFreqMax_Hz &&
            delf->cfg_device_scanFreqMax_Hz != 0 // 0 maxFreq means no limit
        ){
            std::stringstream s;
            s   << "Scanning frequency for leg " << legId << " is "
                << ss.scanFreq_Hz << "Hz but "
                << "max scanning frequency is set to "
                << delf->cfg_device_scanFreqMax_Hz << "Hz\n"
                << "The requested scanning frequency cannot be achieved by "
                << "this scanner.\n"
                << "Please update either the requested scanning "
                << "frequency (potentially via the requested scan resolution) "
                << "or the scanner specification.";
            logging::ERR(s.str());
            std::exit(FORCED_EXIT_STATUS);
        }

    }
}
