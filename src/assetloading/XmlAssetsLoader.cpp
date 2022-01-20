#include "logging.hpp"
#include <HeliosException.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>

#include <glm/gtx/norm.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
namespace fs = boost::filesystem;

#include "typedef.h"
#include <XmlUtils.h>

#include "GroundVehiclePlatform.h"
#include "HelicopterPlatform.h"
#include "LinearPathPlatform.h"

#include "ConicBeamDeflector.h"
#include "FiberArrayBeamDeflector.h"
#include "FullWaveformPulseDetector.h"
#include "OscillatingMirrorBeamDeflector.h"
#include "PolygonMirrorBeamDeflector.h"
#include "RisleyBeamDeflector.h"

#include "WavefrontObjCache.h"
#include "XmlAssetsLoader.h"
#include <FileUtils.h>

#include "MathConverter.h"
#include "TimeWatcher.h"



// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //

XmlAssetsLoader::XmlAssetsLoader(std::string &filePath, std::string &assetsDir)
    : assetsDir(assetsDir)
{

  fs::path xmlFile(filePath);
  xmlDocFilename = xmlFile.filename().string();
  xmlDocFilePath = xmlFile.parent_path().string();
  logging::INFO("xmlDocFilename: " + xmlDocFilename);
  logging::INFO("xmlDocFilePath: " + xmlDocFilePath);

  tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    logging::ERR("ERROR: loading " + filePath + " failed.");
  }
}

// ***  CREATION METHODS  *** //
// ************************** //
std::shared_ptr<Asset>
XmlAssetsLoader::createAssetFromXml(
    std::string type,
    tinyxml2::XMLElement *assetNode,
    void *extraOutput
) {
  if (assetNode == nullptr) {
    logging::ERR("ERROR: Asset definition XML node is null!");
    exit(-1);
  }

  std::shared_ptr<Asset> result;
  if (type == "platform") {
    result = std::dynamic_pointer_cast<Asset>(
        createPlatformFromXml(assetNode)
    );
  } else if (type == "platformSettings") {
    result = std::dynamic_pointer_cast<Asset>(
        createPlatformSettingsFromXml(assetNode));
  } else if (type == "scanner") {
    result = std::dynamic_pointer_cast<Asset>(createScannerFromXml(assetNode));
  } else if (type == "scene") {
    result = std::dynamic_pointer_cast<Asset>(sceneLoader.createSceneFromXml(
        assetNode,
        xmlDocFilePath,
        (SerialSceneWrapper::SceneType *) extraOutput
    ));
  } else if (type == "scannerSettings") {
    result = std::dynamic_pointer_cast<Asset>(
        createScannerSettingsFromXml(assetNode));
  } else if (type == "FWFSettings") {
    result =
        std::dynamic_pointer_cast<Asset>(createFWFSettingsFromXml(assetNode));
  } else {
    logging::ERR("ERROR: Unknown asset type: " + type);
    exit(-1);
  }

  // Read "asset" properties:
  result->id = boost::get<std::string>(
      XmlUtils::getAttribute(assetNode, "id", "string", std::string("")));
  result->name = boost::get<std::string>(XmlUtils::getAttribute(
      assetNode, "name", "string", "Unnamed " + type + " asset"));

  // Store source file path for possible later XML export:
  result->sourceFilePath = xmlDocFilePath;

  return result;
}

std::shared_ptr<Platform>
XmlAssetsLoader::createPlatformFromXml(tinyxml2::XMLElement *platformNode) {
  std::shared_ptr<Platform> platform(new Platform());

  // Read platform type:
  std::string type = platformNode->Attribute("type");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  if (type.compare("groundvehicle") == 0) {
    platform = std::shared_ptr<Platform>(new GroundVehiclePlatform());
  } else if (type.compare("linearpath") == 0) {
    platform = std::shared_ptr<Platform>(new LinearPathPlatform());
  } else if (type.compare("multicopter") == 0) {
    platform = std::shared_ptr<Platform>(new HelicopterPlatform());
  } else {
    logging::INFO("No platform type specified. Using static platform.");
  }

  // Read SimplePhysicsPlatform related stuff
  SimplePhysicsPlatform *spp =
      dynamic_cast<SimplePhysicsPlatform *>(platform.get());
  if (spp != NULL) {
    spp->mCfg_drag = platformNode->DoubleAttribute("drag", 1.0);
  }

  // Read HelicopterPlatform related stuff
  HelicopterPlatform *hp = dynamic_cast<HelicopterPlatform *>(platform.get());
  if (hp != nullptr) {
    hp->cfg_speedup_magnitude =
        platformNode->DoubleAttribute("speedup_magnitude", 2.0);
    hp->cfg_slowdown_magnitude =
        platformNode->DoubleAttribute("slowdown_magnitude", 2.0);
    hp->cfg_pitch_base = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("base_pitch_deg", -5.0));
    hp->ef_xy_max = platformNode->DoubleAttribute("engine_max_force", 0.1);
    hp->cfg_pitch_speed = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("pitch_speed_deg", 85.94));
    hp->cfg_roll_speed = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("roll_speed_deg", 28.65));
    hp->cfg_yaw_speed = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("yaw_speed_deg", 85.94));
    hp->cfg_max_pitch_offset = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("pitch_offset_deg", 35.0));
    hp->cfg_max_roll_offset = MathConverter::degreesToRadians(
        platformNode->DoubleAttribute("roll_offset_deg", 25.0));
    hp->cfg_max_pitch = hp->cfg_pitch_base + hp->cfg_max_pitch_offset;
    hp->cfg_min_pitch = hp->cfg_pitch_base - hp->cfg_max_pitch_offset;
    hp->cfg_slowdown_dist_xy =
        platformNode->DoubleAttribute("slowdown_distance", 5.0);
  }

  // Read relative scanner rotation:
  try {
    tinyxml2::XMLElement *scannerMountNode =
        platformNode->FirstChildElement("scannerMount");

    // Read relative position of the scanner mount on the platform:
    platform->cfg_device_relativeMountPosition =
        XmlUtils::createVec3dFromXml(scannerMountNode, "");

    // Read relative orientation of the scanner mount on the platform:
    platform->cfg_device_relativeMountAttitude =
        XmlUtils::createRotationFromXml(scannerMountNode);
  } catch (std::exception &e) {
    logging::WARN(std::string("No scanner orientation defined.\nEXCEPTION: ") +
                  e.what());
  }

  // ########## BEGIN Read Platform noise specification ##########
  tinyxml2::XMLElement *positionXNoise =
      platformNode->FirstChildElement("positionXNoise");
  if (positionXNoise != nullptr){
      platform->positionXNoiseSource = XmlUtils::createNoiseSource(
          positionXNoise
      );
  }
  else logging::DEBUG("No default platform position X noise was specified");
  tinyxml2::XMLElement *positionYNoise =
      platformNode->FirstChildElement("positionYNoise");
  if (positionYNoise != nullptr){
      platform->positionYNoiseSource = XmlUtils::createNoiseSource(
          positionYNoise
      );
  }
  else logging::DEBUG("No default platform position Y noise was specified");
  tinyxml2::XMLElement *positionZNoise =
      platformNode->FirstChildElement("positionZNoise");
  if (positionZNoise != nullptr){
      platform->positionZNoiseSource = XmlUtils::createNoiseSource(
          positionZNoise
      );
  }
  else
    logging::DEBUG("No default platform position Z noise was specified");

  tinyxml2::XMLElement *attitudeXNoise =
      platformNode->FirstChildElement("attitudeXNoise");
  if (attitudeXNoise != nullptr){
      platform->attitudeXNoiseSource = XmlUtils::createNoiseSource(
          attitudeXNoise
      );
  }
  else
    logging::DEBUG("No default platform attitude X noise was specified");
  tinyxml2::XMLElement *attitudeYNoise =
      platformNode->FirstChildElement("attitudeYNoise");
  if (attitudeYNoise != nullptr){
      platform->attitudeYNoiseSource = XmlUtils::createNoiseSource(
          attitudeYNoise
      );
  }
  else
    logging::DEBUG("No default platform attitude Y noise was specified");
  tinyxml2::XMLElement *attitudeZNoise =
      platformNode->FirstChildElement("attitudeZNoise");
  if (attitudeZNoise != nullptr){
      platform->attitudeZNoiseSource = XmlUtils::createNoiseSource(
          attitudeZNoise
      );
  }
  else
    logging::DEBUG("No default platform attitude Z noise was specified");

  // ########## END Read Platform noise specification ##########

  return platform;
}

std::shared_ptr<PlatformSettings>
XmlAssetsLoader::createPlatformSettingsFromXml(tinyxml2::XMLElement *node) {

  std::shared_ptr<PlatformSettings> settings(new PlatformSettings());
  std::shared_ptr<PlatformSettings> template1(new PlatformSettings());
  if (node->Attribute("template") != nullptr) {
    std::shared_ptr<Asset> bla =
        getAssetByLocation("platformSettings", node->Attribute("template"));
    if (bla != nullptr) {
      template1 = std::dynamic_pointer_cast<PlatformSettings>(bla);
      // ATTENTION:
      // We need to temporarily convert the head rotation settings from radians
      // back to degrees, since degrees is the unit in which they are read from
      // the XML, and below, the template settings are used as defaults in case
      // that a value is not specified in the XML!

    } else {
      std::stringstream ss;
      ss << "XML Assets Loader: WARNING: "
         << "Platform settings template specified in line "
         << node->GetLineNum() << "\nnot found: '"
         << "Using hard-coded defaults instead.";
      logging::WARN(ss.str());
    }
  }

  // Read platform coordinates
  settings->x = boost::get<double>(XmlUtils::getAttribute(
      node, "x", "double", template1->x));
  settings->y = boost::get<double>(XmlUtils::getAttribute(
      node, "y", "double", template1->y));
  settings->z = boost::get<double>(XmlUtils::getAttribute(
      node, "z", "double", template1->z));

  // Read if platform should be put on ground, ignoring z coordinate:
  settings->onGround = boost::get<bool>(
      XmlUtils::getAttribute(node, "onGround", "bool", template1->onGround));

  // Read if platform must use stop and turn mechanics or not
  settings->stopAndTurn = boost::get<bool>(XmlUtils::getAttribute(
      node, "stopAndTurn", "bool", template1->stopAndTurn));

  // Read if platform must use smooth turn mechanics or not
  settings->smoothTurn = boost::get<bool>(XmlUtils::getAttribute(
      node, "smoothTurn", "bool", template1->smoothTurn));

  if (settings->stopAndTurn && settings->smoothTurn) {
    logging::INFO("Both stopAndTurn and smoothTurn have been set to true. "
                  "Setting stopAndTurn to false.");
    settings->stopAndTurn = false;
  }

  // Read if platform must be able to slowdown (true) or not (false)
  settings->slowdownEnabled = boost::get<bool>(XmlUtils::getAttribute(
      node, "slowdownEnabled", "bool", template1->slowdownEnabled));

  // Read platform speed:
  settings->movePerSec_m = boost::get<double>(XmlUtils::getAttribute(
      node, "movePerSec_m", "double", template1->movePerSec_m));

  if (node->FindAttribute("yawAtDeparture_deg") != nullptr) {
    settings->yawAtDepartureSpecified = true;
    settings->yawAtDeparture = MathConverter::degreesToRadians(
        boost::get<double>(XmlUtils::getAttribute(
            node, "yawAtDeparture_deg", "double", template1->yawAtDeparture
        ))
    );
  }

  return settings;
}

std::shared_ptr<Scanner>
XmlAssetsLoader::createScannerFromXml(tinyxml2::XMLElement *scannerNode) {
  // ############ BEGIN Read emitter position and orientation ############
  glm::dvec3 emitterPosition = glm::dvec3(0, 0, 0);
  Rotation emitterAttitude(glm::dvec3(1.0, 0.0, 0.0), 0.0);

  try {
    tinyxml2::XMLElement *emitterNode =
        scannerNode->FirstChildElement("beamOrigin");

    // Read relative position of the scanner mount on the platform:
    emitterPosition = XmlUtils::createVec3dFromXml(emitterNode, "");

    // Read relative orientation of the scanner mount on the platform:
    emitterAttitude = XmlUtils::createRotationFromXml(emitterNode);
  } catch (std::exception &e) {
    logging::WARN(std::string("No scanner orientation defined.\n") +
                  "EXCEPTION: " + e.what());
  }
  // ############ END Read emitter position and orientation ############

  // ########## BEGIN Read supported pulse frequencies ############
  std::string pulseFreqsString = boost::get<std::string>(
      XmlUtils::getAttribute(
          scannerNode, "pulseFreqs_Hz", "string", std::string("")
      )
  );
  std::list<int> pulseFreqs = std::list<int>();

  std::vector<std::string> freqs;
  boost::split(freqs, pulseFreqsString, boost::is_any_of(","));
  for (std::string & freq : freqs) {
    int f = boost::lexical_cast<int>(freq);
    pulseFreqs.push_back(f);
  }
  // ########## END Read supported pulse frequencies ############

  // ########### BEGIN Read all the rest #############
  double beamDiv_rad = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "beamDivergence_rad", "double", 0.0003));
  double pulseLength_ns = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "pulseLength_ns", "double", 4.0));
  std::string id = boost::get<std::string>(XmlUtils::getAttribute(
      scannerNode, "id", "string", std::string("Default")));
  double avgPower = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "averagePower_w", "double", 4.0));
  double beamQuality = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "beamQualityFactor", "double", 1.0));
  double efficiency = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "opticalEfficiency", "double", 0.99));
  double receiverDiameter = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "receiverDiameter_m", "double", 0.15));
  double visibility = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "atmosphericVisibility_km", "double", 23.0));
  int wavelength = boost::get<int>(XmlUtils::getAttribute(
      scannerNode, "wavelength_nm", "int", 1064));
  // ########### END Read all the rest #############

  std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(
      beamDiv_rad, emitterPosition, emitterAttitude, pulseFreqs, pulseLength_ns,
      id, avgPower, beamQuality, efficiency, receiverDiameter, visibility,
      wavelength, false);

  // Parse max number of returns per pulse
  scanner->maxNOR = boost::get<int>(XmlUtils::getAttribute(
      scannerNode, "maxNOR", "int", 0));

  // ########## BEGIN Default FWF_settings ##########
  std::shared_ptr<FWFSettings> settings = std::make_shared<FWFSettings>();
  settings->pulseLength_ns = pulseLength_ns;
  scanner->applySettingsFWF(*createFWFSettingsFromXml(
      scannerNode->FirstChildElement("FWFSettings"), settings));
  // ########## END Default FWF_settings ##########

  // ############################# BEGIN Configure scanner head
  // ##############################
  // ################### BEGIN Read Scan head rotation axis #############
  glm::dvec3 headRotateAxis = glm::dvec3(0, 0, 1);

  try {
    glm::dvec3 axis = XmlUtils::createVec3dFromXml(
        scannerNode->FirstChildElement("headRotateAxis"), "");
    if (glm::l2Norm(axis) > 0.1) {
      headRotateAxis = axis;
    }
  } catch (std::exception &e) {
    std::stringstream ss;
    ss << "XML Assets Loader: Failed to read child element "
       << "<headRotateAxis> of <scanner> element at line "
       << scannerNode->GetLineNum()
       << ". Using default.\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
  }
  // ############### END Read Scan head rotation axis ###############

  // Read head rotation speed:
  double headRotatePerSecMax_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          scannerNode, "headRotatePerSecMax_deg", "double", 0.0
      ))
  );

  // Configure scanner head:
  scanner->scannerHead = std::shared_ptr<ScannerHead>(
      new ScannerHead(headRotateAxis, headRotatePerSecMax_rad));

  // ############################# END Configure scanner head
  // ##############################

  // ################################## BEGIN Configure beam deflector
  // ######################################

  // ########### BEGIN Read and apply generic properties ##########
  double scanFreqMax_Hz = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "scanFreqMax_Hz", "double", 0.0));
  double scanFreqMin_Hz = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "scanFreqMin_Hz", "double", 0.0));

  double scanAngleMax_rad = MathConverter::degreesToRadians(boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "scanAngleMax_deg", "double", 0.0)));
  // ########### END Read and apply generic properties ##########

  std::string str_opticsType = scannerNode->Attribute("optics");
  std::shared_ptr<AbstractBeamDeflector> beamDeflector = NULL;

  if (str_opticsType == "oscillating") {
    int scanProduct = boost::get<int>(
        XmlUtils::getAttribute(scannerNode, "scanProduct", "int", 1000000));
    beamDeflector = std::shared_ptr<OscillatingMirrorBeamDeflector>(
        new OscillatingMirrorBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz,
                                           scanFreqMin_Hz, scanProduct));
  } else if (str_opticsType == "conic") {
    beamDeflector = std::shared_ptr<ConicBeamDeflector>(new ConicBeamDeflector(
        scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz));
  } else if (str_opticsType == "line") {
    int numFibers = boost::get<int>(XmlUtils::getAttribute(
        scannerNode, "numFibers", "int", 1));
    beamDeflector =
        std::shared_ptr<FiberArrayBeamDeflector>(new FiberArrayBeamDeflector(
            scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz, numFibers));
  } else if (str_opticsType == "rotating") {
    double scanAngleEffectiveMax_rad = MathConverter::degreesToRadians(
        boost::get<double>(XmlUtils::getAttribute(
            scannerNode, "scanAngleEffectiveMax_deg", "double", 0.0
        ))
    );
    beamDeflector = std::shared_ptr<PolygonMirrorBeamDeflector>(
        new PolygonMirrorBeamDeflector(scanFreqMax_Hz, scanFreqMin_Hz,
                                       scanAngleMax_rad,
                                       scanAngleEffectiveMax_rad));
  } else if (str_opticsType == "risley") {
    int rotorFreq_1_Hz = boost::get<int>(
        XmlUtils::getAttribute(scannerNode, "rotorFreq1_Hz", "int", 7294));
    int rotorFreq_2_Hz = boost::get<int>(
        XmlUtils::getAttribute(scannerNode, "rotorFreq2_Hz", "int", -4664));
    beamDeflector =
        std::shared_ptr<RisleyBeamDeflector>(new RisleyBeamDeflector(
            scanAngleMax_rad, (double)rotorFreq_1_Hz, (double)rotorFreq_2_Hz));
  }

  if (beamDeflector == nullptr) {
    std::stringstream ss;
    ss << "ERROR: Unknown beam deflector type: '" << str_opticsType
       << "'. Aborting.";
    logging::ERR(ss.str());
    exit(1);
  }

  scanner->beamDeflector = beamDeflector;

  // ################################## END Configure beam deflector
  // #######################################

  // ############################ BEGIN Configure detector
  // ###############################
  double rangeMin_m = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "rangeMin_m", "double", 0.0));
  double rangeMax_m = boost::get<double>(
      XmlUtils::getAttribute(
          scannerNode,
          "rangeMax_m",
          "double",
          std::numeric_limits<double>::max()
      )
  );
  double accuracy_m = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "accuracy_m", "double", 0.0));
  scanner->detector = std::make_shared<FullWaveformPulseDetector>(
      scanner, accuracy_m, rangeMin_m, rangeMax_m
  );
  // ############################ END Configure detector
  // ###############################

  return scanner;
}

std::shared_ptr<ScannerSettings>
XmlAssetsLoader::createScannerSettingsFromXml(
    tinyxml2::XMLElement *node,
    std::unordered_set<std::string> *fields
) {

  std::shared_ptr<ScannerSettings> settings(new ScannerSettings());
  std::shared_ptr<ScannerSettings> template1(new ScannerSettings());

  // Default template
  std::string const DEFAULT_TEMPLATE_ID = "DEFAULT_TEMPLATE1_HELIOSCPP";
  template1->id = DEFAULT_TEMPLATE_ID;
  template1->active = true;
  template1->headRotatePerSec_rad = 0;
  template1->headRotateStart_rad = 0;
  template1->headRotateStop_rad = 0;
  template1->pulseFreq_Hz = 0;
  template1->scanAngle_rad = 0;
  template1->verticalAngleMin_rad = 0;
  template1->verticalAngleMax_rad = 0;
  template1->scanFreq_Hz = 0;

  // Load specified template
  if (node->Attribute("template") != nullptr) {
    std::string templateId = node->Attribute("template");
    std::shared_ptr<ScannerSettings> bla = nullptr;
    if (scannerTemplates.find(templateId) == scannerTemplates.end()) {
      // If scanner template has not been loaded yet, load it
      bla = std::dynamic_pointer_cast<ScannerSettings>(
          getAssetByLocation("scannerSettings", node->Attribute("template")));
      bla->id = templateId;
      scannerTemplates.emplace(templateId, bla);
      std::unordered_set<std::string> templateFields;
      trackNonDefaultScannerSettings(
          bla, template1, DEFAULT_TEMPLATE_ID, templateFields
      );
      scannerTemplatesFields.emplace(templateId, templateFields);
    } else { // If scanner template has been loaded, then use already loaded
      bla = scannerTemplates[templateId];
    }
    if (bla != nullptr) {
      template1 = std::make_shared<ScannerSettings>(*bla);
      // ATTENTION:
      // We need to temporarily convert the head rotation settings from radians
      // back to degrees, since degrees is the unit in which they are read from
      // the XML, and below, the template settings are used as defaults in case
      // that a value is not specified in the XML!
      template1->headRotatePerSec_rad =
          MathConverter::radiansToDegrees(template1->headRotatePerSec_rad);
      template1->headRotateStart_rad =
          MathConverter::radiansToDegrees(template1->headRotateStart_rad);
      template1->headRotateStop_rad =
          MathConverter::radiansToDegrees(template1->headRotateStop_rad);
      template1->scanAngle_rad =
          MathConverter::radiansToDegrees(template1->scanAngle_rad);
      template1->verticalAngleMin_rad =
          MathConverter::radiansToDegrees(template1->scanAngle_rad);
      template1->verticalAngleMax_rad =
          MathConverter::radiansToDegrees(template1->scanAngle_rad);
    } else {
      std::stringstream ss;
      ss << "XML Assets Loader: "
         << "WARNING: Scanner settings template specified in line "
         << node->GetLineNum() << " not found: '"
         << "Using hard-coded defaults instead.";
      logging::WARN(ss.str());
    }
  }

  // Ovearload settings themselves
  settings->baseTemplate = template1;
  settings->active = boost::get<bool>(XmlUtils::getAttribute(
      node, "active", "bool", template1->active
  ));
  settings->headRotatePerSec_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          node, "headRotatePerSec_deg", "double",
          template1->headRotatePerSec_rad
      ))
  );
  settings->headRotateStart_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          node, "headRotateStart_deg", "double", template1->headRotateStart_rad
          )
      ));

  double hrStop_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          node, "headRotateStop_deg", "double", template1->headRotateStop_rad
      )));

  // Make sure that rotation stop angle is larger than rotation start angle if
  // rotation speed is positive:
  if (hrStop_rad < settings->headRotateStart_rad &&
      settings->headRotatePerSec_rad > 0) {
    logging::ERR(std::string("XML Assets Loader: Error: ") +
                 "Head Rotation Stop angle must be larger than start angle " +
                 "if rotation speed is positive!");
    exit(-1);
  }

  // Make sure that rotation stop angle is larger than rotation start angle if
  // rotation speed is negative:
  if (hrStop_rad > settings->headRotateStart_rad &&
      settings->headRotatePerSec_rad < 0) {
    logging::ERR(
        std::string("XML Assets Loader: Error: ") +
        "Head Rotation Stop angle must be smaller than start angle if " +
        "rotation speed is negative!");
    exit(-1);
  }

  settings->headRotateStop_rad = hrStop_rad;
  settings->pulseFreq_Hz = boost::get<int>(XmlUtils::getAttribute(
      node, "pulseFreq_hz", "int", template1->pulseFreq_Hz));
  settings->scanAngle_rad = MathConverter::degreesToRadians(boost::get<double>(
      XmlUtils::getAttribute(
          node, "scanAngle_deg", "double", template1->scanAngle_rad
      )
  ));
  settings->verticalAngleMin_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          node, "verticalAngleMin_deg", "double",
          template1->verticalAngleMin_rad
      ))
  );
  settings->verticalAngleMax_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(
          node, "verticalAngleMax_deg", "double",
          template1->verticalAngleMax_rad
      ))
  );
  settings->scanFreq_Hz = boost::get<double>(XmlUtils::getAttribute(
      node, "scanFreq_hz", "double", template1->scanFreq_Hz
  ));

  settings->trajectoryTimeInterval = boost::get<double>(XmlUtils::getAttribute(
    node, "trajectoryTimeInterval_s", "double", 0.0)
  );

  // Track non default values if requested
  if(fields != nullptr){
    trackNonDefaultScannerSettings(
        settings, template1, DEFAULT_TEMPLATE_ID, *fields
    );
  }

  return settings;
}

std::shared_ptr<FWFSettings> XmlAssetsLoader::createFWFSettingsFromXml(
    tinyxml2::XMLElement *node, std::shared_ptr<FWFSettings> settings) {
  if (settings == nullptr) {
    settings = std::make_shared<FWFSettings>();
  }
  // If no FWFSettings node appears on XML, default is used
  if (node != nullptr) {
    settings->binSize_ns = boost::get<double>(XmlUtils::getAttribute(
        node, "binSize_ns", "double", settings->binSize_ns
    ));
    settings->winSize_ns = settings->pulseLength_ns / 4.0; // By default
    settings->beamSampleQuality = boost::get<int>(XmlUtils::getAttribute(
        node, "beamSampleQuality", "int", settings->beamSampleQuality));
    settings->winSize_ns = boost::get<double>(XmlUtils::getAttribute(
        node, "winSize_ns", "double", settings->winSize_ns));
    settings->maxFullwaveRange_ns = boost::get<double>(XmlUtils::getAttribute(
        node, "maxFullwaveRange_ns", "double", settings->maxFullwaveRange_ns));
  }

  return settings;
}

// ***  GETTERS and SETTERS  *** //
// ***************************** //
std::shared_ptr<Asset> XmlAssetsLoader::getAssetById(
    std::string type,
    std::string id,
    void *extraOutput
) {
  std::string errorMsg = "# DEF ERR MSG #";
  try {
    tinyxml2::XMLElement *assetNodes =
        doc.FirstChild()->NextSibling()->FirstChildElement(type.c_str());

    while (assetNodes != nullptr) {
      std::string str(assetNodes->Attribute("id"));
      if (str.compare(id) == 0) {
        return createAssetFromXml(type, assetNodes, extraOutput);
      }

      assetNodes = assetNodes->NextSiblingElement(type.c_str());
    }

    std::stringstream ss;
    ss << "ERROR: " << type
       << " asset definition not found: " << this->xmlDocFilePath
       << FileUtils::pathSeparator << this->xmlDocFilename << "#" << id
       << "\nExecution aborted!";
    errorMsg = ss.str();
    logging::ERR(errorMsg);

  } catch (std::exception &e) {
    std::stringstream ss;
    ss << "ERROR: Failed to read " << type
       << " asset definition: " << this->xmlDocFilePath
       << FileUtils::pathSeparator << this->xmlDocFilename << "#" << id
       << "\nEXCEPTION: " << e.what() << "\nExecution aborted!";
    errorMsg = ss.str();
    logging::ERR(errorMsg);
  }

  throw HeliosException(errorMsg);
  return nullptr;
}

std::shared_ptr<Asset>
XmlAssetsLoader::getAssetByLocation(
    std::string type,
    std::string location,
    void *extraOutput
) {
  std::vector<std::string> vec;
  boost::split(vec, location, boost::is_any_of("#"));
  XmlAssetsLoader *loader = this;
  std::string id = vec[0].erase(vec[0].find_last_not_of("#") + 1);
  bool freeLoader = false;

  // External document location provided:
  if (vec.size() == 2) {
    loader = new XmlAssetsLoader(id, assetsDir);
    loader->sceneLoader = sceneLoader;
    id = vec[1].erase(vec[1].find_last_not_of('#') + 1);
    freeLoader = true;
  }

  std::shared_ptr<Asset> asset = loader->getAssetById(type, id, extraOutput);
  if (freeLoader)
    delete loader;
  return asset;
}

// ***  UTIL METHODS  *** //
// ********************** //
void XmlAssetsLoader::reinitLoader(){
    scannerTemplates.clear();
    scannerTemplatesFields.clear();
}

void XmlAssetsLoader::trackNonDefaultScannerSettings(
    std::shared_ptr<ScannerSettings> base,
    std::shared_ptr<ScannerSettings> ref,
    std::string const defaultTemplateId,
    std::unordered_set<std::string> &fields
){
    if(ref->id != defaultTemplateId) fields.insert("baseTemplate");
    if(base->active != ref->active) fields.insert("active");
    if(base->headRotatePerSec_rad != ref->headRotatePerSec_rad)
        fields.insert("headRotatePerSec_rad");
    if(base->headRotateStart_rad != ref->headRotateStart_rad)
        fields.insert("headRotateStart_rad");
    if(base->headRotateStop_rad != ref->headRotateStop_rad)
        fields.insert("headRotateStop_rad");
    if(base->pulseFreq_Hz != ref->pulseFreq_Hz)
        fields.insert("pulseFreq_Hz");
    if(base->scanAngle_rad != ref->scanAngle_rad)
        fields.insert("scanAngle_rad");
    if(base->verticalAngleMin_rad != ref->verticalAngleMin_rad)
        fields.insert("verticalAngleMin_rad");
    if(base->verticalAngleMax_rad != ref->verticalAngleMax_rad)
        fields.insert("verticalAngleMax_rad");
    if(base->scanFreq_Hz != ref->scanFreq_Hz)
        fields.insert("scanFreq_Hz");
    if(base->trajectoryTimeInterval != ref->trajectoryTimeInterval)
        fields.insert("trajectoryTimeInterval");
}
