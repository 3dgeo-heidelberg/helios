#include "logging.hpp"
#include <HeliosException.h>
#include <scanner/MultiScanner.h>
#include <scanner/SingleScanner.h>

#include <adt/exprtree/UnivarExprTreeNode.h>

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

#include <fluxionum/TemporalDesignMatrix.h>

#include "GroundVehiclePlatform.h"
#include "HelicopterPlatform.h"
#include "InterpolatedMovingPlatformEgg.h"
#include "LinearPathPlatform.h"

#include "ConicBeamDeflector.h"
#include "FiberArrayBeamDeflector.h"
#include "FullWaveformPulseDetector.h"
#include "OscillatingMirrorBeamDeflector.h"
#include "PolygonMirrorBeamDeflector.h"
#include "RisleyBeamDeflector.h"
#include <scanner/EvalScannerHead.h>
#include <scanner/beamDeflector/evaluable/EvalPolygonMirrorBeamDeflector.h>

#include "WavefrontObjCache.h"
#include "XmlAssetsLoader.h"
#include <FileUtils.h>

#include "MathConverter.h"
#include "TimeWatcher.h"

// ***  CONSTANTS  *** //
// ******************* //
std::string const XmlAssetsLoader::defaultScannerSettingsMsg =
  "Using scanner default value for attribute";
std::string const XmlAssetsLoader::defaultPlatformSettingsMsg =
  "Using platform default value for attribute";

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //

XmlAssetsLoader::XmlAssetsLoader(std::string& filePath,
                                 std::vector<std::string>& assetsDir)
  : assetsDir(assetsDir)
  , sceneLoader(assetsDir)
{
  auto xmlFile = locateAssetFile(filePath);

  xmlDocFilename = xmlFile.filename().string();
  xmlDocFilePath = xmlFile.parent_path().string();
  logging::INFO("xmlDocFilename: " + xmlDocFilename);
  logging::INFO("xmlDocFilePath: " + xmlDocFilePath);

  tinyxml2::XMLError result = doc.LoadFile(xmlFile.string().c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    logging::ERR("ERROR: loading " + filePath + " failed.");
  }

  makeDefaultTemplates();
}

// ***  CREATION METHODS  *** //
// ************************** //
std::shared_ptr<Asset>
XmlAssetsLoader::createAssetFromXml(std::string type,
                                    tinyxml2::XMLElement* assetNode,
                                    void* extraOutput)
{
  if (assetNode == nullptr) {
    logging::ERR("ERROR: Asset definition XML node is null!");
    exit(-1);
  }

  std::shared_ptr<Asset> result = nullptr;
  if (type == "platform") {
    result = std::dynamic_pointer_cast<Asset>(createPlatformFromXml(assetNode));
  } else if (type == "platformSettings") {
    result = std::dynamic_pointer_cast<Asset>(
      createPlatformSettingsFromXml(assetNode));
  } else if (type == "scanner") {
    result = std::dynamic_pointer_cast<Asset>(createScannerFromXml(assetNode));
  } else if (type == "scene") {
    result = std::dynamic_pointer_cast<Asset>(sceneLoader.createSceneFromXml(
      assetNode, xmlDocFilePath, (SerialSceneWrapper::SceneType*)extraOutput));
  } else if (type == "scannerSettings") {
    result =
      std::dynamic_pointer_cast<Asset>(createScannerSettingsFromXml(assetNode));
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

std::shared_ptr<Asset>
XmlAssetsLoader::createProceduralAssetFromXml(std::string const& type,
                                              std::string const& id,
                                              void* extraOutput)
{
  std::shared_ptr<Asset> result = nullptr;
  if (type == "platform") {
    result = std::dynamic_pointer_cast<Asset>(
      procedurallyCreatePlatformFromXml(type, id));
  } else {
    logging::ERR("ERROR: Unknown procedurally created asset type: " + type);
    exit(-1);
  }
  return result;
}

std::shared_ptr<Platform>
XmlAssetsLoader::createPlatformFromXml(tinyxml2::XMLElement* platformNode)
{
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
  SimplePhysicsPlatform* spp =
    dynamic_cast<SimplePhysicsPlatform*>(platform.get());
  if (spp != nullptr) {
    spp->mCfg_drag = platformNode->DoubleAttribute("drag", 1.0);
  }

  // Read HelicopterPlatform related stuff
  HelicopterPlatform* hp = dynamic_cast<HelicopterPlatform*>(platform.get());
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
    tinyxml2::XMLElement* scannerMountNode =
      platformNode->FirstChildElement("scannerMount");

    // Read relative position of the scanner mount on the platform:
    platform->cfg_device_relativeMountPosition =
      XmlUtils::createVec3dFromXml(scannerMountNode, "");

    // Read relative orientation of the scanner mount on the platform:
    platform->cfg_device_relativeMountAttitude =
      XmlUtils::createRotationFromXml(scannerMountNode);
  } catch (std::exception& e) {
    logging::WARN(std::string("No scanner orientation defined.\nEXCEPTION: ") +
                  e.what());
  }

  // ########## BEGIN Read Platform noise specification ##########
  tinyxml2::XMLElement* positionXNoise =
    platformNode->FirstChildElement("positionXNoise");
  if (positionXNoise != nullptr) {
    platform->positionXNoiseSource =
      XmlUtils::createNoiseSource(positionXNoise);
  } else
    logging::DEBUG("No default platform position X noise was specified");
  tinyxml2::XMLElement* positionYNoise =
    platformNode->FirstChildElement("positionYNoise");
  if (positionYNoise != nullptr) {
    platform->positionYNoiseSource =
      XmlUtils::createNoiseSource(positionYNoise);
  } else
    logging::DEBUG("No default platform position Y noise was specified");
  tinyxml2::XMLElement* positionZNoise =
    platformNode->FirstChildElement("positionZNoise");
  if (positionZNoise != nullptr) {
    platform->positionZNoiseSource =
      XmlUtils::createNoiseSource(positionZNoise);
  } else
    logging::DEBUG("No default platform position Z noise was specified");

  tinyxml2::XMLElement* attitudeXNoise =
    platformNode->FirstChildElement("attitudeXNoise");
  if (attitudeXNoise != nullptr) {
    platform->attitudeXNoiseSource =
      XmlUtils::createNoiseSource(attitudeXNoise);
  } else
    logging::DEBUG("No default platform attitude X noise was specified");
  tinyxml2::XMLElement* attitudeYNoise =
    platformNode->FirstChildElement("attitudeYNoise");
  if (attitudeYNoise != nullptr) {
    platform->attitudeYNoiseSource =
      XmlUtils::createNoiseSource(attitudeYNoise);
  } else
    logging::DEBUG("No default platform attitude Y noise was specified");
  tinyxml2::XMLElement* attitudeZNoise =
    platformNode->FirstChildElement("attitudeZNoise");
  if (attitudeZNoise != nullptr) {
    platform->attitudeZNoiseSource =
      XmlUtils::createNoiseSource(attitudeZNoise);
  } else
    logging::DEBUG("No default platform attitude Z noise was specified");

  // ########## END Read Platform noise specification ##########

  return platform;
}

std::shared_ptr<PlatformSettings>
XmlAssetsLoader::createPlatformSettingsFromXml(
  tinyxml2::XMLElement* node,
  std::unordered_set<std::string>* fields)
{
  // Prepare platform settings
  std::shared_ptr<PlatformSettings> settings =
    std::make_shared<PlatformSettings>();

  // Start with default template as basis
  std::shared_ptr<PlatformSettings> template1 =
    std::make_shared<PlatformSettings>(defaultPlatformTemplate.get());
  std::string const DEFAULT_TEMPLATE_ID = template1->id;

  // Load specified template
  if (node->Attribute("template") != nullptr) {
    std::string templateId = node->Attribute("template");
    std::shared_ptr<PlatformSettings> bla = nullptr;
    if (platformTemplates.find(templateId) == platformTemplates.end()) {
      // If platform template has not been loaded yet, load it
      bla = std::dynamic_pointer_cast<PlatformSettings>(
        getAssetByLocation("platformSettings", node->Attribute("template")));
      bla->id = templateId;
      platformTemplates.emplace(templateId, bla);
      std::unordered_set<std::string> templateFields;
      trackNonDefaultPlatformSettings(
        bla, template1, DEFAULT_TEMPLATE_ID, templateFields);
      platformTemplatesFields.emplace(templateId, templateFields);
    } else {
      bla = platformTemplates[templateId];
    }
    if (bla != nullptr) {
      template1 = std::make_shared<PlatformSettings>(*bla);
      // ATTENTION:
      // We need to temporarily convert the yaw at departure from radians
      // back to degrees, since degrees is the unit in which they are read from
      // the XML, and below, the template settings are used as defaults in case
      // that a value is not specified in the XML!
      template1->yawAtDeparture =
        MathConverter::radiansToDegrees(template1->yawAtDeparture);
    } else {
      std::stringstream ss;
      ss << "XML Assets Loader: WARNING: "
         << "Platform settings template specified in line "
         << node->GetLineNum() << "\nnot found: '"
         << "Using hard-coded defaults instead.";
      logging::WARN(ss.str());
    }
  }

  // Overload settings themselves
  settings->baseTemplate = template1;
  settings->x = boost::get<double>(XmlUtils::getAttribute(
    node, "x", "double", template1->x, defaultPlatformSettingsMsg));
  settings->y = boost::get<double>(XmlUtils::getAttribute(
    node, "y", "double", template1->y, defaultPlatformSettingsMsg));
  settings->z = boost::get<double>(XmlUtils::getAttribute(
    node, "z", "double", template1->z, defaultPlatformSettingsMsg));

  // Read if platform should be put on ground, ignoring z coordinate:
  settings->onGround = boost::get<bool>(XmlUtils::getAttribute(
    node, "onGround", "bool", template1->onGround, defaultPlatformSettingsMsg));

  // Read if platform must use stop and turn mechanics or not
  settings->stopAndTurn =
    boost::get<bool>(XmlUtils::getAttribute(node,
                                            "stopAndTurn",
                                            "bool",
                                            template1->stopAndTurn,
                                            defaultPlatformSettingsMsg));

  // Read if platform must use smooth turn mechanics or not
  settings->smoothTurn =
    boost::get<bool>(XmlUtils::getAttribute(node,
                                            "smoothTurn",
                                            "bool",
                                            template1->smoothTurn,
                                            defaultPlatformSettingsMsg));

  if (settings->stopAndTurn && settings->smoothTurn) {
    logging::INFO("Both stopAndTurn and smoothTurn have been set to true. "
                  "Setting stopAndTurn to false.");
    settings->stopAndTurn = false;
  }

  // Read if platform must be able to slowdown (true) or not (false)
  settings->slowdownEnabled =
    boost::get<bool>(XmlUtils::getAttribute(node,
                                            "slowdownEnabled",
                                            "bool",
                                            template1->slowdownEnabled,
                                            defaultPlatformSettingsMsg));

  // Read platform speed:
  settings->movePerSec_m =
    boost::get<double>(XmlUtils::getAttribute(node,
                                              "movePerSec_m",
                                              "double",
                                              template1->movePerSec_m,
                                              defaultPlatformSettingsMsg));

  if (node->FindAttribute("yawAtDeparture_deg") != nullptr) {
    settings->yawAtDepartureSpecified = true;
    settings->yawAtDeparture = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "yawAtDeparture_deg",
                                                "double",
                                                template1->yawAtDeparture,
                                                defaultPlatformSettingsMsg)));
  }

  // Track non default values if requested
  if (fields != nullptr) {
    trackNonDefaultPlatformSettings(
      settings, template1, DEFAULT_TEMPLATE_ID, *fields);
  }

  // Return platform settings
  return settings;
}

std::shared_ptr<Platform>
XmlAssetsLoader::procedurallyCreatePlatformFromXml(std::string const& type,
                                                   std::string const& id)
{
  if (id == "interpolated")
    return createInterpolatedMovingPlatform();
  else {
    logging::ERR("Unexpected procedurally creatable platform type: " + type);
    std::exit(-1);
  }
}
std::shared_ptr<Platform>
XmlAssetsLoader::createInterpolatedMovingPlatform()
{
  // Validate
  XmlUtils::assertDocumentForAssetLoading(
    doc,
    xmlDocFilename,
    xmlDocFilePath,
    "platform",
    "interpolated",
    "XmlAssetsLoader::createInterpolatedMovingPlatform");
  // Prepare egg building
  std::shared_ptr<InterpolatedMovingPlatformEgg> platform =
    std::make_shared<InterpolatedMovingPlatformEgg>();
  tinyxml2::XMLElement* survey =
    doc.FirstChildElement()->FirstChildElement("survey");
  tinyxml2::XMLElement* leg = survey->FirstChildElement("leg");
  std::unordered_set<std::string> trajectoryFiles;
  std::unordered_map<std::string, vector<size_t>> indices; // t, RPY, XYZ
  std::string interpDom = "position_and_attitude";
  std::string rotspec = XmlUtils::hasAttribute(survey, "rotationSpec")
                          ? survey->Attribute("rotationSpec")
                          : "ARINC 705";
  bool firstInterpDom = true;
  bool toRadians = true;
  double startTime = 0.0;
  bool syncGPSTime = false;
  if (leg == nullptr) {
    logging::ERR("XmlAssetsLoader::createInterpolatedMovingPlatform failed\n"
                 "There is no leg in the Survey XML document");
    std::exit(-1);
  }
  // Iterate over legs, to obtain indices
  while (leg != nullptr) {
    // Obtain platform settings
    tinyxml2::XMLElement* ps = leg->FirstChildElement("platformSettings");
    // Validate platform settings
    if (ps == nullptr) {
      logging::ERR("XmlAssetsLoader::createInterpolatedMovingPlatform failed\n"
                   "There is no platformSettings in the leg");
      std::exit(-1);
    }
    if (!XmlUtils::hasAttribute(ps, "trajectory")) {
      logging::ERR("XmlAssetsLoader::createInterpolatedMovingPlatform failed\n"
                   "The platformSettings element has no trajectory attribute");
      std::exit(-1);
    }
    // Get the trajectory path
    string const trajectoryPath = ps->Attribute("trajectory");
    // Check if input is given either as radians or as degrees
    toRadians &= ps->BoolAttribute("toRadians", true);
    // Check if either GPS time must be synchronized or not
    syncGPSTime |= ps->BoolAttribute("syncGPSTime", false);
    // Handle interpolation domain
    string const interpolationDomain =
      (XmlUtils::hasAttribute(ps, "interpolationDomain"))
        ? ps->Attribute("interpolationDomain")
        : "";
    if (!interpolationDomain.empty()) {
      if (firstInterpDom) {
        if (interpolationDomain != "position" &&
            interpolationDomain != "position_and_attitude") {
          std::stringstream ss;
          ss << "XmlAssetsLoader::createInterpolatedMovingPlatform "
             << "failed.\n"
             << "Unexpected interpolation domain: \"" << interpolationDomain
             << "\"";
          logging::ERR(ss.str());
          std::exit(-1);
        }
        interpDom = interpolationDomain;
      } else if (interpDom != interpolationDomain) {
        std::stringstream ss;
        ss << "XmlAssetsLoader::createInterpolatedMovingPlatform "
           << "failed.\n"
           << "Interpolation domain \"" << interpDom << "\" "
           << "was first specified.\n"
           << "But then, interpolation domain \"" << interpolationDomain
           << "\" was given.";
        logging::ERR(ss.str());
        std::exit(-1);
      }
      firstInterpDom = false;
    }
    // Handle trajectory metadata : indices
    if (XmlUtils::hasAttribute(ps, "tIndex") ||
        XmlUtils::hasAttribute(ps, "rollIndex") ||
        XmlUtils::hasAttribute(ps, "pitchIndex") ||
        XmlUtils::hasAttribute(ps, "yawIndex") ||
        XmlUtils::hasAttribute(ps, "xIndex") ||
        XmlUtils::hasAttribute(ps, "yIndex") ||
        XmlUtils::hasAttribute(ps, "zIndex")) {
      if (indices.find(trajectoryPath) != indices.end()) {
        logging::ERR("XmlAssetsLoader::createInterpolatedMovingPlatform failed."
                     "\nIndices were specified more than once for the same "
                     "trajectory:\n\"" +
                     trajectoryPath + "\"");
        std::exit(-1);
      }
      indices.emplace(
        trajectoryPath,
        vector<size_t>({ (size_t)ps->IntAttribute("tIndex", 0),
                         (size_t)ps->IntAttribute("rollIndex", 1),
                         (size_t)ps->IntAttribute("pitchIndex", 2),
                         (size_t)ps->IntAttribute("yawIndex", 3),
                         (size_t)ps->IntAttribute("xIndex", 4),
                         (size_t)ps->IntAttribute("yIndex", 5),
                         (size_t)ps->IntAttribute("zIndex", 6) }));
    }

    // Prepare next iteration
    leg = leg->NextSiblingElement("leg");
  }

  // Correct indices for POSITION scope
  if (interpDom == "position") { // Position indices
    std::unordered_map<std::string, std::vector<std::size_t>>::iterator it;
    for (it = indices.begin(); it != indices.end(); ++it) {
      std::vector<std::size_t>& inds = it->second;
      inds.erase(inds.begin() + 1, inds.begin() + 4);
    }
  }

  // Iterate over legs again, to fulfill egg
  leg = survey->FirstChildElement("leg");
  while (leg != nullptr) {
    // Obtain platform settings
    tinyxml2::XMLElement* ps = leg->FirstChildElement("platformSettings");
    // Obtain trajectory column separator
    string sep = boost::get<string>(XmlUtils::getAttribute(
      ps, "trajectory_separator", "string", string(",")));
    // Handle trajectory itself
    string const trajectoryPath = ps->Attribute("trajectory");
    bool const alreadyLoaded =
      trajectoryFiles.find(trajectoryPath) != trajectoryFiles.end();
    if (!alreadyLoaded) { // Load trajectory data if not already loaded
      if (platform->tdm == nullptr) { // First loaded trajectory
        if (indices.find(trajectoryPath) != indices.end()) { // XML inds
          fluxionum::DesignMatrix<double> dm(trajectoryPath, sep);
          dm.swapColumns(indices[trajectoryPath]);
          platform->tdm =
            std::make_shared<fluxionum::TemporalDesignMatrix<double, double>>(
              dm, 0);
        } else { // Trajectory file indices
          platform->tdm =
            std::make_shared<fluxionum::TemporalDesignMatrix<double, double>>(
              trajectoryPath, sep);
          if (interpDom == "position") { // t, x, y, z from header
            vector<unsigned long long> inds({ 0, 1, 2 });
            vector<string> const& names = platform->tdm->getColumnNames();
            for (size_t i = 0; i < names.size(); ++i) {
              if (names[i] == "x")
                inds[0] = i;
              else if (names[i] == "y")
                inds[1] = i;
              else if (names[i] == "z")
                inds[2] = i;
              else if (names[i] == "roll" || names[i] == "pitch" ||
                       names[i] == "yaw")
                ; // Ignore roll pitch yaw in position mode
              else {
                logging::ERR("XmlAssetsLoader::createInterpolated"
                             "MovingPlatform failed\n"
                             "Unexpected column \"" +
                             names[i] + "\"");
                std::exit(-1);
              }
            }
            platform->tdm->swapColumns(inds);
          } else { // t, roll, pitch, yaw, x, y, z from header
            vector<unsigned long long> inds({ 0, 1, 2, 3, 4, 5 });
            vector<string> const& names = platform->tdm->getColumnNames();
            for (size_t i = 0; i < names.size(); ++i) {
              if (names[i] == "roll")
                inds[0] = i;
              else if (names[i] == "pitch")
                inds[1] = i;
              else if (names[i] == "yaw")
                inds[2] = i;
              else if (names[i] == "x")
                inds[3] = i;
              else if (names[i] == "y")
                inds[4] = i;
              else if (names[i] == "z")
                inds[5] = i;
              else {
                logging::ERR("XmlAssetsLoader::createInterpolated"
                             "MovingPlatform failed\n"
                             "Unexpected column \"" +
                             names[i] + "\"");
                std::exit(-1);
              }
            }
            platform->tdm->swapColumns(inds);
          }
        }
        // Drop columns, if necessary
        if (interpDom == "position" && platform->tdm->getNumColumns() > 3) {
          platform->tdm->dropColumns(vector<size_t>({ 0, 1, 2 }));
        }
        // Apply slope filter, if requested
        double const slopeFilterThreshold =
          ps->DoubleAttribute("slopeFilterThreshold", 0.0);
        if (slopeFilterThreshold > 0.0) {
          platform->tdm->sortByTime();
          size_t const filteredPoints =
            platform->tdm->slopeFilter(slopeFilterThreshold);
          std::stringstream ss;
          ss << "Slope filter removed " << filteredPoints << " "
             << "points from \"" + trajectoryPath + "\"";
          logging::DEBUG(ss.str());
        }
      } else {
        // Not first loaded, so merge with previous data
        auto resolved_path = locateAssetFile(trajectoryPath).string();
        std::unique_ptr<fluxionum::TemporalDesignMatrix<double, double>> tdm;
        if (indices.find(trajectoryPath) != indices.end()) { // XML inds
          fluxionum::DesignMatrix<double> dm(resolved_path, sep);
          dm.swapColumns(indices[trajectoryPath]);
          tdm =
            std::unique_ptr<fluxionum::TemporalDesignMatrix<double, double>>(
              new fluxionum::TemporalDesignMatrix<double, double>(dm, 0));
        } else { // Trajectory file indices
          tdm =
            std::unique_ptr<fluxionum::TemporalDesignMatrix<double, double>>(
              new fluxionum::TemporalDesignMatrix<double, double>(resolved_path,
                                                                  sep));
          if (interpDom == "position") { // t, x, y, z from header
            vector<unsigned long long> inds({ 0, 1, 2 });
            vector<string> const& names = tdm->getColumnNames();
            for (size_t i = 0; i < names.size(); ++i) {
              if (names[i] == "x")
                inds[0] = i;
              else if (names[i] == "y")
                inds[1] = i;
              else if (names[i] == "z")
                inds[2] = i;
              else if (names[i] == "roll" || names[i] == "pitch" ||
                       names[i] == "yaw")
                ; // Ignore roll pitch yaw in position mode
              else {
                logging::ERR("XmlAssetsLoader::createInterpolated"
                             "MovingPlatform failed\n"
                             "Unexpected column \"" +
                             names[i] + "\"");
                std::exit(-1);
              }
            }
            tdm->swapColumns(inds);
          } else { // t, roll, pitch, yaw, x, y, z from header
            vector<unsigned long long> inds({ 0, 1, 2, 3, 4, 5 });
            vector<string> const& names = tdm->getColumnNames();
            for (size_t i = 0; i < names.size(); ++i) {
              if (names[i] == "roll")
                inds[0] = i;
              else if (names[i] == "pitch")
                inds[1] = i;
              else if (names[i] == "yaw")
                inds[2] = i;
              else if (names[i] == "x")
                inds[3] = i;
              else if (names[i] == "y")
                inds[4] = i;
              else if (names[i] == "z")
                inds[5] = i;
              else {
                logging::ERR("XmlAssetsLoader::createInterpolated"
                             "MovingPlatform failed\n"
                             "Unexpected column \"" +
                             names[i] + "\"");
                std::exit(-1);
              }
            }
            tdm->swapColumns(inds);
          }
        }
        // Drop columns, if necessary
        if (interpDom == "position" && tdm->getNumColumns() > 3) {
          tdm->dropColumns(vector<size_t>({ 0, 1, 2 }));
        }
        // Merge with previously loaded data
        platform->tdm->mergeInPlace(*tdm);
      }
      trajectoryFiles.emplace(trajectoryPath);
    }

    // Prepare next iteration
    leg = leg->NextSiblingElement("leg");
  }

  // Sort by time
  platform->tdm->sortByTime();

  // Subtract min time so time starts at t0=0, also handle sync GPS time flag
  startTime = arma::min(platform->tdm->getTimeVector());
  platform->startTime = startTime;
  platform->tdm->shiftTime(-startTime);
  platform->syncGPSTime = syncGPSTime;

  // Angle to radians, if angles are given
  if (interpDom == "position_and_attitude" && toRadians) {
    for (size_t j = 0; j < 3; ++j) {
      platform->tdm->setColumn(j, platform->tdm->getColumn(j) * PI_OVER_180);
    }
  }

  // Differentiate temporal matrix through FORWARD FINITE DIFFERENCES
  platform->ddm = platform->tdm->toDiffDesignMatrixPointer(
    fluxionum::DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES, false);

  // Configure interpolation scope
  if (interpDom == "position") {
    platform->scope = InterpolatedMovingPlatform::InterpolationScope::POSITION;
  }

  // Configure rotation specification
  if (rotspec == "CANONICAL")
    platform->rotspec = InterpolatedMovingPlatform::RotationSpec::CANONICAL;
  else if (rotspec == "ARINC 705")
    platform->rotspec = InterpolatedMovingPlatform::RotationSpec::ARINC_705;
  else {
    std::stringstream ss;
    ss << "XmlAssetsLoader::createInterpolatedMovingPlatform got an "
       << "unexpected rotation specification: \"" << rotspec << "\"";
    logging::ERR(ss.str());
    std::exit(3);
  }

  // Configure scanner mount
  // Algorithm to take ScannerMount from platforms ---
  // Check basePlatform was given
  string basePlatformLocation = boost::get<string>(
    XmlUtils::getAttribute(survey, "basePlatform", "string", string("")));
  if (basePlatformLocation.size() > 0) { // If so, ScannerMount from base plat.
    std::shared_ptr<Platform> bp = std::dynamic_pointer_cast<Platform>(
      getAssetByLocation("platform", basePlatformLocation));
    platform->cfg_device_relativeMountPosition =
      bp->cfg_device_relativeMountPosition;
    platform->cfg_device_relativeMountAttitude =
      bp->cfg_device_relativeMountAttitude;
    // Also, propagate noise sources from base platform to interpolated
    platform->positionXNoiseSource = bp->positionXNoiseSource;
    platform->positionYNoiseSource = bp->positionYNoiseSource;
    platform->positionZNoiseSource = bp->positionZNoiseSource;
    platform->attitudeXNoiseSource = bp->attitudeXNoiseSource;
    platform->attitudeYNoiseSource = bp->attitudeYNoiseSource;
    platform->attitudeZNoiseSource = bp->attitudeZNoiseSource;
  }
  // --- Algorithm to take ScannerMount from platforms

  // Algorithm to set ScannerMount from survey ---
  // Check scanner mount is specified in Survey
  tinyxml2::XMLElement* scMount = survey->FirstChildElement("scannerMount");
  if (scMount != nullptr) { // If so, assign it to interpolated platform
    platform->cfg_device_relativeMountPosition =
      XmlUtils::createVec3dFromXml(scMount, "");
    platform->cfg_device_relativeMountAttitude =
      XmlUtils::createRotationFromXml(scMount);
  }
  // --- Algorithm to set ScannerMount from survey

  // Return egg
  return platform;
}

std::shared_ptr<Scanner>
XmlAssetsLoader::createScannerFromXml(tinyxml2::XMLElement* scannerNode)
{
  // ############ BEGIN Read emitter position and orientation ############
  glm::dvec3 emitterPosition = glm::dvec3(0, 0, 0);
  Rotation emitterAttitude(glm::dvec3(1.0, 0.0, 0.0), 0.0);

  try {
    tinyxml2::XMLElement* emitterNode =
      scannerNode->FirstChildElement("beamOrigin");

    // Read relative position of the scanner mount on the platform:
    emitterPosition = XmlUtils::createVec3dFromXml(emitterNode, "");

    // Read relative orientation of the scanner mount on the platform:
    emitterAttitude = XmlUtils::createRotationFromXml(emitterNode);
  } catch (std::exception& e) {
    logging::WARN(std::string("No scanner orientation defined.\n") +
                  "EXCEPTION: " + e.what());
  }
  // ############ END Read emitter position and orientation ############

  // ########## BEGIN Read supported pulse frequencies ############
  std::string pulseFreqsString = boost::get<std::string>(XmlUtils::getAttribute(
    scannerNode, "pulseFreqs_Hz", "string", std::string("")));
  std::list<int> pulseFreqs = std::list<int>();

  std::vector<std::string> freqs;
  boost::split(freqs, pulseFreqsString, boost::is_any_of(","));
  for (std::string& freq : freqs) {
    int f = boost::lexical_cast<int>(freq);
    pulseFreqs.push_back(f);
  }
  // ########## END Read supported pulse frequencies ############
  // BEGIN : Read range error ---
  tinyxml2::XMLElement* rangeErrorNode =
    scannerNode->FirstChildElement("rangeError");
  std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr = nullptr;
  if (rangeErrorNode != nullptr) {
    rangeErrExpr = XmlUtils::createUnivarExprTree<double>(rangeErrorNode,
                                                          { { "THETA", "t" } });
  }
  // --- END : Read range error

  // ########### BEGIN Read all the rest #############
  double beamDiv_rad = boost::get<double>(XmlUtils::getAttribute(
    scannerNode, "beamDivergence_rad", "double", 0.0003));
  double pulseLength_ns = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "pulseLength_ns", "double", 4.0));
  std::string id = boost::get<std::string>(XmlUtils::getAttribute(
    scannerNode, "id", "string", std::string("Default")));
  double avgPower = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "averagePower_w", "double", 4.0));
  double beamQuality = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "beamQualityFactor", "double", 1.0));
  double efficiency = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "opticalEfficiency", "double", 0.99));
  double receiverDiameter = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "receiverDiameter_m", "double", 0.15));
  double visibility = boost::get<double>(XmlUtils::getAttribute(
    scannerNode, "atmosphericVisibility_km", "double", 23.0));
  int wavelength = boost::get<int>(
    XmlUtils::getAttribute(scannerNode, "wavelength_nm", "int", 1064));
  // ########### END Read all the rest #############

  // Check multi scanner
  tinyxml2::XMLElement* channels = scannerNode->FirstChildElement("channels");
  bool const isMultiScanner = channels != nullptr;
  std::shared_ptr<Scanner> scanner;
  if (isMultiScanner) {
    size_t nChannels = 0;
    tinyxml2::XMLElement* chan = channels->FirstChildElement("channel");
    while (chan != nullptr) {
      ++nChannels;
      chan = chan->NextSiblingElement("channel");
    }
    ScanningDevice baseScanDev(0,
                               id,
                               beamDiv_rad,
                               emitterPosition,
                               emitterAttitude,
                               pulseFreqs,
                               pulseLength_ns,
                               avgPower,
                               beamQuality,
                               efficiency,
                               receiverDiameter,
                               visibility,
                               wavelength * 1e-9,
                               rangeErrExpr);
    baseScanDev.setReceivedEnergyMin(boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "receivedEnergyMin_W", "double", 0.0001)));
    std::vector<ScanningDevice> scanDevs(nChannels, baseScanDev);
    scanner =
      std::make_shared<MultiScanner>(std::move(scanDevs), id, pulseFreqs);
    std::shared_ptr<FWFSettings> settings = std::make_shared<FWFSettings>();
    fillScanningDevicesFromChannels(
      scanner,
      scannerNode,
      channels,
      createBeamDeflectorFromXml(scannerNode),
      createDetectorFromXml(scannerNode, scanner),
      createScannerHeadFromXml(scannerNode),
      createFWFSettingsFromXml(scannerNode->FirstChildElement("FWFSettings"),
                               settings));
  } else { // Scanner as single scanner
    scanner = std::make_shared<SingleScanner>(beamDiv_rad,
                                              emitterPosition,
                                              emitterAttitude,
                                              pulseFreqs,
                                              pulseLength_ns,
                                              id,
                                              avgPower,
                                              beamQuality,
                                              efficiency,
                                              receiverDiameter,
                                              visibility,
                                              wavelength,
                                              rangeErrExpr);
    // Parse max number of returns per pulse
    scanner->setMaxNOR(
      boost::get<int>(XmlUtils::getAttribute(scannerNode, "maxNOR", "int", 0)));
    // Parse beam deflector
    scanner->setBeamDeflector(createBeamDeflectorFromXml(scannerNode));
    // Parse detector
    scanner->setDetector(createDetectorFromXml(scannerNode, scanner));
    // Parse scanner head
    scanner->setScannerHead(createScannerHeadFromXml(scannerNode));
    // Parse full waveform settings
    std::shared_ptr<FWFSettings> settings = std::make_shared<FWFSettings>();
    settings->pulseLength_ns = pulseLength_ns;
    scanner->applySettingsFWF(*createFWFSettingsFromXml(
      scannerNode->FirstChildElement("FWFSettings"), settings));
    // Parse minimum received energy threshold
    scanner->setReceivedEnergyMin(boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "receivedEnergyMin_W", "double", 0.0001)));
  }
  // Return built scanner
  return scanner;
}

std::shared_ptr<AbstractBeamDeflector>
XmlAssetsLoader::createBeamDeflectorFromXml(tinyxml2::XMLElement* scannerNode)
{
  // Prepare beam deflector variable
  std::shared_ptr<AbstractBeamDeflector> beamDeflector = nullptr;
  // Parse beam deflector
  std::string str_opticsType = scannerNode->Attribute("optics");
  double scanFreqMax_Hz = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "scanFreqMax_Hz", "double", 0.0));
  double scanFreqMin_Hz = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "scanFreqMin_Hz", "double", 0.0));
  double scanAngleMax_rad = MathConverter::degreesToRadians(boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "scanAngleMax_deg", "double", 0.0)));
  // Build beam deflector
  if (str_opticsType == "oscillating") {
    int scanProduct = boost::get<int>(
      XmlUtils::getAttribute(scannerNode, "scanProduct", "int", 1000000));
    beamDeflector = std::make_shared<OscillatingMirrorBeamDeflector>(
      scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz, scanProduct);
  } else if (str_opticsType == "conic") {
    beamDeflector = std::make_shared<ConicBeamDeflector>(
      scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz);
  } else if (str_opticsType == "line") {
    int numFibers = boost::get<int>(
      XmlUtils::getAttribute(scannerNode, "numFibers", "int", 1));
    beamDeflector = std::make_shared<FiberArrayBeamDeflector>(
      scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz, numFibers);
  } else if (str_opticsType == "rotating") {
    double scanAngleEffectiveMax_rad =
      MathConverter::degreesToRadians(boost::get<double>(XmlUtils::getAttribute(
        scannerNode, "scanAngleEffectiveMax_deg", "double", 0.0)));
    tinyxml2::XMLElement* deflectionErrorNode =
      scannerNode->FirstChildElement("deflectionError");
    if (deflectionErrorNode != nullptr) { // Build evaluable beam deflector
      if (XmlUtils::hasAttribute(deflectionErrorNode, "expr")) {
        std::shared_ptr<UnivarExprTreeNode<double>> vertAngErrExpr =
          XmlUtils::createUnivarExprTree<double>(deflectionErrorNode,
                                                 { { "THETA", "t" } });
        beamDeflector = std::make_shared<EvalPolygonMirrorBeamDeflector>(
          scanFreqMax_Hz,
          scanFreqMin_Hz,
          scanAngleMax_rad,
          scanAngleEffectiveMax_rad,
          vertAngErrExpr);
      } else {
        throw HeliosException(
          "XmlAssetsLoader::createBeamDeflectorFromXml received a "
          "deflectionError XML element with no expr attribute.");
      }
    } else { // Build classical beam deflector
      beamDeflector =
        std::make_shared<PolygonMirrorBeamDeflector>(scanFreqMax_Hz,
                                                     scanFreqMin_Hz,
                                                     scanAngleMax_rad,
                                                     scanAngleEffectiveMax_rad);
    }
  } else if (str_opticsType == "risley") {
    double rotorFreq_1_Hz = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "rotorFreq1_Hz", "double", 0.));
    double rotorFreq_2_Hz = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "rotorFreq2_Hz", "double", -77.73333333));
    double rotorFreq_3_Hz = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "rotorFreq3_Hz", "double", 121.56666666666666));

    double prism1_angle_deg = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism1_angle_deg", "double", 0.0));
    double prism2_angle_deg = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism2_angle_deg", "double", 18.0));
    double prism3_angle_deg = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism3_angle_deg", "double", 18.0));

    double prism1_thickness = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "prism1_thickness_mm", "double", 0.0));
    double prism2_thickness = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "prism2_thickness_mm", "double", 1.0));
    double prism3_thickness = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "prism3_thickness_mm", "double", 1.0));

    double prism1_radius = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism1_radius_mm", "double", 10.0));
    double prism2_radius = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism2_radius_mm", "double", 10.0));
    double prism3_radius = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "prism3_radius_mm", "double", 10.0));

    double distance_1_2 = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "distance_prism1_2_mm", "double", 0.0));
    double distance_2_3 = boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "distance_prism2_3_mm", "double", 2.0));

    double refr_prism1 = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "refrIndex_prism1", "double", 1.0));
    double refr_prism2 = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "refrIndex_prism2", "double", 1.51));
    double refr_prism3 = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "refrIndex_prism3", "double", 1.51));
    double refr_air = boost::get<double>(
      XmlUtils::getAttribute(scannerNode, "refrIndex_air", "double", 1.0));

    beamDeflector = std::make_shared<RisleyBeamDeflector>(scanAngleMax_rad,
                                                          rotorFreq_1_Hz,
                                                          rotorFreq_2_Hz,
                                                          rotorFreq_3_Hz,
                                                          prism1_angle_deg,
                                                          prism2_angle_deg,
                                                          prism3_angle_deg,
                                                          prism1_thickness,
                                                          prism2_thickness,
                                                          prism3_thickness,
                                                          prism1_radius,
                                                          prism2_radius,
                                                          prism3_radius,
                                                          distance_1_2,
                                                          distance_2_3,
                                                          refr_prism1,
                                                          refr_prism2,
                                                          refr_prism3,
                                                          refr_air);
  }

  if (beamDeflector == nullptr) {
    std::stringstream ss;
    ss << "ERROR: Unknown beam deflector type: '" << str_opticsType
       << "'. Aborting.";
    logging::ERR(ss.str());
    exit(1);
  }
  // Return built beam deflector
  return beamDeflector;
}

std::shared_ptr<AbstractDetector>
XmlAssetsLoader::createDetectorFromXml(tinyxml2::XMLElement* scannerNode,
                                       std::shared_ptr<Scanner> scanner)
{
  double const rangeMin_m = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "rangeMin_m", "double", 0.0));
  double const rangeMax_m = boost::get<double>(XmlUtils::getAttribute(
    scannerNode, "rangeMax_m", "double", std::numeric_limits<double>::max()));
  double const accuracy_m = boost::get<double>(
    XmlUtils::getAttribute(scannerNode, "accuracy_m", "double", 0.0));
  return std::make_shared<FullWaveformPulseDetector>(
    scanner, accuracy_m, rangeMin_m, rangeMax_m);
}

std::shared_ptr<ScannerHead>
XmlAssetsLoader::createScannerHeadFromXml(tinyxml2::XMLElement* scannerNode)
{
  glm::dvec3 headRotateAxis = glm::dvec3(0, 0, 1);
  try {
    glm::dvec3 axis = XmlUtils::createVec3dFromXml(
      scannerNode->FirstChildElement("headRotateAxis"), "");
    if (glm::l2Norm(axis) > 0.1) {
      headRotateAxis = axis;
    }
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "XML Assets Loader: Failed to read child element "
       << "<headRotateAxis> of <scanner> element at line "
       << scannerNode->GetLineNum()
       << ". Using default.\nEXCEPTION: " << e.what();
    logging::WARN(ss.str());
  }
  double headRotatePerSecMax_rad =
    MathConverter::degreesToRadians(boost::get<double>(XmlUtils::getAttribute(
      scannerNode, "headRotatePerSecMax_deg", "double", 0.0)));
  tinyxml2::XMLElement* headErrorNode =
    scannerNode->FirstChildElement("headError");
  if (headErrorNode != nullptr) { // Build evaluable scanner head
    if (std::string(scannerNode->Attribute("optics")) != "rotating") {
      throw HeliosException(
        "Error at XmlAssetsLoader::createScannerHeadFromXml because "
        "<headError ...> is ONLY supported for rotating optics "
        "(PolygonMirrorBeamDeflector)");
    }
    if (XmlUtils::hasAttribute(headErrorNode, "expr")) {
      std::shared_ptr<UnivarExprTreeNode<double>> horizAngErrExpr =
        XmlUtils::createUnivarExprTree<double>(headErrorNode,
                                               { { "THETA", "t" } });
      double const zeroSinThreshold_deg =
        boost::get<double>(XmlUtils::getAttribute(
          headErrorNode, "zeroSinThreshold_deg", "double", 0));
      return std::make_shared<EvalScannerHead>(
        headRotateAxis,
        headRotatePerSecMax_rad,
        horizAngErrExpr,
        MathConverter::degreesToRadians(zeroSinThreshold_deg));
    } else {
      throw HeliosException(
        "XmlAssetsLoader::createScannerHeadFromXml received a "
        "headError XML element with no expr attribute");
    }
  }
  return std::make_shared<ScannerHead>(headRotateAxis, headRotatePerSecMax_rad);
}

std::shared_ptr<ScannerSettings>
XmlAssetsLoader::createScannerSettingsFromXml(
  tinyxml2::XMLElement* node,
  std::unordered_set<std::string>* fields)
{
  // Prepare scanner settings
  std::shared_ptr<ScannerSettings> settings =
    std::make_shared<ScannerSettings>();

  // Start with default template as basis
  std::shared_ptr<ScannerSettings> template1 =
    std::make_shared<ScannerSettings>(defaultScannerTemplate.get());
  std::string const DEFAULT_TEMPLATE_ID = template1->id;

  // Load specified template
  if (XmlUtils::hasAttribute(node, "template")) {
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
        bla, template1, DEFAULT_TEMPLATE_ID, templateFields);
      scannerTemplatesFields.emplace(templateId, templateFields);
    } else { // If scanner template has been loaded, then use already loaded
      bla = scannerTemplates[templateId];
    }
    if (bla != nullptr) {
      template1 = std::make_shared<ScannerSettings>(*bla);
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
    node, "active", "bool", template1->active, defaultScannerSettingsMsg));
  if (XmlUtils::hasAttribute(node, "headRotatePerSec_deg")) {
    settings->headRotatePerSec_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "headRotatePerSec_deg",
                                                "double",
                                                0.0,
                                                defaultScannerSettingsMsg)));
  } else
    settings->headRotatePerSec_rad = template1->headRotatePerSec_rad;
  if (XmlUtils::hasAttribute(node, "headRotateStart_deg")) {
    settings->headRotateStart_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "headRotateStart_deg",
                                                "double",
                                                0.0,
                                                defaultScannerSettingsMsg)));
  } else
    settings->headRotateStart_rad = template1->headRotateStart_rad;

  if (XmlUtils::hasAttribute(node, "headRotateStop_deg")) {
    double hrStop_rad =
      MathConverter::degreesToRadians(boost::get<double>(XmlUtils::getAttribute(
        node, "headRotateStop_deg", "double", 0.0, defaultScannerSettingsMsg)));

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
  } else
    settings->headRotateStop_rad = template1->headRotateStop_rad;
  settings->pulseFreq_Hz =
    boost::get<int>(XmlUtils::getAttribute(node,
                                           "pulseFreq_hz",
                                           "int",
                                           template1->pulseFreq_Hz,
                                           defaultScannerSettingsMsg));
  if (XmlUtils::hasAttribute(node, "scanAngle_deg")) {
    settings->scanAngle_rad =
      MathConverter::degreesToRadians(boost::get<double>(XmlUtils::getAttribute(
        node, "scanAngle_deg", "double", 0.0, defaultScannerSettingsMsg)));
  } else
    settings->scanAngle_rad = template1->scanAngle_rad;
  if (XmlUtils::hasAttribute(node, "verticalAngleMin_deg")) {
    settings->verticalAngleMin_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "verticalAngleMin_deg",
                                                "double",
                                                NAN,
                                                defaultScannerSettingsMsg)));
  } else
    settings->verticalAngleMin_rad = template1->verticalAngleMin_rad;
  if (XmlUtils::hasAttribute(node, "verticalAngleMax_deg")) {
    settings->verticalAngleMax_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "verticalAngleMax_deg",
                                                "double",
                                                NAN,
                                                defaultScannerSettingsMsg)));
  } else
    settings->verticalAngleMax_rad = template1->verticalAngleMax_rad;
  settings->scanFreq_Hz =
    boost::get<double>(XmlUtils::getAttribute(node,
                                              "scanFreq_hz",
                                              "double",
                                              template1->scanFreq_Hz,
                                              defaultScannerSettingsMsg));

  settings->beamDivAngle =
    boost::get<double>(XmlUtils::getAttribute(node,
                                              "beamDivergence_rad",
                                              "double",
                                              template1->beamDivAngle,
                                              defaultScannerSettingsMsg));

  settings->trajectoryTimeInterval =
    boost::get<double>(XmlUtils::getAttribute(node,
                                              "trajectoryTimeInterval_s",
                                              "double",
                                              template1->trajectoryTimeInterval,
                                              defaultScannerSettingsMsg));

  // Parse alternative spec. based on vertical and horizontal resolutions
  if (XmlUtils::hasAttribute(node, "verticalResolution_deg")) {
    settings->verticalResolution_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "verticalResolution_deg",
                                                "double",
                                                0.0,
                                                defaultScannerSettingsMsg)));
  } else
    settings->verticalResolution_rad = template1->verticalResolution_rad;
  if (XmlUtils::hasAttribute(node, "horizontalResolution_deg")) {
    settings->horizontalResolution_rad = MathConverter::degreesToRadians(
      boost::get<double>(XmlUtils::getAttribute(node,
                                                "horizontalResolution_deg",
                                                "double",
                                                0.0,
                                                defaultScannerSettingsMsg)));
  } else
    settings->horizontalResolution_rad = template1->horizontalResolution_rad;

  // Track non default values if requested
  if (fields != nullptr) {
    trackNonDefaultScannerSettings(
      settings, template1, DEFAULT_TEMPLATE_ID, *fields);
  }

  // Return scanner settings
  return settings;
}

std::shared_ptr<FWFSettings>
XmlAssetsLoader::createFWFSettingsFromXml(tinyxml2::XMLElement* node,
                                          std::shared_ptr<FWFSettings> settings)
{
  // If no FWFSettings node appears on XML, default is used
  if (settings == nullptr) {
    settings = std::make_shared<FWFSettings>();
  }
  // Given FWFSettings
  if (node != nullptr) {
    settings->binSize_ns = boost::get<double>(XmlUtils::getAttribute(
      node, "binSize_ns", "double", settings->binSize_ns));
    settings->winSize_ns = settings->pulseLength_ns / 4.0; // By default
    settings->beamSampleQuality = boost::get<int>(XmlUtils::getAttribute(
      node, "beamSampleQuality", "int", settings->beamSampleQuality));
    settings->winSize_ns = boost::get<double>(XmlUtils::getAttribute(
      node, "winSize_ns", "double", settings->winSize_ns));
    settings->maxFullwaveRange_ns = boost::get<double>(XmlUtils::getAttribute(
      node, "maxFullwaveRange_ns", "double", settings->maxFullwaveRange_ns));
    settings->apertureDiameter = boost::get<double>(XmlUtils::getAttribute(
      node, "apertureDiameter_m", "double", settings->apertureDiameter));
  }

  return settings;
}

std::shared_ptr<TrajectorySettings>
XmlAssetsLoader::createTrajectorySettingsFromXml(
  tinyxml2::XMLElement* legNode,
  std::shared_ptr<TrajectorySettings> settings)
{
  // Create settings if not given
  if (settings == nullptr) {
    settings = std::make_shared<TrajectorySettings>();
  }

  // Load start and end times from XML, if any
  tinyxml2::XMLElement* ps = legNode->FirstChildElement("platformSettings");
  if (ps != nullptr) { // If PlatformSettings were specified
    settings->tStart = ps->DoubleAttribute("tStart", settings->tStart);
    settings->tEnd = ps->DoubleAttribute("tEnd", settings->tEnd);
    settings->teleportToStart =
      ps->BoolAttribute("teleportToStart", settings->teleportToStart);
  }

  // Return loaded settings
  return settings;
}

void
XmlAssetsLoader::fillScanningDevicesFromChannels(
  std::shared_ptr<Scanner> scanner,
  tinyxml2::XMLElement* scannerNode,
  tinyxml2::XMLElement* channels,
  std::shared_ptr<AbstractBeamDeflector> deflec,
  std::shared_ptr<AbstractDetector> detec,
  std::shared_ptr<ScannerHead> scanHead,
  std::shared_ptr<FWFSettings> fwfSettings)
{
  tinyxml2::XMLElement* chan = channels->FirstChildElement("channel");
  tinyxml2::XMLElement* elem;
  size_t idx = 0;           // Device/channel index
  while (chan != nullptr) { // Update i-th device with i-th channel
    // Set id
    scanner->setDeviceIndex(idx, idx);
    std::string const deviceId = boost::get<std::string>(XmlUtils::getAttribute(
      chan, "id", "string", std::to_string(idx), "DeviceID"));
    scanner->setDeviceId(deviceId, idx);
    // Check beam update
    elem = chan->FirstChildElement("beamOrigin");
    if (elem != nullptr) {
      // Check beam origin
      try {
        scanner->setHeadRelativeEmitterPosition(
          XmlUtils::createVec3dFromXml(elem, ""), idx);
      } catch (HeliosException& hex) {
        std::stringstream ss;
        ss << "Failed to find beamOrigin (x, y, z) in channel " << idx
           << ".\nEXCEPTION:\n"
           << hex.what();
        logging::WARN(ss.str());
      }
      // Check beam attitude
      try {
        scanner->setHeadRelativeEmitterAttitude(
          XmlUtils::createRotationFromXml(elem), idx);
      } catch (HeliosException& hex) {
        std::stringstream ss;
        ss << "Failed to find beamAttitude (q0, q1, q2, q3) in "
           << "channel " << idx << ".\nEXCEPTION:\n"
           << hex.what();
        logging::WARN(ss.str());
      }
    }
    // Check full waveform settings update
    elem = chan->FirstChildElement("FWFSettings");
    if (elem != nullptr) {
      std::shared_ptr<FWFSettings> fwfs =
        std::make_shared<FWFSettings>(*fwfSettings);
      fwfs = createFWFSettingsFromXml(elem, fwfs);
      scanner->applySettingsFWF(*fwfs, idx);
    } else {
      scanner->applySettingsFWF(*fwfSettings, idx);
    }
    // Check beam deflector update
    bool updateDeflector = true;
    if (XmlUtils::hasAttribute(chan, "optics")) {
      std::string optics = chan->Attribute("optics");
      bool deflectorsMatch =
        (optics == "oscillating" &&
         std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(deflec) !=
           nullptr) ||
        (optics == "conic" &&
         std::dynamic_pointer_cast<ConicBeamDeflector>(deflec) != nullptr) ||
        (optics == "line" && std::dynamic_pointer_cast<FiberArrayBeamDeflector>(
                               deflec) != nullptr) ||
        (optics == "rotating" &&
         std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(deflec) !=
           nullptr) ||
        (optics == "risley" &&
         std::dynamic_pointer_cast<RisleyBeamDeflector>(deflec) != nullptr);
      if (!deflectorsMatch) { // Assign new beam deflector, dont update
        scanner->setBeamDeflector(createBeamDeflectorFromXml(chan), idx);
        updateDeflector = false;
      }
    }
    if (updateDeflector) {
      std::string optics = scannerNode->Attribute("optics");
      scanner->setBeamDeflector(deflec->clone(), idx);
      std::shared_ptr<AbstractBeamDeflector> _deflec =
        scanner->getBeamDeflector(idx);
      // Common beam deflector updates
      _deflec->cfg_device_scanFreqMin_Hz = boost::get<double>(
        XmlUtils::getAttribute(chan,
                               "scanFreqMin_Hz",
                               "double",
                               _deflec->cfg_device_scanFreqMin_Hz));
      _deflec->cfg_device_scanFreqMax_Hz = boost::get<double>(
        XmlUtils::getAttribute(chan,
                               "scanFreqMax_Hz",
                               "double",
                               _deflec->cfg_device_scanFreqMax_Hz));
      if (XmlUtils::hasAttribute(chan, "scanAngleMax_deg")) {
        _deflec->cfg_device_scanAngleMax_rad = boost::get<double>(
          XmlUtils::getAttribute(chan, "scanAngleMax_deg", "double", 0.0));
      }
      // Oscillating mirror beam deflector updates
      if (optics == "oscillating") {
        std::shared_ptr<OscillatingMirrorBeamDeflector> ombd =
          std::static_pointer_cast<OscillatingMirrorBeamDeflector>(_deflec);
        ombd->cfg_device_scanProduct = boost::get<int>(XmlUtils::getAttribute(
          chan, "scanProduct", "int", ombd->cfg_device_scanProduct));
      }
      // Conic beam deflector updates (NONE)
      // Fiber array beam deflector updates
      if (optics == "line") {
        std::shared_ptr<FiberArrayBeamDeflector> fabd =
          std::static_pointer_cast<FiberArrayBeamDeflector>(_deflec);
        fabd->setNumFibers(boost::get<int>(XmlUtils::getAttribute(
          scannerNode, "numFibers", "int", fabd->getNumFibers())));
      }
      // Polygon mirror beam deflector updates
      if (optics == "rotating") {
        std::shared_ptr<PolygonMirrorBeamDeflector> pmbd =
          std::static_pointer_cast<PolygonMirrorBeamDeflector>(_deflec);
        pmbd->cfg_device_scanAngleMax_rad =
          MathConverter::degreesToRadians(boost::get<double>(
            XmlUtils::getAttribute(chan,
                                   "scanAngleEffectiveMax_deg",
                                   "double",
                                   MathConverter::radiansToDegrees(
                                     pmbd->cfg_device_scanAngleMax_rad))));
      }
      // Risley beam deflector updates
      if (optics == "risley") {
        std::shared_ptr<RisleyBeamDeflector> rbd =
          std::static_pointer_cast<RisleyBeamDeflector>(_deflec);
        if (XmlUtils::hasAttribute(chan, "rotorFreq1_Hz")) {
          rbd->rotorSpeed_rad_1 = (boost::get<double>(XmlUtils::getAttribute(
                                    chan, "rotorFreq1_Hz", "double", 0.0))) /
                                  PI_2;
        }
        if (XmlUtils::hasAttribute(chan, "rotorFreq2_Hz")) {
          rbd->rotorSpeed_rad_2 = (boost::get<double>(XmlUtils::getAttribute(
                                    chan, "rotorFreq2_Hz", "double", 7294.0))) /
                                  PI_2;
        }
        if (XmlUtils::hasAttribute(chan, "rotorFreq3_Hz")) {
          rbd->rotorSpeed_rad_3 =
            (boost::get<double>(XmlUtils::getAttribute(
              chan, "rotorFreq3_Hz", "double", -4664.0))) /
            PI_2;
        }
        if (XmlUtils::hasAttribute(chan, "prism1_angle_deg")) {
          rbd->prism1_angle_rad = MathConverter::radiansToDegrees(
            (boost::get<double>(XmlUtils::getAttribute(
              chan, "prism1_angle_deg", "double", 0.0))));
        }
        if (XmlUtils::hasAttribute(chan, "prism2_angle_deg")) {
          rbd->prism2_angle_rad = MathConverter::radiansToDegrees(
            (boost::get<double>(XmlUtils::getAttribute(
              chan, "prism2_angle_deg", "double", 0.0))));
        }
        if (XmlUtils::hasAttribute(chan, "prism3_angle_deg")) {
          rbd->prism3_angle_rad = MathConverter::radiansToDegrees(
            (boost::get<double>(XmlUtils::getAttribute(
              chan, "prism3_angle_deg", "double", 0.0))));
        }
        if (XmlUtils::hasAttribute(chan, "prism1_thickness")) {
          rbd->prism1_thickness_base = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism1_thickness", "double", 0.0));
        }
        if (XmlUtils::hasAttribute(chan, "prism2_thickness")) {
          rbd->prism2_thickness_base = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism2_thickness", "double", 4.0));
        }
        if (XmlUtils::hasAttribute(chan, "prism3_thickness")) {
          rbd->prism3_thickness_base = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism3_thickness", "double", 4.0));
        }
        if (XmlUtils::hasAttribute(chan, "prism1_radius")) {
          rbd->prism1_radius = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism1_radius", "double", 0.0));
        }
        if (XmlUtils::hasAttribute(chan, "prism2_radius")) {
          rbd->prism2_radius = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism2_radius", "double", 2.0));
        }
        if (XmlUtils::hasAttribute(chan, "prism3_radius")) {
          rbd->prism3_radius = boost::get<double>(
            XmlUtils::getAttribute(chan, "prism3_radius", "double", 2.0));
        }
        if (XmlUtils::hasAttribute(chan, "distance_prism1_2")) {
          rbd->distance_prism1_2 = boost::get<double>(
            XmlUtils::getAttribute(chan, "distance_prism1_2", "double", 0.0));
        }
        if (XmlUtils::hasAttribute(chan, "distance_prism2_3")) {
          rbd->distance_prism2_3 = boost::get<double>(
            XmlUtils::getAttribute(chan, "distance_prism2_3", "double", 3.0));
        }
        if (XmlUtils::hasAttribute(chan, "refrIndex_prism1")) {
          rbd->refrIndex_prism1 = boost::get<double>(
            XmlUtils::getAttribute(chan, "refrIndex_prism1", "double", 1.0));
        }
        if (XmlUtils::hasAttribute(chan, "refrIndex_prism2")) {
          rbd->refrIndex_prism2 = boost::get<double>(
            XmlUtils::getAttribute(chan, "refrIndex_prism2", "double", 1.5));
        }
        if (XmlUtils::hasAttribute(chan, "refrIndex_prism3")) {
          rbd->refrIndex_prism3 = boost::get<double>(
            XmlUtils::getAttribute(chan, "refrIndex_prism3", "double", 1.5));
        }
      }
    }
  }
  // Check detector related attributes
  std::shared_ptr<AbstractDetector> _detec = detec->clone();
  _detec->cfg_device_accuracy_m = boost::get<double>(XmlUtils::getAttribute(
    chan, "accuracy_m", "double", _detec->cfg_device_accuracy_m));
  _detec->cfg_device_rangeMin_m = boost::get<double>(XmlUtils::getAttribute(
    chan, "rangeMin_m", "double", _detec->cfg_device_rangeMin_m));
  _detec->cfg_device_rangeMax_m = boost::get<double>(XmlUtils::getAttribute(
    chan, "rangeMax_m", "double", _detec->cfg_device_rangeMax_m));
  scanner->setDetector(_detec, idx);
  // Check scanner head related attributes
  std::shared_ptr<ScannerHead> _scanHead =
    std::make_shared<ScannerHead>(*scanHead);
  _scanHead->setRotatePerSecMax(
    MathConverter::degreesToRadians(boost::get<double>(XmlUtils::getAttribute(
      chan,
      "headRotatePerSecMax_deg",
      "double",
      MathConverter::radiansToDegrees(_scanHead->getRotatePerSecMax())))));
  tinyxml2::XMLElement* hraNode = chan->FirstChildElement("headRotateAxis");
  if (hraNode != nullptr) {
    glm::dvec3 shra = XmlUtils::createVec3dFromXml(
      chan->FirstChildElement("headRotateAxis"), "");
    _scanHead->cfg_device_rotateAxis = shra;
  }
  scanner->setScannerHead(_scanHead, idx);
  // Check general attributes
  scanner->setBeamDivergence(
    boost::get<double>(XmlUtils::getAttribute(
      chan, "beamDivergence_rad", "double", scanner->getBeamDivergence(idx))),
    idx);
  scanner->setPulseLength_ns(
    boost::get<double>(XmlUtils::getAttribute(
      chan, "pulseLength_ns", "double", scanner->getPulseLength_ns(idx))),
    idx);
  if (XmlUtils::hasAttribute(chan, "wavelength_nm")) {
    scanner->setWavelength(boost::get<int>(XmlUtils::getAttribute(
                             chan, "wavelength_nm", "int", 1064)) *
                             1e-9,
                           idx);
  }
  scanner->setMaxNOR(
    boost::get<int>(XmlUtils::getAttribute(chan, "maxNOR", "int", 0)), idx);
  scanner->setReceivedEnergyMin(boost::get<double>(XmlUtils::getAttribute(
                                  chan,
                                  "receivedEnergyMin_W",
                                  "double",
                                  scanner->getReceivedEnergyMin(idx))),
                                idx);
  // Next channel, if any
  chan = chan->NextSiblingElement("channel");
  ++idx;
}

// ***  GETTERS and SETTERS  *** //
// ***************************** //
std::shared_ptr<Asset>
XmlAssetsLoader::getAssetById(std::string type,
                              std::string id,
                              void* extraOutput)
{
  std::string errorMsg = "# DEF ERR MSG #";
  XmlUtils::assertDocumentForAssetLoading(doc,
                                          xmlDocFilename,
                                          xmlDocFilePath,
                                          type,
                                          id,
                                          "XmlAssetsLoader::getAssetById");
  try {
    tinyxml2::XMLElement* assetNodes =
      doc.FirstChild()->NextSiblingElement()->FirstChildElement(type.c_str());

    if (isProceduralAsset(type, id)) { // Generate procedural assets
      return createProceduralAssetFromXml(type, id, extraOutput);
    }

    while (assetNodes != nullptr) { // Load standard assets
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

  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "ERROR: Failed to read " << type
       << " asset definition: " << this->xmlDocFilePath
       << FileUtils::pathSeparator << this->xmlDocFilename << "#" << id
       << "\nEXCEPTION: " << e.what() << "\nExecution aborted!";
    errorMsg = ss.str();
    logging::ERR(errorMsg);
  }

  throw HeliosException(errorMsg);
}

std::shared_ptr<Asset>
XmlAssetsLoader::getAssetByLocation(std::string type,
                                    std::string location,
                                    void* extraOutput)
{
  std::vector<std::string> vec;
  boost::split(vec, location, boost::is_any_of("#"));
  XmlAssetsLoader* loader = this;
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
void
XmlAssetsLoader::reinitLoader()
{
  scannerTemplates.clear();
  scannerTemplatesFields.clear();
}

void
XmlAssetsLoader::makeDefaultTemplates()
{
  // Make default scanner settings template
  defaultScannerTemplate = std::make_shared<ScannerSettings>();
  defaultScannerTemplate->id = "DEFAULT_TEMPLATE1_HELIOSCPP";
  defaultScannerTemplate->active = true;
  defaultScannerTemplate->headRotatePerSec_rad = 0;
  defaultScannerTemplate->headRotateStart_rad = 0;
  defaultScannerTemplate->headRotateStop_rad = 0;
  defaultScannerTemplate->pulseFreq_Hz = 0;
  defaultScannerTemplate->scanAngle_rad = 0;
  defaultScannerTemplate->verticalAngleMin_rad = NAN;
  defaultScannerTemplate->verticalAngleMax_rad = NAN;
  defaultScannerTemplate->scanFreq_Hz = 0;

  // Make default platform settings template
  defaultPlatformTemplate = std::make_shared<PlatformSettings>();
  defaultPlatformTemplate->id = "DEFAULT_TEMPLATE1_HELIOSCPP";
  defaultPlatformTemplate->setPosition(0, 0, 0);
  defaultPlatformTemplate->yawAtDepartureSpecified = false;
  defaultPlatformTemplate->yawAtDeparture = 0.0;
  defaultPlatformTemplate->onGround = false;
  defaultPlatformTemplate->stopAndTurn = true;
  defaultPlatformTemplate->smoothTurn = false;
  defaultPlatformTemplate->slowdownEnabled = true;
  defaultPlatformTemplate->movePerSec_m = 70;
}

void
XmlAssetsLoader::trackNonDefaultScannerSettings(
  std::shared_ptr<ScannerSettings> base,
  std::shared_ptr<ScannerSettings> ref,
  std::string const defaultTemplateId,
  std::unordered_set<std::string>& fields)
{
  if (ref->id != defaultTemplateId)
    fields.insert("baseTemplate");
  if (base->active != ref->active)
    fields.insert("active");
  if (base->headRotatePerSec_rad != ref->headRotatePerSec_rad)
    fields.insert("headRotatePerSec_rad");
  if (base->headRotateStart_rad != ref->headRotateStart_rad)
    fields.insert("headRotateStart_rad");
  if (base->headRotateStop_rad != ref->headRotateStop_rad)
    fields.insert("headRotateStop_rad");
  if (base->pulseFreq_Hz != ref->pulseFreq_Hz)
    fields.insert("pulseFreq_Hz");
  if (base->scanAngle_rad != ref->scanAngle_rad)
    fields.insert("scanAngle_rad");
  if (base->verticalAngleMin_rad != ref->verticalAngleMin_rad)
    fields.insert("verticalAngleMin_rad");
  if (base->verticalAngleMax_rad != ref->verticalAngleMax_rad)
    fields.insert("verticalAngleMax_rad");
  if (base->scanFreq_Hz != ref->scanFreq_Hz)
    fields.insert("scanFreq_Hz");
  if (base->beamDivAngle != ref->beamDivAngle)
    fields.insert("beamDivAngle");
  if (base->trajectoryTimeInterval != ref->trajectoryTimeInterval)
    fields.insert("trajectoryTimeInterval");
  if (base->verticalResolution_rad != ref->verticalResolution_rad)
    fields.insert("verticalResolution_rad");
  if (base->horizontalResolution_rad != ref->horizontalResolution_rad)
    fields.insert("horizontalResolution_rad");
}

void
XmlAssetsLoader::trackNonDefaultPlatformSettings(
  std::shared_ptr<PlatformSettings> base,
  std::shared_ptr<PlatformSettings> ref,
  std::string const defaultTemplateId,
  std::unordered_set<std::string>& fields)
{
  if (ref->id != defaultTemplateId)
    fields.insert("baseTemplate");
  if (base->x != ref->x)
    fields.insert("x");
  if (base->y != ref->y)
    fields.insert("y");
  if (base->z != ref->z)
    fields.insert("z");
  if (base->yawAtDepartureSpecified != ref->yawAtDepartureSpecified)
    fields.insert("yawAtDepartureSpecified");
  if (base->yawAtDeparture != ref->yawAtDeparture)
    fields.insert("yawAtDeparture");
  if (base->onGround != ref->onGround)
    fields.insert("onGround");
  if (base->stopAndTurn != ref->stopAndTurn)
    fields.insert("stopAndTurn");
  if (base->smoothTurn != ref->smoothTurn)
    fields.insert("smoothTurn");
  if (base->slowdownEnabled != ref->slowdownEnabled)
    fields.insert("slowdownEnabled");
  if (base->movePerSec_m != ref->movePerSec_m)
    fields.insert("movePerSec_m");
}
// ***  STATIC METHODS  *** //
// ************************ //
bool
XmlAssetsLoader::isProceduralAsset(std::string const& type,
                                   std::string const& id)
{
  if (type == "platform") {
    if (id == "interpolated")
      return true;
  }
  return false;
}

fs::path
XmlAssetsLoader::locateAssetFile(const std::string& filename)
{
  fs::path searchfile(filename);
  if (searchfile.is_relative()) {
    for (const auto path : assetsDir) {
      if (fs::exists(fs::path(path) / searchfile)) {
        searchfile = fs::path(path) / searchfile;
        break;
      }
    }
  }
  return searchfile.string();
}
