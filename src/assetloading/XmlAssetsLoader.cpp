#include "logging.hpp"
#include <HeliosException.h>


#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>

#include <glm/gtx/norm.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "typedef.h"

#include "AbstractGeometryFilter.h"
#include "ScaleFilter.h"
#include "GeoTiffFileLoader.h"
#include "WavefrontObjFileLoader.h"
#include "RotateFilter.h"
#include "TranslateFilter.h"
#include "XYZPointCloudFileLoader.h"
#include "DetailedVoxelLoader.h"

#include "GroundVehiclePlatform.h"
#include "HelicopterPlatform.h"
#include "LinearPathPlatform.h"

#include "ConicBeamDeflector.h"
#include "FiberArrayBeamDeflector.h"
#include "FullWaveformPulseDetector.h"
#include "OscillatingMirrorBeamDeflector.h"
#include "PolygonMirrorBeamDeflector.h"

#include "XmlAssetsLoader.h"
#include <NormalNoiseSource.h>
#include <UniformNoiseSource.h>

#include "MathConverter.h"

XmlAssetsLoader::XmlAssetsLoader(std::string& filePath, std::string& assetsDir)
: assetsDir(assetsDir){
		
	fs::path xmlFile(filePath);
	xmlDocFilename = xmlFile.filename().string();
	xmlDocFilePath = xmlFile.parent_path().string();
	logging::INFO("xmlDocFilename: " + xmlDocFilename);
	logging::INFO("xmlDocFilePath: " + xmlDocFilePath);

	tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
	if (result != tinyxml2::XML_SUCCESS) {
		logging::WARN("ERROR: loading " + filePath + " failed.");
	}
}

std::shared_ptr<Asset> XmlAssetsLoader::createAssetFromXml(
    std::string type,
    tinyxml2::XMLElement* assetNode
){
	if (assetNode == NULL) {
		logging::ERR("ERROR: Asset definition XML node is null!");
		exit(-1);
	}

	std::shared_ptr<Asset> result;
	if (type == "platform") {
		result = std::dynamic_pointer_cast<Asset>(createPlatformFromXml(assetNode));
	}
	else if (type == "platformSettings") {
		result = std::dynamic_pointer_cast<Asset>(createPlatformSettingsFromXml(assetNode));
	}
	else if (type == "scanner") {
		result = std::dynamic_pointer_cast<Asset>(createScannerFromXml(assetNode));
	}
	else if (type == "scene") {
		result = std::dynamic_pointer_cast<Asset>(createSceneFromXml(
		    assetNode, xmlDocFilePath
        ));
	}
	else if (type == "scannerSettings") {
		result = std::dynamic_pointer_cast<Asset>(createScannerSettingsFromXml(assetNode));
	}
	else if (type == "FWFSettings") {
		result = std::dynamic_pointer_cast<Asset>(createFWFSettingsFromXml(assetNode));
	}
	else {
		logging::ERR("ERROR: Unknown asset type: " + type );
		exit(-1);
	}

	// Read "asset" properties:
	result->id = boost::get<std::string>(getAttribute(assetNode,
	    "id", "string", std::string("")));
	result->name = boost::get<std::string>(getAttribute(assetNode,
	    "name", "string", "Unnamed " + type + " asset"));

	// Store source file path for possible later XML export:
	result->sourceFilePath = xmlDocFilePath;

	return result;
}

Color4f XmlAssetsLoader::createColorFromXml(tinyxml2::XMLElement* node) {
	Color4f col;
	try {
		float r = boost::lexical_cast<float>(node->Attribute("r"));
		float g = boost::lexical_cast<float>(node->Attribute("g"));
		float b = boost::lexical_cast<float>(node->Attribute("b"));
		col = Color4f(r, g, b, 1);
	}
	catch (std::exception &e) {
		logging::INFO(
		    std::string("Error creating color from xml.\nEXCEPTION: ") +
		    e.what()
        );
	}
	return col;
}

std::map<std::string, ObjectT> XmlAssetsLoader::createParamsFromXml(
    tinyxml2::XMLElement* paramsNode
){

	std::map<std::string, ObjectT> result;

	if (paramsNode == nullptr) {
		return result;
	}

	tinyxml2::XMLElement* element = paramsNode->FirstChildElement("param");
	while (element != nullptr) {

		try {
			std::string type = element->Attribute("type");
			std::string key = element->Attribute("key");
			std::string valueString;
			const char* attribute = element->Attribute("value");
			if (attribute) valueString = attribute;

			if (type.empty() || type.compare("string") == 0) {
				result.insert(std::pair<std::string, std::string>(key, valueString));
			}
			else {

				if (type == "boolean") {
					bool b = valueString == "true" ? true : false;
					result.insert(std::pair<std::string, bool>(key, b));
				}
				else if (type == "double") {
					result.insert(std::pair<std::string, double>(
					    key, boost::lexical_cast<double>(valueString)
                    ));
				}
				else if (type == "integer" || type == "int"){
				    result.insert(std::pair<std::string, int>(
                        key, boost::lexical_cast<int>(valueString)
                    ));
				}
				else if (type == "rotation") {
					result.insert(
					    std::pair<std::string, Rotation>(
					        key,
					        createRotationFromXml(element)
                        )
                    );
				}
				else if (type == "vec3") {
					std::vector<std::string> vec;
					boost::split(vec, valueString, boost::is_any_of(";"));
					double x = boost::lexical_cast<double>(vec.at(0));
					double y = boost::lexical_cast<double>(vec.at(1));
					double z = boost::lexical_cast<double>(vec.at(2));

					result.insert(std::pair<std::string, glm::dvec3>(key, glm::dvec3(x, y, z)));
				}
			}
		}
		catch (std::exception &e) {
			logging::INFO(
			    std::string("Failed to read filter parameter: ")+e.what()
            );
		}

		element = element->NextSiblingElement("param");
	}

	return result;
}

std::shared_ptr<Platform> XmlAssetsLoader::createPlatformFromXml(
    tinyxml2::XMLElement* platformNode
) {
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
    SimplePhysicsPlatform *spp = dynamic_cast<SimplePhysicsPlatform*>(platform.get());
    if(spp!=NULL){
        spp->mCfg_drag = platformNode->DoubleAttribute("drag", 1.0);
    }

    // Read HelicopterPlatform related stuff
    HelicopterPlatform *hp = dynamic_cast<HelicopterPlatform*>(platform.get());
    if(hp!=nullptr){
        hp->cfg_speedup_magnitude =
            platformNode->DoubleAttribute("speedup_magnitude", 2.0);
        hp->cfg_slowdown_magnitude =
            platformNode->DoubleAttribute("slowdown_magnitude", 2.0);
        hp->cfg_pitch_base = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("base_pitch_deg", -5.0)
        );
        hp->ef_xy_max = platformNode->DoubleAttribute("engine_max_force", 0.1);
        hp->cfg_pitch_speed = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("pitch_speed_deg", 85.94)
        );
        hp->cfg_roll_speed = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("roll_speed_deg", 28.65)
        );
        hp->cfg_yaw_speed = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("yaw_speed_deg", 85.94)
        );
        hp->cfg_max_pitch_offset = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("pitch_offset_deg", 35.0)
        );
        hp->cfg_max_roll_offset = MathConverter::degreesToRadians(
            platformNode->DoubleAttribute("roll_offset_deg", 25.0)
        );
        hp->cfg_max_pitch = hp->cfg_pitch_base + hp->cfg_max_pitch_offset;
        hp->cfg_min_pitch = hp->cfg_pitch_base - hp->cfg_max_pitch_offset;
        hp->cfg_slowdown_dist_xy =
            platformNode->DoubleAttribute("slowdown_distance", 5.0);
    }


    // Read relative scanner rotation:
    try {
        tinyxml2::XMLElement *scannerMountNode = platformNode->FirstChildElement("scannerMount");

        // Read relative position of the scanner mount on the platform:
        platform->cfg_device_relativeMountPosition = createVec3dFromXml(scannerMountNode, "");

        // Read relative orientation of the scanner mount on the platform:
        platform->cfg_device_relativeMountAttitude = createRotationFromXml(
            scannerMountNode
        );
    }
    catch (std::exception &e) {
        logging::WARN(
            std::string("No scanner orientation defined.\nEXCEPTION: ") +
            e.what()
        );
    }

    // ########## BEGIN Read Platform noise specification ##########
    tinyxml2::XMLElement *positionXNoise =
        platformNode->FirstChildElement("positionXNoise");
    if (positionXNoise != NULL)
        platform->positionXNoiseSource = createNoiseSource(positionXNoise);
    else
        logging::DEBUG("No default platform position X noise was specified");
    tinyxml2::XMLElement *positionYNoise =
        platformNode->FirstChildElement("positionYNoise");
    if (positionYNoise != NULL)
        platform->positionYNoiseSource = createNoiseSource(positionYNoise);
    else
        logging::DEBUG("No default platform position Y noise was specified");
    tinyxml2::XMLElement *positionZNoise =
        platformNode->FirstChildElement("positionZNoise");
    if (positionZNoise != NULL)
        platform->positionZNoiseSource = createNoiseSource(positionZNoise);
    else
        logging::DEBUG("No default platform position Z noise was specified");

    tinyxml2::XMLElement *attitudeXNoise =
        platformNode->FirstChildElement("attitudeXNoise");
    if (attitudeXNoise != NULL)
        platform->attitudeXNoiseSource = createNoiseSource(attitudeXNoise);
    else
        logging::DEBUG("No default platform attitude X noise was specified");
    tinyxml2::XMLElement *attitudeYNoise =
        platformNode->FirstChildElement("attitudeYNoise");
    if (attitudeYNoise != NULL)
        platform->attitudeYNoiseSource = createNoiseSource(attitudeYNoise);
    else
        logging::DEBUG("No default platform attitude Y noise was specified");
    tinyxml2::XMLElement *attitudeZNoise =
        platformNode->FirstChildElement("attitudeZNoise");
    if (attitudeZNoise != NULL)
        platform->attitudeZNoiseSource = createNoiseSource(attitudeZNoise);
    else
        logging::DEBUG("No default platform attitude Z noise was specified");

    // ########## END Read Platform noise specification ##########

	return platform;
}

std::shared_ptr<PlatformSettings> XmlAssetsLoader::createPlatformSettingsFromXml(tinyxml2::XMLElement* node) {

	std::shared_ptr<PlatformSettings> settings(new PlatformSettings());
	std::shared_ptr<PlatformSettings> template1(new PlatformSettings());
	if (node->Attribute("template") != NULL) {
		std::shared_ptr<Asset> bla = getAssetByLocation("platformSettings", node->Attribute("template"));
		if (bla != NULL) {
			template1 = std::dynamic_pointer_cast<PlatformSettings>(bla);
			// ATTENTION:
			// We need to temporarily convert the head rotation settings from radians back to degrees, since degrees
			// is the unit in which they are read from the XML, and below, the template settings are used as defaults
			// in case that a value is not specified in the XML!

		}
		else {
		    std::stringstream ss;
            ss << "XML Assets Loader: WARNING: " <<
                "Platform settings template specified in line " <<
                node->GetLineNum() << "\nnot found: '" <<
				"Using hard-coded defaults instead.";
            logging::WARN(ss.str());
		}
	}

	// Read platform coordinates
	settings->x = boost::get<double>(getAttribute(
	    node, "x", "double", template1->x));
	settings->y = boost::get<double>(getAttribute(
	    node, "y", "double", template1->y));
	settings->z = boost::get<double>(getAttribute(
	    node, "z", "double", template1->z));

	// Read if platform should be put on ground, ignoring z coordinate:
	settings->onGround = boost::get<bool>(getAttribute(
	    node, "onGround", "bool", template1->onGround));

	// Read if platform must use stop and turn mechanics or not
	settings->stopAndTurn = boost::get<bool>(getAttribute(
	    node, "stopAndTurn", "bool", template1->stopAndTurn));

	// Read if platform must use smooth turn mechanics or not
	settings->smoothTurn = boost::get<bool>(getAttribute(
	    node, "smoothTurn", "bool", template1->smoothTurn));

	// Read if platform must be able to slowdown (true) or not (false)
	settings->slowdownEnabled = boost::get<bool>(getAttribute(
	    node, "slowdownEnabled", "bool", template1->slowdownEnabled));

	// Read platform speed:
	settings->movePerSec_m = boost::get<double>(getAttribute(
	    node, "movePerSec_m", "double", template1->movePerSec_m));

	if(node->FindAttribute("yawAtDeparture_deg") != nullptr){
	    settings->yawAtDepartureSpecified = true;
	    settings->yawAtDeparture = MathConverter::degreesToRadians(
	        boost::get<double>(getAttribute(
	            node, "yawAtDeparture_deg", "double", template1->yawAtDeparture
            ))
        );
	}

	return settings;
}

std::shared_ptr<Scanner> XmlAssetsLoader::createScannerFromXml(
    tinyxml2::XMLElement* scannerNode
) {

	// ############ BEGIN Read emitter position and orientation ############
	glm::dvec3 emitterPosition = glm::dvec3(0, 0, 0);
	Rotation emitterAttitude(glm::dvec3(1.0, 0.0, 0.0), 0.0);

	try {
		tinyxml2::XMLElement* emitterNode = scannerNode->FirstChildElement("beamOrigin");

		// Read relative position of the scanner mount on the platform:
		emitterPosition = createVec3dFromXml(emitterNode, "");

		// Read relative orientation of the scanner mount on the platform:
		emitterAttitude = createRotationFromXml(emitterNode);
	}
	catch (std::exception &e) {
		logging::WARN(
		    std::string("No scanner orientation defined.\n") +
            "EXCEPTION: "+e.what());
	}

	// ############ END Read emitter position and orientation ############

	// ########## BEGIN Read supported pulse frequencies ############
	std::string pulseFreqsString = boost::get<std::string>(getAttribute(
	    scannerNode, "pulseFreqs_Hz", "string", std::string("")));
	std::list<int> pulseFreqs = std::list<int>();

	std::vector<std::string> freqs;
	boost::split(freqs, pulseFreqsString, boost::is_any_of(","));
	for (std::string freq : freqs) {
		int f = boost::lexical_cast<int>(freq);
		pulseFreqs.push_back(f);
	}

	// ########## END Read supported pulse frequencies ############

	// ########### BEGIN Read all the rest #############
	double beamDiv_rad = boost::get<double>(getAttribute(scannerNode,
	    "beamDivergence_rad", "double", 0.0003));
	double pulseLength_ns = boost::get<double>(getAttribute(scannerNode,
	    "pulseLength_ns", "double", 4.0));
	std::string id = boost::get<std::string>(getAttribute(scannerNode,
	    "id", "string", std::string("Default")));
	double avgPower = boost::get<double>(getAttribute(scannerNode,
	    "averagePower_w", "double", 4.0));
	double beamQuality = boost::get<double>(getAttribute(scannerNode,
	    "beamQualityFactor", "double", 1.0));
	double efficiency = boost::get<double>(getAttribute(scannerNode,
	    "opticalEfficiency", "double", 0.99));
	double receiverDiameter = boost::get<double>(getAttribute(scannerNode,
	    "receiverDiameter_m", "double", 0.15));
	double visibility = boost::get<double>(getAttribute(scannerNode,
	    "atmosphericVisibility_km", "double", 23.0));
	int wavelength = boost::get<int>(getAttribute(scannerNode,
	    "wavelength_nm", "int", 1064));
	// ########### END Read all the rest #############

	std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(
        beamDiv_rad,
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
        false
    );

    // ########## BEGIN Default FWF_settings ##########
    std::shared_ptr<FWFSettings> settings = std::make_shared<FWFSettings>();
    settings->pulseLength_ns = pulseLength_ns;
    scanner->applySettingsFWF(*createFWFSettingsFromXml(
        scannerNode->FirstChildElement("FWFSettings"),
        settings
    ));
    // ########## END Default FWF_settings ##########

	// ############################# BEGIN Configure scanner head ##############################
	// ################### BEGIN Read Scan head rotation axis #############
	glm::dvec3 headRotateAxis = glm::dvec3(0, 0, 1);

	try {
		glm::dvec3 axis = createVec3dFromXml(scannerNode->FirstChildElement("headRotateAxis"), "");
		if (glm::l2Norm(axis) > 0.1) {
			headRotateAxis = axis;
		}
	}
	catch (std::exception &e) {
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
	    boost::get<double>(
	        getAttribute(scannerNode, "headRotatePerSecMax_deg", "double", 0.0)
        )
    );

	// Configure scanner head:
	scanner->scannerHead = std::shared_ptr<ScannerHead>(
	    new ScannerHead(headRotateAxis, headRotatePerSecMax_rad));

	// ############################# END Configure scanner head ##############################

	// ################################## BEGIN Configure beam deflector ######################################

	// ########### BEGIN Read and apply generic properties ##########
	double scanFreqMax_Hz = boost::get<double>(getAttribute(scannerNode, "scanFreqMax_Hz", "double", 0.0));
	double scanFreqMin_Hz = boost::get<double>(getAttribute(scannerNode, "scanFreqMin_Hz", "double", 0.0));
	double scanAngleMax_rad = MathConverter::degreesToRadians(
	    boost::get<double>(
	        getAttribute(scannerNode, "scanAngleMax_deg", "double", 0.0)
        )
    );
	// ########### END Read and apply generic properties ##########

	std::string str_opticsType = scannerNode->Attribute("optics");
	std::shared_ptr<AbstractBeamDeflector> beamDeflector = NULL;

	if (str_opticsType == "oscillating") {
		int scanProduct = boost::get<int>(getAttribute(scannerNode, "scanProduct", "int", 1000000));
		beamDeflector = std::shared_ptr<OscillatingMirrorBeamDeflector>(new OscillatingMirrorBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz, scanProduct));
	}
	else if (str_opticsType == "conic") {
		beamDeflector = std::shared_ptr<ConicBeamDeflector>(new ConicBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz));
	}
	else if (str_opticsType == "line") {
		int numFibers = boost::get<int>(getAttribute(scannerNode, "numFibers", "int", 1));
		beamDeflector = std::shared_ptr<FiberArrayBeamDeflector>(new FiberArrayBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz, numFibers));
	}
	else if (str_opticsType == "rotating") {
		double scanAngleEffectiveMax_rad = MathConverter::degreesToRadians(
		    boost::get<double>(getAttribute(
		        scannerNode, "scanAngleEffectiveMax_deg", "double", 0.0
            ))
        );
		beamDeflector = std::shared_ptr<PolygonMirrorBeamDeflector>(new PolygonMirrorBeamDeflector(scanFreqMax_Hz, scanFreqMin_Hz, scanAngleMax_rad, scanAngleEffectiveMax_rad));
	}

	if (beamDeflector == NULL) {
	    std::stringstream ss;
	    ss << "ERROR: Unknown beam deflector type: '" << str_opticsType <<
	        "'. Aborting.";
	    logging::ERR(ss.str());
		exit(1);
	}

	scanner->beamDeflector = beamDeflector;

	// ################################## END Configure beam deflector #######################################

	// ############################ BEGIN Configure detector ###############################
	double rangeMin_m = boost::get<double>(getAttribute(
	    scannerNode, "rangeMin_m", "double", 0.0));
	double accuracy_m = boost::get<double>(getAttribute(
	    scannerNode, "accuracy_m", "double", 0.0));
	scanner->detector = std::make_shared<FullWaveformPulseDetector>(
	    scanner, accuracy_m, rangeMin_m);
	// ############################ END Configure detector ###############################

	return scanner;
}

std::shared_ptr<ScannerSettings> XmlAssetsLoader::createScannerSettingsFromXml(
    tinyxml2::XMLElement* node
){

	std::shared_ptr<ScannerSettings> settings(new ScannerSettings());
	std::shared_ptr<ScannerSettings> template1(new ScannerSettings());

	template1->active = true;
	template1->headRotatePerSec_rad = 0;
	template1->headRotateStart_rad = 0;
	template1->headRotateStop_rad = 0;
	template1->pulseFreq_Hz = 0;
	template1->scanAngle_rad = 0;
	template1->verticalAngleMin_rad = 0;
	template1->verticalAngleMax_rad = 0;
	template1->scanFreq_Hz = 0;

	if (node->Attribute("template") != NULL) {
		std::shared_ptr<ScannerSettings> bla =
		    std::dynamic_pointer_cast<ScannerSettings>(
		        getAssetByLocation(
		            "scannerSettings",
		            node->Attribute("template")
                )
            );
		if (bla != NULL) {
			template1 = bla;

			// ATTENTION:
			// We need to temporarily convert the head rotation settings from radians back to degrees, since degrees
			// is the unit in which they are read from the XML, and below, the template settings are used as defaults
			// in case that a value is not specified in the XML!
			template1->headRotatePerSec_rad =
			    MathConverter::radiansToDegrees(
			        template1->headRotatePerSec_rad
                );
			template1->headRotateStart_rad =
			    MathConverter::radiansToDegrees(
			        template1->headRotateStart_rad
                );
			template1->headRotateStop_rad =
			    MathConverter::radiansToDegrees(
			        template1->headRotateStop_rad
                );
			template1->scanAngle_rad =
			    MathConverter::radiansToDegrees(
                    template1->scanAngle_rad
                );
			template1->verticalAngleMin_rad =
			    MathConverter::radiansToDegrees(
			        template1->scanAngle_rad
                );
			template1->verticalAngleMax_rad =
			    MathConverter::radiansToDegrees(
			        template1->scanAngle_rad
                );
		}
		else {
		    std::stringstream ss;
		    ss << "XML Assets Loader: " <<
                "WARNING: Scanner settings template specified in line " <<
                node->GetLineNum() + " not found: '" <<
				"Using hard-coded defaults instead.";
            logging::WARN(ss.str());
		}
	}

	settings->active = boost::get<bool>(getAttribute(
	    node, "active", "bool", template1->active));
	settings->beamSampleQuality = boost::get<int>(
	    getAttribute(node, "beamSampleQuality", "int", template1->beamSampleQuality));
	settings->headRotatePerSec_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(
	        node, "headRotatePerSec_deg", "double",
	        template1->headRotatePerSec_rad)
        )
    );
	settings->headRotateStart_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(
	        node, "headRotateStart_deg", "double",
	        template1->headRotateStart_rad
        ))
    );

	double hrStop_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(
	        node, "headRotateStop_deg", "double",
	        template1->headRotateStop_rad
        ))
    );

	// Make sure that rotation stop angle is larger than rotation start angle if rotation speed is positive:
	if (hrStop_rad < settings->headRotateStart_rad &&
	    settings->headRotatePerSec_rad > 0
    ){
		logging::ERR(
		    std::string("XML Assets Loader: Error: ")+
		    "Head Rotation Stop angle must be larger than start angle "+
            "if rotation speed is positive!"
        );
		exit(-1);
	}

	// Make sure that rotation stop angle is larger than rotation start angle if rotation speed is positive:
	if (hrStop_rad > settings->headRotateStart_rad &&
	    settings->headRotatePerSec_rad < 0) {
		logging::ERR(
		    std::string("XML Assets Loader: Error: ")+
		    "Head Rotation Stop angle must be smaller than start angle if "+
		    "rotation speed is negative!"
        );
		exit(-1);
	}

	settings->headRotateStop_rad = hrStop_rad;
	settings->pulseFreq_Hz = boost::get<int>(getAttribute(node,
	    "pulseFreq_hz", "int", template1->pulseFreq_Hz));
	settings->scanAngle_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(
	        node, "scanAngle_deg", "double", template1->scanAngle_rad
        ))
    );
	settings->verticalAngleMin_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(node,
	        "verticalAngleMin_deg", "double", template1->verticalAngleMin_rad
        ))
    );
	settings->verticalAngleMax_rad = MathConverter::degreesToRadians(
	    boost::get<double>(getAttribute(node,
	        "verticalAngleMax_deg", "double", template1->verticalAngleMax_rad
        ))
    );
	settings->scanFreq_Hz = boost::get<int>(getAttribute(node,
	    "scanFreq_hz", "int", template1->scanFreq_Hz));

    settings->trajectoryTimeInterval = boost::get<double>(getAttribute(
        node, "trajectoryTimeInterval_s", "double", 0.0));

	return settings;
}


std::shared_ptr<FWFSettings> XmlAssetsLoader::createFWFSettingsFromXml(
    tinyxml2::XMLElement* node,
    std::shared_ptr<FWFSettings> settings
) {
    if(settings == nullptr){
        settings = std::make_shared<FWFSettings>();
    }
    // If no FWFSettings node appears on XML, default is used
    if(node != nullptr) {
        settings->binSize_ns = boost::get<double>(getAttribute(node, "binSize_ns", "double", settings->binSize_ns));
        settings->winSize_ns = settings->pulseLength_ns / 4.0; // By default
        settings->beamSampleQuality = boost::get<int>(getAttribute(node, "beamSampleQuality", "int", settings->beamSampleQuality));
        settings->winSize_ns = boost::get<double>(getAttribute(node, "winSize_ns", "double", settings->winSize_ns));
        settings->maxFullwaveRange_ns = boost::get<double>(getAttribute(node, "maxFullwaveRange_ns", "double", settings->maxFullwaveRange_ns));
    }

	return settings;
}

// ################# END get(asset) by id methods #############

std::shared_ptr<Scene> XmlAssetsLoader::createSceneFromXml(
    tinyxml2::XMLElement* sceneNode,
    std::string path
) {
	std::shared_ptr<Scene> scene(new Scene());
	scene->sourceFilePath = path;

	// ####################### BEGIN Loop over all part nodes ############################
	int partIndex = -1;
	tinyxml2::XMLElement* scenePartNodes = sceneNode->FirstChildElement("part");
	while (scenePartNodes != nullptr) {
	    partIndex++;
		ScenePart *scenePart = nullptr;
		tinyxml2::XMLElement* filterNodes = scenePartNodes->FirstChildElement("filter");
        bool holistic = false;
		while (filterNodes != nullptr) {
			std::string filterType = filterNodes->Attribute("type");
			std::transform(filterType.begin(), filterType.end(), filterType.begin(), ::tolower);
			AbstractGeometryFilter* filter = nullptr;

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
				// Read defined point cloud color:
				Color4f color(1.0, 1.0, 1.0, 1.0);
				if(scenePartNodes->FindAttribute("color") != nullptr) {
                    color = createColorFromXml(
                        scenePartNodes->FirstChildElement("color")
                    );
                }
				filter = new XYZPointCloudFileLoader();
				holistic = true;
			}

			// Read detailed voxels file
			else if (filterType == "detailedvoxels"){
			    filter = new DetailedVoxelLoader();
			}

			// ################### END Set up filter ##################

			// Finally, apply the filter:
			if (filter != nullptr) {
				// Set params:
				filter->params = createParamsFromXml(filterNodes);
				logging::INFO("Applying filter: "+filterType);
				scenePart = filter->run();
				if(scenePart == filter->primsOut) filter->primsOut = nullptr;
                delete filter;
			}

			filterNodes = filterNodes->NextSiblingElement("filter");
		}
		// ############## END Loop over filter nodes ##################

		// ######### BEGIN Read and set scene part ID #########
		std::string partId = "";
		tinyxml2::XMLAttribute const * partIdAttr = scenePartNodes->FindAttribute("id");
		char const* str = NULL;
		if(partIdAttr != NULL) str = scenePartNodes->FindAttribute("id")->Value();
		if (str!=NULL) {
			partId = std::string(str);
			try{
			    boost::lexical_cast<int>(partId);
            }
            catch(boost::bad_lexical_cast &blcex){
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
		}
		else {
			scenePart->mId = partId;
			splitPart=false;
		}

		// ######### END Read and set scene part ID #########

		// Consider scene loading specification
        sceneSpec.apply(scenePart);

		// For all primitives, set reference to their scene part and transform:
		std::shared_ptr<ScenePart> sharedScenePart(scenePart);
		for (Primitive* p : scenePart->mPrimitives) {
			p->part = sharedScenePart;
			p->rotate(scenePart->mRotation);
            if(holistic){
                for(size_t i = 0 ; i < p->getNumVertices() ; i++){
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
		if(splitPart) {
            size_t partIndexOffset = scenePart->subpartLimit.size() - 1;
            if (scenePart->splitSubparts()) partIndex += partIndexOffset;
        }
        scenePartNodes = scenePartNodes->NextSiblingElement("part");
    }
    // ####################### END Loop over all part nodes ############################
    bool success = scene->finalizeLoading();

    if (!success) {
        logging::ERR("Finalizing the scene failed.");
		exit(-1);
	}

    return scene;
}

Rotation XmlAssetsLoader::createRotationFromXml(
    tinyxml2::XMLElement* rotGroupNode
) {
    bool globalRotation = true;
    if(rotGroupNode->Attribute("rotations", "local"))
        globalRotation=false;

    Rotation r = Rotation(glm::dvec3(1, 0, 0), 0);
	Rotation r2 = Rotation(glm::dvec3(1, 0, 0), 0);

	if (rotGroupNode == nullptr) {
		return r;
	}

	tinyxml2::XMLElement* rotNodes = rotGroupNode->FirstChildElement("rot");
	while (rotNodes != nullptr) {
		std::string axis = rotNodes->Attribute("axis");
		double angle_rad = MathConverter::degreesToRadians(
		    boost::lexical_cast<double>(rotNodes->Attribute("angle_deg"))
        );

		if (angle_rad != 0) {
			if (axis == "z" || axis == "Z") {
				r2 = Rotation(glm::dvec3(0, 0, 1), angle_rad);
            }
			if (axis == "x" || axis == "X") {
				r2 = Rotation(glm::dvec3(1, 0, 0), angle_rad);
			}
			if (axis == "y" || axis == "Y") {
				r2 = Rotation(glm::dvec3(0, 1, 0), angle_rad);
			}
			if(globalRotation) r = r2.applyTo(r);  // Global rotation
			else r = r.applyTo(r2);  // Local rotation
		}

		rotNodes = rotNodes->NextSiblingElement("rot");
	}

	return r;
}

glm::dvec3 XmlAssetsLoader::createVec3dFromXml(tinyxml2::XMLElement* node, std::string attrPrefix) {
	if (node == nullptr) {
		throw HeliosException(
		    "No node with attribute " + attrPrefix +"[xyz]"
        );
	}

	double x = boost::get<double>(getAttribute(node, attrPrefix + "x", "double", 0.0));
	double y = boost::get<double>(getAttribute(node, attrPrefix + "y", "double", 0.0));
	double z = boost::get<double>(getAttribute(node, attrPrefix + "z", "double", 0.0));

	return glm::dvec3(x, y, z);
}

std::shared_ptr<Asset> XmlAssetsLoader::getAssetById(
    std::string type,
    std::string id
){
	try {
	    tinyxml2::XMLElement* assetNodes = doc.FirstChild()->NextSibling()->FirstChildElement(type.c_str());
		
	    while (assetNodes != nullptr) {
            std::string str(assetNodes->Attribute("id"));
            if (str.compare(id) == 0) {
                return createAssetFromXml(type, assetNodes);
            }

            assetNodes = assetNodes->NextSiblingElement(type.c_str());
	    }

	    logging::WARN(
	        "ERROR: "+type+" asset definition not found: "+
	        this->xmlDocFilePath+"#"+id
        );

	} catch (std::exception &e) {
		logging::WARN(
		    "ERROR: Failed to read " + type + " asset definition: " +
		    this->xmlDocFilePath + "#" + id + "\nEXCEPTION: " + e.what()
        );
	}

	return NULL;
}

std::shared_ptr<Asset> XmlAssetsLoader::getAssetByLocation(
    std::string type,
    std::string location
){
	std::vector<std::string> vec;
	boost::split(vec, location, boost::is_any_of("#"));
	XmlAssetsLoader* loader = this;
	std::string id = vec[0].erase(vec[0].find_last_not_of("#") + 1);
	bool freeLoader = false;

    // External document location provided:
	if (vec.size() == 2) {
		loader = new XmlAssetsLoader(id, assetsDir);
		loader->sceneSpec = sceneSpec;
		id = vec[1].erase(vec[1].find_last_not_of('#') + 1);
		freeLoader = true;
	}

    std::shared_ptr<Asset> asset = loader->getAssetById(type, id);
	if(freeLoader) delete loader;
	return asset;
}

// ################# BEGIN (small stuff)FromXML methods #############

ObjectT XmlAssetsLoader::getAttribute(
    tinyxml2::XMLElement* element,
    std::string attrName,
    std::string type,
    ObjectT defaultVal
){

	ObjectT result;
	try {
		if (!element->Attribute(attrName.c_str())) {
			throw HeliosException(
			    "Attribute '" + attrName + "' does not exist!"
            );
		}
		std::string attrVal = element->Attribute(attrName.c_str());
		if (type == "bool") {
			if (attrVal == "1" || attrVal == "true") result = true;
			else if (attrVal == "0" || attrVal == "false") result = false;
			else {
				std::ostringstream s;
				s << "Attribute '" << attrName <<
				    "' does not exist!";
				logging::WARN(s.str());
				throw std::exception();
			}
		}
		else if (type == "int") {
			result = boost::lexical_cast<int>(attrVal);
		}
		else if (type == "float") {
			result = boost::lexical_cast<float>(attrVal);
		}
		else if (type == "double") {
			result = boost::lexical_cast<double>(attrVal);
		}
		else if (type == "string") {
			result = attrVal;
		}
		else {
			logging::WARN("ERROR: unknown type " + type);
		}
	}

	catch (std::exception &e) {
		std::stringstream ss;
	    ss  << "XML Assets Loader: Could not find attribute '"
		    << attrName << "' of <" << element->Name()
		    << "> element in line " << element->GetLineNum();
        logging::DEBUG(ss.str());
        ss.flush();
        ss.str("");

		if (!defaultVal.empty()) {
			result = defaultVal;
			ss  << "Using default value for attribute '"<< attrName << "' : "
			    << boost::apply_visitor(stringVisitor{}, defaultVal);
			logging::INFO(ss.str());
		}
		else {
		    ss  << "Exception:\n" << e.what() << "\n";
		    ss  << "ERROR: No default value specified for attribute '"
			    << attrName << "'. Aborting.";
		    logging::ERR(ss.str());
			exit(-1);
		}
	}

	return result;
}

std::shared_ptr<NoiseSource<double>>
XmlAssetsLoader::createNoiseSource(tinyxml2::XMLElement *noise){
    std::shared_ptr<NoiseSource<double>> ns;

    // Instantiate considering type
    std::string type = noise->Attribute("type", "NORMAL");
    if(type == "UNIFORM") {
        double min = noise->DoubleAttribute("min", 0.0);
        double max = noise->DoubleAttribute("max", 1.0);
        ns = std::make_shared<UniformNoiseSource<double>>(
            UniformNoiseSource<double>(*DEFAULT_RG, min, max)
        );
    }
    else{
        double mean = noise->DoubleAttribute("mean", 0.0);
        double stdev = noise->DoubleAttribute("stdev", 1.0);
        ns = std::make_shared<NormalNoiseSource<double>>(
            NormalNoiseSource<double>(*DEFAULT_RG, mean, stdev)
        );
    }

    // Configure clipping
    bool clipEnabled = noise->BoolAttribute("clipEnabled", false);
    double clipMin = noise->DoubleAttribute("clipMin", 0.0);
    double clipMax = noise->DoubleAttribute("clipMax", 1.0);
    ns->setClipMin(clipMin).setClipMax(clipMax).setClipEnabled(clipEnabled);

    // Configure fixed behavior
    unsigned long fixedLifespan =
        noise->Unsigned64Attribute("fixedLifespan", 1L);
    ns->setFixedLifespan(fixedLifespan);

    return ns;
}
