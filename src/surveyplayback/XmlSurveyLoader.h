#pragma once

#include <XmlAssetsLoader.h>
#include <SpectralLibrary.h>
#include <Survey.h>
#include <Leg.h>
#include <ScanningStrip.h>

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <memory>

/**
 * @brief Survey loader from XML
 *
 * This class strongly depends on its parent class XmlAssetsLoader
 *
 * @see XmlAssetsLoader
 */
class XmlSurveyLoader : public XmlAssetsLoader {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Map of scaning strips
     * @see ScanningStrip
     */
    std::unordered_map<std::string, std::shared_ptr<ScanningStrip>> strips;
    /**
     * @brief Serial identifier for last created leg. It is initialized as -1,
     *  so the first leg has 0 as serial id.
     */
    int lastLegSerialId = -1;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief XML survey loader constructor
     * @param filePath
     * @param assetsDir
     */
	XmlSurveyLoader(std::string& filePath, std::string& assetsDir)
	    : XmlAssetsLoader(filePath, assetsDir) {}

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Create a leg from a XML element/node
     * @param legNode XML element/node containing leg data
     * @param[in] scannerFields Unordered set to track which scanner settings
     *  have been overloaded
     * @param[in] platformFields Unordered set to track which platform settings
     *  have been overloaded
     * @return Created leg
     * @see Leg
     */
	std::shared_ptr<Leg> createLegFromXML(
	    tinyxml2::XMLElement* legNode,
	    std::unordered_set<std::string> *scannerFields,
	    std::unordered_set<std::string> *platformFields
    );
	/**
	 * @brief Create a survey form a XML element/node
	 *
	 * Notice that calling this method will reinitialize the loader by
	 *  calling XmlSurveyLoader::reinitLoader
	 *
	 * @param surveyNode XML element/node containing survey data
	 * @param legNoiseDisabled Flag to disable leg noise
	 * @param rebuildScene Flag to specify scene must be rebuild even
	 *  when a previously built one is found (true) or not (false)
	 * @return Created survey
	 * @see Survey
	 * @see XmlSurveyLoader::reinitLoader
	 */
	std::shared_ptr<Survey> createSurveyFromXml(
	    tinyxml2::XMLElement* surveyNode,
	    bool legNoiseDisabled=false,
	    bool rebuildScene=false
    );
	/**
	 * @brief Load a full survey from XML
	 * @param legNoiseDisabled Flag to disable leg noise
	 * @param rebuildScene Flag to specify scene must be rebuild even
	 *  when a previously built one is found (true) or not (false)
	 * @return Fully loaded survey
	 * @see Survey
	 */
	std::shared_ptr<Survey> load(
	    bool legNoiseDisabled=false,
	    bool rebuildScene=false
    );

protected:
    // ***  UTIL METHODS  *** //
    // ********************** //
    /**
     * @brief Call the reinitLoader from XmlAssetsLoader and then also assure
     *  that the map of scanning strips is empty. The last serial leg
     *  identifier is also setted to -1.
     *
     * @see XmlAssetsLoader::reinitLoader
     */
    void reinitLoader() override;
    /**
     * @brief Load scene from XML
     * @param sceneString String from XML scene attribute at survey
     *  element/node
	 * @param rebuildScene Flag to specify scene must be rebuild even
	 *  when a previously built one is found (true) or not (false)
     * @return Loaded scene
     * @see Scene
     */
	std::shared_ptr<Scene> loadScene(
	    std::string sceneString,
	    bool rebuildScene=false
    );
	/**
	 * @brief Load the core components of the survey. It is, its name,
	 *  the source file path, the scanner and the platform
	 * @param surveyNode XML element/node containing survey data
	 * @param survey The survey to be loaded
	 * @see XmlSurveyLoader::handleCoreOverloading
	 */
	void loadSurveyCore(
        tinyxml2::XMLElement *surveyNode,
        std::shared_ptr<Survey> survey
    );
	/**
	 * @brief Handle overloading of survey's core components.
	 * @param surveyNode XML element/node containing survey data
	 * @param survey The survey to be loaded
	 * @see XmlSurveyLoader::loadSurveyCore
	 */
	void handleCoreOverloading(
	    tinyxml2::XMLElement *surveyNode,
	    std::shared_ptr<Survey> survey
    );
	/**
	 * @brief Load all legs defining the survey
	 * @param legNodes First leg node
	 * @param scannerSettings The scanner settings of the survey scanner itself
	 * @param platformSettings The platform settings of the survey platform
	 *  itself
	 * @param legs Vector where loaded legs must be inserted
	 */
	void loadLegs(
	    tinyxml2::XMLElement *legNodes,
	    std::shared_ptr<ScannerSettings> scannerSettings,
	    std::shared_ptr<PlatformSettings> platformSettings,
	    std::vector<std::shared_ptr<Leg>> &legs
    );
	/**
	 * @brief Apply scene geometry shift to platform waypoints
	 * @param surveyNode XML element/node containing survey data
	 * @param legNoiseDisabled Flag to disable leg noise
	 * @sparam survey The survey containing all legs and scene data
	 */
    void applySceneShift(
        tinyxml2::XMLElement *surveyNode,
        bool const legNoiseDisabled,
        std::shared_ptr<Survey> survey
    );
    /**
     * @brief Configure the default randomness generador. If a seed was
     *  specified through XML survey node, then it will replace the
     *  previous one.
     * @param surveyNode XML elemetn/node containing survey data
     */
    static void configureDefaultRandomnessGenerator(
        tinyxml2::XMLElement *surveyNode
    );
    /**
     * @brief Load plataform noise, overriding what is specified in
     *  platform.xml if necessary
     * @param surveyNode XML element/node containing survey data
     * @param platform The platform which noise must be loaded
     */
	void loadPlatformNoise(
	    tinyxml2::XMLElement *surveyNode,
        std::shared_ptr<Platform> platform
    );
};