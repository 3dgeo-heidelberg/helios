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
     * @return Created leg
     * @see Leg
     */
	std::shared_ptr<Leg> createLegFromXML(
	    tinyxml2::XMLElement* legNode,
	    std::unordered_set<std::string> *scannerFields
    );
	/**
	 * @brief Create a survey form a XML element/node
	 *
	 * Notice that calling this method will reinitialize the loader by
	 *  calling XmlSurveyLoader::reinitLoader
	 *
	 * @param surveyNode XML element/node containing survey data
	 * @param legNoiseDisabled Disable leg noise
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
	 * @param legNoiseDisabled Disable leg noise
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
};