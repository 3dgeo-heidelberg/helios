#pragma once

#include "XmlAssetsLoader.h"
#include "SpectralLibrary.h"
#include "Survey.h"
#include "Leg.h"

#include <string>
#include <unordered_set>

/**
 * @brief Survey loader from XML
 *
 * This class strongly depends on its parent class XmlAssetsLoader
 *
 * @see XmlAssetsLoader
 */
class XmlSurveyLoader : public XmlAssetsLoader {

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
	 * @param surveyNode XML element/node containing survey data
	 * @param legNoiseDisabled Disable leg noise
	 * @param rebuildScene Flag to specify scene must be rebuild even
	 *  when a previously built one is found (true) or not (false)
	 * @return Created survey
	 * @see Survey
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

private:
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