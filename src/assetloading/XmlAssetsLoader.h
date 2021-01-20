#pragma once

#include <memory>
#include <string>

#include <tinyxml2.h>

#include "typedef.h"

#include "Asset.h"
#include "Color4f.h"
#include "Platform.h"
#include "PlatformSettings.h"
#include "Scanner.h"
#include <NoiseSource.h>
#include <SceneLoadingSpecification.h>

/**
 * @brief Class for asset loading from XML file
 */
class XmlAssetsLoader {
    // ***  ATTRIBUTES  *** //
    // ******************** //
protected:
    /**
     * @brief Assets directory
     */
	std::string assetsDir;
	/**
	 * @brief Name of the XML file
	 */
	std::string xmlDocFilename = "unknown.xml";
	/**
	 * @brief Path to the XML file
	 */
	std::string xmlDocFilePath = "";

public:
    /**
     * @brief XML file through tinyxml2 library
     */
	tinyxml2::XMLDocument doc;
	/**
	 * @brief Scene loading specification
	 * @see SceneLoadingSpecification
	 */
	SceneLoadingSpecification sceneSpec;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Build a XmlAssetsLoader for given XML file and assets directory
	 * @param filePath Path to XML File
	 * @param assetsDir Path to assets directory
	 */
	XmlAssetsLoader(std::string& filePath, std::string& assetsDir);

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	/**
	 * @brief Get asset by its identifier
	 * @param type Type of the asset
	 * @param id Identifier of the asset
	 * @return Shared pointer to requested asset
	 */
	std::shared_ptr<Asset> getAssetById(std::string type, std::string id);
	/**
	 * @brief Get asset by location
	 * @param type Type of the asset
	 * @param location Location of the asset
	 * @return Shared pointer to requested asset
	 */
	std::shared_ptr<Asset> getAssetByLocation(std::string type, std::string location);
protected:
    /**
     * @brief Obtain attribute from XML
     * @param element XML element (node) where the attribute must be taken from
     * @param attrName Name of the attribute to be obtained
     * @param type Type of the attribute to be obtained
     * @param defaultVal Default value to be used in case attribute was not
     * found
     * @return Obtained attribute or default value if attribute was not found
     */
    ObjectT getAttribute(
        tinyxml2::XMLElement* element,
        std::string attrName,
        std::string type,
        ObjectT defaultVal
    );

public:
    // ***  CREATION METHODS  *** //
    // ************************** //
    /**
     * @brief Create an asset from given XML element (node)
     * @param type Asset type
     * @param assetNode XML element (node) containing asset data
     * @return Shared pointer to created asset
     * @see Asset
     */
	std::shared_ptr<Asset> createAssetFromXml(
	    std::string type,
	    tinyxml2::XMLElement* assetNode
    );

	/**
	 * @brief Create a color from given XML element (node)
	 * @param node XML element (node) containing color data
	 * @return Created color
	 * @see Color4f
	 */
	Color4f createColorFromXml(tinyxml2::XMLElement* node);
	/**
	 * @brief Create a map of parameters from given XML element (node)
	 * @param paramsNode XML element (node) containing parameters
	 * @return Map with parameters, so each one is identified by a different
	 * string
	 */
	std::map<std::string, ObjectT> createParamsFromXml(
	    tinyxml2::XMLElement* paramsNode
    );
	/**
	 * @brief Create a platform from given XML element (node)
	 * @param platformNode XML element (node) containing platform data
	 * @return Shared pointer to created platform
	 * @see Platform
	 */
	std::shared_ptr<Platform> createPlatformFromXml(
	    tinyxml2::XMLElement* platformNode
    );
	/**
	 * @brief Create platform settings from given XML element (node)
	 * @param node XML element (node) containing platform settings data
	 * @return Shared pointer to created platform settings
	 * @see PlatformSettings
	 */
	std::shared_ptr<PlatformSettings> createPlatformSettingsFromXml(
	    tinyxml2::XMLElement* node
    );
	/**
	 * @brief Create scanner from given XML element (node)
	 * @param scannerNode XML element (node) containing scanner data
	 * @return Shared pointer to created scanner
	 * @see Scanner
	 */
	std::shared_ptr<Scanner> createScannerFromXml(
	    tinyxml2::XMLElement* scannerNode
    );
	/**
	 * @brief Create scanner settings from given XML element (node)
	 * @param node XML element (node) containing scanner settings data
	 * @return Shared pointer to created scanner settings
	 * @see ScannerSettings
	 */
	std::shared_ptr<ScannerSettings> createScannerSettingsFromXml(
	    tinyxml2::XMLElement* node
    );
	/**
	 * @brief Create FWF settings from given XML element (node)
	 * @param node XML element (node) containing FWF settings data
	 * @param settings Specify the FWFSettings instance to use as output.
	 * If nullptr is specified, then a new instance of FWFSettings is used
	 * @return Shared pointer to created FWF settings
	 * @see FWFSettings
	 */
	std::shared_ptr<FWFSettings> createFWFSettingsFromXml(
	    tinyxml2::XMLElement* node,
	    std::shared_ptr<FWFSettings> settings = nullptr
    );
	/**
	 * @brief Create scene from given XML element (node)
	 * @param sceneNode XML element (node) containing scene data
	 * @param path Path to scene file
	 * @return Shared pointer to created scene
	 * @see Scene
	 */
	std::shared_ptr<Scene> createSceneFromXml(
	    tinyxml2::XMLElement* sceneNode,
	    std::string path
    );
	/**
	 * @brief Create a rotation from given XML element (node)
	 * @param rotGroupNode XML element (node) containing rotation data
	 * @return Created rotation
	 * @see Rotation
	 */
	Rotation createRotationFromXml(
	    tinyxml2::XMLElement* rotGroupNode
    );
	/**
	 * @brief Create a 3D vector from given XML element (node)
	 * @param node XML element (node) containing 3D vector data
	 * @param attrPrefix Attribute prefix. It will be used so x component is
	 * "attrPrefix" + "x" and so on for y and z components too.
	 * @return Created 3D vector
	 * @see glm::dvec3
	 */
	glm::dvec3 createVec3dFromXml(
	    tinyxml2::XMLElement* node,
	    std::string attrPrefix
    );
	/**
	 * @brief Create a noise source from given XML element (node)
	 * @param noise XML element (node) containing noise source specification
	 * @return Shared pointer to created noise source
	 * @see NoiseSource
	 */
    static std::shared_ptr<NoiseSource<double>>
    createNoiseSource(
        tinyxml2::XMLElement *noise
    );

};