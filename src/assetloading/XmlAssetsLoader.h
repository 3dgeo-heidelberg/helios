#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <tinyxml2.h>

#include "typedef.h"

#include "Asset.h"
#include "Color4f.h"
#include "Platform.h"
#include "PlatformSettings.h"
#include "Scanner.h"
#include <XmlSceneLoader.h>

/**
 * @brief Class for asset loading from XML file.
 *
 * It is the main class for XML loading. It uses both XmlSceneLoader and
 *  XmlUtils
 *
 * @see XmlSceneLoader
 * @see XmlUtils
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

	/**
	 * @brief Map containing all scanner templates that were loading while
	 *  building from XML file. No repetitions, each template appears only
	 *  one time.
	 *
	 * The id of the template is used as the key, while the template itself
	 *  is the value (ScannerSettings object)
	 */
	std::unordered_map<std::string, std::shared_ptr<ScannerSettings>>
	    scannerTemplates;

public:
    /**
	 * @brief The scene loader. It is used to load scenes from XML files
	 */
    XmlSceneLoader sceneLoader;
    /**
     * @brief XML file through tinyxml2 library
     */
	tinyxml2::XMLDocument doc;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Build a XmlAssetsLoader for given XML file and assets directory
	 * @param filePath Path to XML File
	 * @param assetsDir Path to assets directory
	 */
	XmlAssetsLoader(std::string& filePath, std::string& assetsDir);
	virtual ~XmlAssetsLoader() {}

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
};