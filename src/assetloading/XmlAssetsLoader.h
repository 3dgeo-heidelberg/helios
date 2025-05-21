#pragma once

#include "Asset.h"
#include "Color4f.h"
#include "Platform.h"
#include "PlatformSettings.h"
#include "Scanner.h"
#include "typedef.h"
#include <XmlSceneLoader.h>
#include <platform/trajectory/TrajectorySettings.h>

#include <tinyxml2.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fs = boost::filesystem;

/**
 * @brief Class for asset loading from XML file.
 *
 * It is the main class for XML loading. It uses both XmlSceneLoader and
 *  XmlUtils
 *
 * @see XmlSceneLoader
 * @see XmlUtils
 */
class XmlAssetsLoader
{
protected:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * @brief The message to be shown when default value is loaded for scanner
   *  settings
   */
  static std::string const defaultScannerSettingsMsg;
  /**
   * @brief The message to be shown when default value is loaded for platform
   *  settings
   */
  static std::string const defaultPlatformSettingsMsg;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Assets directory
   */
  std::vector<std::string> assetsDir;
  /**
   * @brief Name of the XML file
   */
  std::string xmlDocFilename = "unknown.xml";
  /**
   * @brief Path to the XML file
   */
  std::string xmlDocFilePath = "";

  /**
   * @brief The default scanner settings template
   */
  std::shared_ptr<ScannerSettings> defaultScannerTemplate;
  /**
   * @brief The default platform settings template
   */
  std::shared_ptr<PlatformSettings> defaultPlatformTemplate;

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
  /**
   * @brief Map containing all platform templates that were loading while
   *  building from XML file. No repetitions, each template appears only
   *  one time.
   *
   * The id of the template is used as the key, while the template itself
   *  is the value (PlatformSettings object)
   */
  std::unordered_map<std::string, std::shared_ptr<PlatformSettings>>
    platformTemplates;
  /**
   * @brief Map containing the set of fields that have been overloaded by
   *  each scanner template
   *
   * The id of the template is used as the key, while the set of fields
   *  itself is the value
   * @see XmlAssetsLoader::scannerTemplates
   */
  std::unordered_map<std::string, std::unordered_set<std::string>>
    scannerTemplatesFields;
  /**
   * @brief Map containing the set of fields that have been overloaded by
   *  each platform template
   *
   * The id of the template is used as the key, while the set of fields
   *  itself is the value
   * @see XmlAssetsLoader::platformTemplates
   */
  std::unordered_map<std::string, std::unordered_set<std::string>>
    platformTemplatesFields;

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
  XmlAssetsLoader(std::string& filePath, std::vector<std::string>& assetsDir);
  virtual ~XmlAssetsLoader() {}

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Get asset by its identifier
   * @param type Type of the asset
   * @param id Identifier of the asset
   * @param extraOutput[out] Pointer to where extra output must be stored.
   *  If it is null, it means no extra output is required. Notice this only
   *  will be used when requested asset supports extra output
   * @return Shared pointer to requested asset
   */
  std::shared_ptr<Asset> getAssetById(std::string type,
                                      std::string id,
                                      void* extraOutput = nullptr);
  /**
   * @brief Get asset by location
   * @param type Type of the asset
   * @param location Location of the asset
   * @param extraOutput[out] Pointer to where extra output must be stored.
   *  If it is null, it means no extra output is required. Notice this only
   *  will be used when requested asset supports extra output
   * @return Shared pointer to requested asset
   */
  std::shared_ptr<Asset> getAssetByLocation(std::string type,
                                            std::string location,
                                            void* extraOutput = nullptr);

  // ***  CREATION METHODS  *** //
  // ************************** //
  /**
   * @brief Create an asset from given XML element (node)
   * @param type Asset type
   * @param assetNode XML element (node) containing asset data
   * @param extraOutput[out] Pointer to where extra output must be stored.
   *  If it is null, it means no extra output is required. Notice this only
   *  will be used when requested asset supports extra output
   * @return Shared pointer to created asset
   * @see Asset
   */
  std::shared_ptr<Asset> createAssetFromXml(std::string type,
                                            tinyxml2::XMLElement* assetNode,
                                            void* extraOutput = nullptr);
  /**
   * @brief Like XmlAssetsLoader::createAssetFromXml but creating the asset
   *  procedurally instead of simply loading it
   * @see XmlAssetsLoader::createAssetFromXml
   * @see XmlAssetsLoader::isProceduralAsset
   */
  std::shared_ptr<Asset> createProceduralAssetFromXml(
    std::string const& type,
    std::string const& id,
    void* extraOutput = nullptr);

  /**
   * @brief Create a platform from given XML element (node)
   * @param platformNode XML element (node) containing platform data
   * @return Shared pointer to created platform
   * @see Platform
   */
  std::shared_ptr<Platform> createPlatformFromXml(
    tinyxml2::XMLElement* platformNode);
  /**
   * @brief Create platform settings from given XML element (node)
   * @param node XML element (node) containing platform settings data
   * @return Shared pointer to created platform settings
   * @see PlatformSettings
   */
  std::shared_ptr<PlatformSettings> createPlatformSettingsFromXml(
    tinyxml2::XMLElement* node,
    std::unordered_set<std::string>* fields = nullptr);
  /**
   * @brief Procedurally create the platform from given XML specification
   * @param type Asset type
   * @param id Identifier of the asset
   * @return Shared pointer to created platform
   * @see Platform
   * @see InterpolatedMovingPlatform
   * @see XmlAssetsLoader::createInterpolatedMovingPlatform
   */
  std::shared_ptr<Platform> procedurallyCreatePlatformFromXml(
    string const& type,
    string const& id);
  /**
   * @brief Procedurally create an InterpolatedMovingPlatform
   * @return Procedurally created InterpolatedMovingPlatform
   * @see XmlAssetsLoader::procedurallyCreatePlatformFromXml
   * @see InterpolatedMovingPlatform
   */
  std::shared_ptr<Platform> createInterpolatedMovingPlatform();

  /**
   * @brief Create scanner from given XML element (node)
   * @param scannerNode XML element (node) containing scanner data
   * @return Shared pointer to created scanner
   * @see Scanner
   */
  std::shared_ptr<Scanner> createScannerFromXml(
    tinyxml2::XMLElement* scannerNode);

  /**
   * @brief Create a beam deflector from given XML element (node)
   * @param scannerNode XML element (node) containing deflector data
   * @return Shared pointer to created beam deflector
   * @see AbstractBeamDeflector
   */
  std::shared_ptr<AbstractBeamDeflector> createBeamDeflectorFromXml(
    tinyxml2::XMLElement* scannerNode);

  /**
   * @brief Create a pulse detector from given XML element (node)
   * @param scannerNode XML element (node) containing deflector data
   * @param scanner The shared pointer to the scanner associated to the
   *  detector
   * @return Shared pointer to created detector
   * @see AbstractDetector
   */
  std::shared_ptr<AbstractDetector> createDetectorFromXml(
    tinyxml2::XMLElement* scannerNode,
    std::shared_ptr<Scanner> scanner);

  /**
   * @brief Create a scanner head from given XML element (node)
   * @param scannerNode XML element (node) containing scanner head data
   * @return Shared pointer to created scanner head
   * @see ScannerHead
   */
  std::shared_ptr<ScannerHead> createScannerHeadFromXml(
    tinyxml2::XMLElement* scannerNode);

  /**
   * @brief Create scanner settings from given XML element (node)
   * @param node XML element (node) containing scanner settings data
   * @param[out] fields When it is not a nullptr, names of read values will
   *  be stored here
   * @return Shared pointer to created scanner settings
   * @see ScannerSettings
   */
  std::shared_ptr<ScannerSettings> createScannerSettingsFromXml(
    tinyxml2::XMLElement* node,
    std::unordered_set<std::string>* fields = nullptr);
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
    std::shared_ptr<FWFSettings> settings = nullptr);
  /**
   * @brief Create TrajectorySettings from given XML element (node)
   * @param legNode XML element (node) containing the Leg specification to
   *  build TrajectorySettings from
   * @param settings Specify the TrajectorySettings instance to use as
   *  output. If nullptr is specified, then a new instance of
   *  TrajectorySettings is used
   * @return Shared pointer to created TrajectorySettings
   * @see TrajectorySettings
   */
  std::shared_ptr<TrajectorySettings> createTrajectorySettingsFromXml(
    tinyxml2::XMLElement* legNode,
    std::shared_ptr<TrajectorySettings> settings = nullptr);

  /**
   * @brief Fill vector of scanning devices with channel data
   * @param scanner The multi-channel scanner to be filled
   * @param scannerNode The XML element representing the scanner
   * @param channels The channels to fill each scanning device
   * @param deflec The base deflector for each scanning device
   * @param detec The base detector for each scanning device
   * @param scanHead The base scanner head for each scanning device
   * @param fwfSettings The base fullwave settings for each scanning device
   * @see MultiScanner
   */
  void fillScanningDevicesFromChannels(
    std::shared_ptr<Scanner> scanner,
    tinyxml2::XMLElement* scannerNode,
    tinyxml2::XMLElement* channels,
    std::shared_ptr<AbstractBeamDeflector> deflec,
    std::shared_ptr<AbstractDetector> detec,
    std::shared_ptr<ScannerHead> scanHead,
    std::shared_ptr<FWFSettings> fwfSettings);

protected:
  // ***  UTIL METHODS  *** //
  // ********************** //
  /**
   * @brief Reinitialize the loader so maps with scanner fields and scanner
   *  templates and their fields are empty
   */
  virtual void reinitLoader();
  /**
   * @brief
   * @see XmlAssetsLoader::defaultScannerTemplate
   * @see XmlAssetsLoader::defaultPlatformTemplate
   */
  void makeDefaultTemplates();
  /**
   * @brief Track non default values at base. It is, those values which are
   *  in base distinct than the reference (ref)
   * @param base Scanner settings to be tracked
   * @param ref Reference scanner settings defining default values
   * @param defaultTemplateId The identifier of reference/default template
   * @param fields Where tracked non default values must be stored
   */
  void trackNonDefaultScannerSettings(std::shared_ptr<ScannerSettings> base,
                                      std::shared_ptr<ScannerSettings> ref,
                                      std::string const defaultTemplateId,
                                      std::unordered_set<std::string>& fields);
  /**
   * @brief Track non default values at base. It is, those values which are
   *  in base distinct than the reference (ref)
   * @param base Platform settings to be tracked
   * @param ref Reference platform settings defining default values
   * @param defaultTemplateId The identifier of reference/default template
   * @param fields Where tracked non default values must be stored
   */
  void trackNonDefaultPlatformSettings(std::shared_ptr<PlatformSettings> base,
                                       std::shared_ptr<PlatformSettings> ref,
                                       std::string const defaultTemplateId,
                                       std::unordered_set<std::string>& fields);

  /**
   * @brief Locate an asset file in the given asset directories
   *
   */
  fs::path locateAssetFile(const std::string& filename);

public:
  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Check whether the given asset is a procedural asset or not
   *
   * A procedural asset is said to be an asset which is generated during
   *  execution. While it can be based in input data, it is not fully defined
   *  by that data. An example of procedural asset would be the
   *  InterpolatedMovingPlatform which is specified through the type
   *  "platform" and the id "interpolated"
   *
   * @param type Type of the asset
   * @param id Identifier of the asset
   * @return True if the asset is a procedural one, false otherwise
   * @see InterpolatedMovingPlatform
   */
  static bool isProceduralAsset(std::string const& type, std::string const& id);
};
