#pragma once

#include <string>

/**
 * @brief Base class for all assets
 */
class Asset
{
  // *********************** //
private:
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Asset identifier
   */
  std::string id = "";
  /**
   * @brief Asset name
   */
  std::string name = "Unnamed Asset";
  /**
   * @brief Path to asset file
   */
  std::string sourceFilePath = "";

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~Asset() {}

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain asset location string
   * @return Asset location string
   */
  std::string getLocationString() { return sourceFilePath + "#" + id; }
  /**
   * @brief Check whether the asset is an EggAsset or not
   * @return True if the asset is an EggAsset, false otherwise
   * @see EggAsset
   */
  virtual bool isEgg() const { return false; }
};
