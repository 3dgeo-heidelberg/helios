#pragma once

#include <helios/util/HeliosException.h>
#include <map>
#include <string>

#include <helios/scene/Material.h>

/**
 * @brief Class to read materials files
 */
class MaterialsFileReader
{
public:
  /**
   * @brief Static method to load materials from given file
   * @param filePathString Path to materials file
   * @return Parsed materials
   */
  static std::map<std::string, std::shared_ptr<Material>> loadMaterials(
    std::string filePathString);
};
