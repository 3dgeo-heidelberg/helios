#pragma once

#include <map>
#include <string>

#include "Material.h"

/**
 * @brief Class to read materials files
 */
class MaterialsFileReader {
public:
    /**
     * @brief Static method to load materials from given file
     * @param filePathString Path to materials file
     * @return Parsed materials
     */
	static std::map<std::string, Material> loadMaterials(
	    std::string filePathString
    );
};