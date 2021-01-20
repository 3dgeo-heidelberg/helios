#pragma once

#include <string>

#include <boost/serialization/base_object.hpp>

/**
 * @brief Base class for all assets
 */
class Asset {

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)	{
		ar & id;
		ar & name;
		ar & sourceFilePath;
	}

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
	std::string getLocationString() {
		return sourceFilePath + "#" + id;
	}
};