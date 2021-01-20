#pragma once


#include <map>
#include <string>

#include "Material.h"
#include "ScenePart.h"

#include "maths/Rotation.h"

#include "typedef.h"
#include <ogr_spatialref.h>

/**
 * @brief Abstract class defining asset loading filters common behavior
 */
class AbstractGeometryFilter {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
	OGRSpatialReference* sourceCRS;

    /**
     * @brief Available parameters
     */
	std::map<std::string, ObjectT> params;
	/**
	 * @brief Available materials
	 */
	std::map<std::string, Material> materials;

	/**
	 * @brief ScenePart used to build the output. Not strictly necessary for
	 * any filter.
	 */
	ScenePart* primsOut = nullptr;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Base constructor for asset loading filters
	 * @param parts_ Instance to be used as primsOut
	 * @see AbstractGeometryFilter::primsOut
	 */
	AbstractGeometryFilter(ScenePart* parts_) : primsOut(parts_)
	{if(primsOut == nullptr) primsOut = new ScenePart();}
	virtual ~AbstractGeometryFilter()
	{if(primsOut != nullptr) delete primsOut;}

	// ***  R U N  *** //
	// *************** //
	/**
	 * @brief Abstract method to run the filter and obtain the output.
	 * @return Built scene part, if any. In most cases, the primsOut
	 * variable.
	 * @see AbstractGeometryFilter::primsOut
	 */
    virtual ScenePart* run() = 0;


	// ***  GETTERs and SETTERs  *** //
	// ***************************** //
	/**
	 * @brief Retrieve requested material by name
	 * @param materialName Name of material to be retrieved
	 * @return Requested material
	 */
	std::shared_ptr<Material> getMaterial(std::string materialName);

	// ***  U T I L S  *** //
    // ******************* //
    /**
     * @brief Parse materials specified through "matfile", which can be
     * concreted through "matname" parameter. It can also handle random
     * materials when parameter "randomMaterials" is set.
     * @return Vector of shared pointers to parsed materials
     */
    std::vector<std::shared_ptr<Material>> parseMaterials();

};