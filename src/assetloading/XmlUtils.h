#pragma once

#include <Color4f.h>
#include <typedef.h>
#include <NoiseSource.h>
#include <scene/dynamic/DynSequence.h>
#include <maths/rigidmotion/RigidMotion.h>
#include <DynMotion.h>

#include <tinyxml2.h>

#include <map>
#include <vector>

using namespace rigidmotion;

/**
 * @brief Common utils for XML handling
 */
class XmlUtils{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    XmlUtils() = delete;

    // ***  STATIC METHODS  *** //
    // ************************ //
    /**
	 * @brief Create a color from given XML element (node)
	 * @param node XML element (node) containing color data
	 * @return Created color
	 * @see Color4f
	 */
    static Color4f createColorFromXml(tinyxml2::XMLElement* node);

    /**
	 * @brief Create a map of parameters from given XML element (node)
	 * @param paramsNode XML element (node) containing parameters
	 * @return Map with parameters, so each one is identified by a different
	 * string
	 */
    static std::map<std::string, ObjectT> createParamsFromXml(
        tinyxml2::XMLElement* paramsNode
    );
    /**
	 * @brief Create a rotation from given XML element (node)
	 * @param rotGroupNode XML element (node) containing rotation data
	 * @return Created rotation
	 * @see Rotation
	 */
    static Rotation createRotationFromXml(tinyxml2::XMLElement* rotGroupNode);
    /**
	 * @brief Create a 3D vector from given XML element (node)
	 * @param node XML element (node) containing 3D vector data
	 * @param attrPrefix Attribute prefix. It will be used so x component is
	 * "attrPrefix" + "x" and so on for y and z components too.
	 * @return Created 3D vector
	 * @see glm::dvec3
	 */
    static glm::dvec3 createVec3dFromXml(
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
    createNoiseSource(tinyxml2::XMLElement *noise);

    /**
     * @brief Obtain attribute from XML
     * @param element XML element (node) where the attribute must be taken from
     * @param attrName Name of the attribute to be obtained
     * @param type Type of the attribute to be obtained
     * @param defaultVal Default value to be used in case attribute was not
     * found
     * @return Obtained attribute or default value if attribute was not found
     */
    static ObjectT getAttribute(
        tinyxml2::XMLElement* element,
        std::string attrName,
        std::string type,
        ObjectT defaultVal
    );

    /**
     * @brief Obtain a dynamic sequence of dynamic motions from given XML
     *  element.
     *
     * It is expected that given element contains children elements of type
     *  <motion type="motion_type"/> where motion_type can be either
     *  translation, rotation, rotsym, reflection, glideplane or helical.
     * These children element will be used to build the dynamic sequence of
     *  dynamic motions with the same order as the one in the XML.
     *
     * @param element XML element containing motion children elements
     * @return Vector of dynamic motions built from XML
     * @see DynSequencer
     * @see DynSequence
     * @see rigidmotion::RigidMotion
     * @see DynMotion
     */
    static std::vector<std::shared_ptr<DynMotion>> createDynMotionsVector(
        tinyxml2::XMLElement *element
    );
};