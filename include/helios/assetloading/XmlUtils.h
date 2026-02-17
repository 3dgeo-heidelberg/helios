#ifndef _HELIOS_XMLUTILS_H_
#define _HELIOS_XMLUTILS_H_

#include <helios/adt/exprtree/UnivarExprTreeNode.h>
#include <helios/assetloading/geometryfilter/AbstractGeometryFilter.h>
#include <helios/maths/rigidmotion/RigidMotion.h>
#include <helios/noise/NoiseSource.h>
#include <helios/scene/dynamic/DynMotion.h>
#include <helios/scene/dynamic/DynSequence.h>
#include <helios/util/Color4f.h>
#include <helios/util/typedef.h>

#include <tinyxml2.h>

#include <map>
#include <unordered_map>
#include <vector>

/**
 * @brief Common utils for XML handling
 */
class XmlUtils
{
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
    tinyxml2::XMLElement* paramsNode);
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
  static glm::dvec3 createVec3dFromXml(tinyxml2::XMLElement* node,
                                       std::string attrPrefix);
  /**
   * @brief Create a noise source from given XML element (node)
   * @param noise XML element (node) containing noise source specification
   * @return Shared pointer to created noise source
   * @see NoiseSource
   */
  static std::shared_ptr<NoiseSource<double>> createNoiseSource(
    tinyxml2::XMLElement* noise);

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
    ObjectT defaultVal,
    std::string const defaultMsg = "Using default value for attribute");

  /**
   * @brief Obtain attribute from XML with a given type
   * @param element XML element (node) where the attribute must be taken from
   * @param attrName Name of the attribute to be obtained
   * @param defaultVal Default value to be used in case attribute was not
   * found
   * @return Obtained attribute or default value if attribute was not found,
   * cast to the template type
   */
  template<typename T>
  static T getAttributeCast(
    tinyxml2::XMLElement* element,
    std::string attrName,
    T defaultValue,
    std::string const defaultMsg = "Using default value for attribute")
  {
    std::string type = typenameHelper<T>::name();
    return boost::get<T>(XmlUtils::getAttribute(
      element, attrName, type, defaultValue, defaultMsg));
  }

  /**
   * @brief Check whether XML node contains an attribute with given name
   *  (true) or not (false)
   * @param element XML node to which children must be checked
   * @param attrName Name of the element to be checked
   * @return True if attribute is contained in XML node, false otherwise
   */
  static bool hasAttribute(tinyxml2::XMLElement* element, std::string attrName);

  /**
   * @brief Check whether the given filter loads a geometry (true) or not
   *  (false, e.g., transforming a geometry with rotations).
   * @param filter The filter to be checked.
   * @return True if the given filter loads a geometry, false otherwise.
   */
  static bool isGeometryLoadingFilter(AbstractGeometryFilter* filter);

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
    tinyxml2::XMLElement* element);

  /**
   * @brief Assert whether the given document is valid for asset loading or
   *  not. If it is in an error state, then adequate logging is printed and
   *  the opportune exception is thrown.
   * @param doc The document to be asserted.
   */
  static void assertDocumentForAssetLoading(tinyxml2::XMLDocument& doc,
                                            std::string const& filename,
                                            std::string const& path,
                                            std::string const& type,
                                            std::string const& id,
                                            std::string const& caller);

  /**
   * @brief Create a univariate expression tree from given XML expression
   *  node (must have an "expr" attribute containing the mathematical
   *  expression)
   * @tparam NumericType The numeric type of the expression tree
   * @param exprNode The XML expression node
   * @return Built univariate expression tree
   * @see XmlUtils::createUnivarExprTree(XMLElement *, unordered_map const &)
   */
  template<typename NumericType = double>
  static std::shared_ptr<UnivarExprTreeNode<NumericType>> createUnivarExprTree(
    tinyxml2::XMLElement* exprNode);

  /**
   * @brief Like XmlUtils::createUnivarExprTree(XMLElement *) but applying
   *  the rename map (dictionary) before parsing the expression
   * @param[in] renameMap The <UserFriendly, TreeFriendly> dictionary to
   *  translate between human language and expression tree language.
   * @see XmlUtils::createUnivarExprTree(XMLElement *)
   */
  template<typename NumericType = double>
  static std::shared_ptr<UnivarExprTreeNode<NumericType>> createUnivarExprTree(
    tinyxml2::XMLElement* exprNode,
    std::unordered_map<std::string, std::string> const& renameMap);
};

#include <helios/assetloading/XmlUtils.tpp>
#endif
