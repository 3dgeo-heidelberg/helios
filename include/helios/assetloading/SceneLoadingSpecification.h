#pragma once
#include <helios/assetloading/geometryfilter/RotateFilter.h>
#include <helios/assetloading/geometryfilter/ScaleFilter.h>
#include <helios/assetloading/geometryfilter/TranslateFilter.h>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Manually specify scene transformation filters to apply when loading
 * a scene.
 *
 * SceneLoadingSpecification class is not a substitute for XML specified
 * filters but a different approach to filter specification. It was designed
 * with pyhelios scripting in mind.
 */
class SceneLoadingSpecification
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Rotations to be applied
   * @see RotateFilter
   */
  std::vector<RotateFilter> rotations;
  /**
   * @brief Specify the id of the scene part to apply the rotation over.
   * Specifying an empty string means it will be applied to all scene parts
   */
  std::vector<std::string> rotationsId;
  /**
   * @brief Scales to be applied
   * @see ScaleFilter
   */
  std::vector<ScaleFilter> scales;
  /**
   * @brief Specify the id of the scene part to apply scaling over.
   * Specifying an empty string means it will be applied to all scene parts
   */
  std::vector<std::string> scalesId;
  /**
   * @brief Translations to be applied
   * @see TranslateFilter
   */
  std::vector<TranslateFilter> translations;
  /**
   * @brief Specify the id of the scene part to apply translation over.
   * Specifying an empty string means it will be applied to all scene parts
   */
  std::vector<std::string> translationsId;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build default scene loading specification
   */
  SceneLoadingSpecification() = default;
  virtual ~SceneLoadingSpecification() = default;

  // ***  A P P L Y  *** //
  // ******************* //
  /**
   * @brief Apply the scene loading specification to given scene part
   * @param sp Scene part to apply scene loading specification over
   */
  void apply(std::shared_ptr<ScenePart> sp);
};
