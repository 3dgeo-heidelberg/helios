#pragma once

#include <glm/glm.hpp>
#include <helios/assetloading/geometryfilter/AbstractGeometryFilter.h>

/**
 * @brief Translate transform filter
 */
class TranslateFilter : public AbstractGeometryFilter
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Specify if use local translation (true) or translation from
   * parsed parameters (false, by default)
   */
  bool useLocalTranslation = false;
  /**
   * @brief Local translation specification
   */
  glm::dvec3 localTranslation;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for translate transform filter
   * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
   */
  TranslateFilter(ScenePart* parts)
    : AbstractGeometryFilter(parts)
  {
  }

  // ***  R U N  *** //
  // *************** //
  /**
   * @see AbstractGeometryFilter::run
   */
  ScenePart* run() override;
};
