#pragma once

#include <helios/assetloading/geometryfilter/AbstractGeometryFilter.h>

/**
 * @brief Scale transform filter
 */
class ScaleFilter : public AbstractGeometryFilter
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Specify if use local scale factor (true) or scale factor from
   * parsed parameters (false by default)
   */
  bool useLocalScaleFactor = false;
  /**
   * @brief Local scale factor specification
   */
  double localScaleFactor;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for scale transform filter
   * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
   */
  explicit ScaleFilter(ScenePart* parts)
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
