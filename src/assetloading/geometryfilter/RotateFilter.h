#pragma once

#include "AbstractGeometryFilter.h"

/**
 * @brief Rotate transform filter
 */
class RotateFilter : public AbstractGeometryFilter {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Specify if use local rotation (true) or rotation from parsed
     * parameters (false, by default)
     */
    bool useLocalRotation = false;
    /**
     * @brief Local rotation specification
     */
    Rotation localRotation;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for rotate transform filter
     * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
     */
	RotateFilter(ScenePart* parts) : AbstractGeometryFilter(parts) {}

	// ***  R U N  *** //
	// *************** //
	/**
	 * @see AbstractGeometryFilter::run
	 */
	ScenePart* run() override;
};