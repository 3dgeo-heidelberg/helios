#pragma once

#include "AbstractGeometryFilter.h"

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Loader for deatiled voxel geometries
 */
class DetailedVoxelLoader : public AbstractGeometryFilter {
public:
    // *** CONSTRUCTION  *** //
    // ********************* //
    /**
     * @brief Constructor for detailed voxel loader
     * @see DetailedVoxel
     * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
     */
    DetailedVoxelLoader() : AbstractGeometryFilter(new ScenePart()) {}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see AbstractGeometryFilter::run
     */
    ScenePart* run() override;

    /**
     * @brief Load a Detailed Voxels file
     */
    void loadDv(std::string const & pathString);

    /**
     * @brief Load specified material for Detailed Voxels. If no material
     * was specified, then none is loaded.
     */
    void loadMaterial();

    /**
     * @brief Load specified ladlut for Detailed Voxels. If no ladlut
     * was specified, then none is loaded.
     */
    void loadLadlut();
};
