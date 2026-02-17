#pragma once

#include <helios/assetloading/geometryfilter/AbstractGeometryFilter.h>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Loader for deatiled voxel geometries
 */
class DetailedVoxelLoader : public AbstractGeometryFilter
{
public:
  // *** CONSTRUCTION  *** //
  // ********************* //
  /**
   * @brief Constructor for detailed voxel loader
   * @see DetailedVoxel
   * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
   */
  DetailedVoxelLoader()
    : AbstractGeometryFilter(new ScenePart())
  {
  }

  // ***  R U N  *** //
  // *************** //
  /**
   * @see AbstractGeometryFilter::run
   */
  ScenePart* run() override;

  /**
   * @brief Load a Detailed Voxels file
   * @param discardNullPad If true, detailed voxels with PadBVTotal==0 will
   *  be discarded (it is useful, for instance, when using transmittive mode)
   */
  void loadDv(std::string const& pathString, bool const discardNullPad);

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
