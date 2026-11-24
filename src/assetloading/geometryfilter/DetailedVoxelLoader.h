#pragma once

#include "AbstractGeometryFilter.h"
#include <string>
#include <vector>

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
   * @param discardNullPad If true, detailed voxels with PadBVTotal == 0 or
   *  NaN will be discarded. Used in transmittive mode, where such voxels
   *  never produce a hit.
   */
  void loadDv(std::string const& pathString, bool const discardNullPad);

  /**
   * @brief Load specified material for Detailed Voxels. If no material
   * was specified, then none is loaded.
   */
  void loadMaterial();

  /**
   * @brief Load the leaf angle distribution for Detailed Voxels given by
   * the "lad" parameter (a named type or a file path). If none was
   * specified, the one declared in the voxel file header (#lad_type) is
   * used. If the header declares none or an unsupported one, none is
   * loaded and the default one (spherical) is used.
   * @param filePaths Paths of the loaded voxel files
   * @throws HeliosException if the removed "ladlut" parameter is given
   */
  void loadLeafAngleDistribution(std::vector<std::string> const& filePaths);

  /**
   * @brief Read the leaf angle distribution type (#lad_type) from the
   * header of a voxel file
   * @return The type as written in the file, or an empty string if the
   * header does not declare one
   */
  static std::string readLadType(std::string const& path);
};
