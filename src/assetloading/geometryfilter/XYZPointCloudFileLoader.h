#pragma once

#include "AbstractGeometryFilter.h"
#include "Voxel.h"
#include <assetloading/geometryfilter/IVoxelGrid.h>

#include <armadillo>
#include <fstream>
#include <string>

/**
 * @brief Import point cloud files abstracting them to a set of voxels
 */
class XYZPointCloudFileLoader : public AbstractGeometryFilter
{
public:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * @brief How many points are necessary for the normal estimation to be
   * reliable.
   */
  static int const minPointsForSafeNormalEstimation = 3;
  /**
   * @brief How many points consider per batch when estimating normals
   */
  static size_t const batchSize = 10000000;
  /**
   * @brief How many points can be contained inside a voxel without
   * triggering a warning due to excessive population
   */
  static int const voxelPopulationThreshold = 40000;

private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Field separator specification used by the point cloud file
   */
  std::string separator = " ";
  /**
   * @brief Size for each voxel
   */
  double voxelSize = 1;
  /**
   * @brief Max value with respect to used color codification
   */
  double maxColorValue = 0;

  /**
   * @brief Snap neighbor normal instead of average when enabled (true).
   */
  bool snapNeighborNormal = false;
  /**
   * @brief Specify whether default normal must be assigned to voxels which
   * normal cannot be safely determined (True) or not (False).
   */
  bool assignDefaultNormal = false;
  /**
   * @brief The default normal to be used when needed to fill missing ones.
   */
  glm::dvec3 defaultNormal = glm::dvec3(0, 0, 0);
  /**
   * @brief How many voxels normals could not be safely estimated because
   * there was not enough data inside the voxel.
   */
  std::size_t unsafeNormalEstimations = 0;
  /**
   * @brief How many points were discarded because they were expected to
   * have a correct normal but they did not.
   * @see XYZPointCloudFileLoader::correctNormal
   */
  std::size_t discardedPointsByNormal = 0;

  /**
   * @brief The number of points in the point cloud
   */
  std::size_t n;
  /**
   * @brief Minimum X coordinate considering all points
   */
  double minX;
  /**
   * @brief Minimum Y coordinate considering all points
   */
  double minY;
  /**
   * @brief Minimum Z coordinate considering all points
   */
  double minZ;
  /**
   * @brief Maximum X coordinate considering all points
   */
  double maxX;
  /**
   * @brief Maximum Y coordinate considering all points
   */
  double maxY;
  /**
   * @brief Maximum Z coordinate considering all points
   */
  double maxZ;

  /**
   * @brief Number of partitions along x-axis
   */
  std::size_t nx;
  /**
   * @brief Number of partitions along y-axis
   */
  std::size_t ny;
  /**
   * @brief Number of partitions along z-axis
   */
  std::size_t nz;
  /**
   * @brief Product \f$ny \cdot nz\f$. Stored in a variable because of
   * its recurrent usage.
   */
  std::size_t nynz;

  /**
   * @brief Coefficient (\f$K_{x}\f$) to compute \f$I\f$ voxel index for a
   * given \f$x\f$ coordinate
   *
   * \f[
   * I = (x - x_{min}) \cdot K_{x}
   * \f]
   */
  double xCoeff;
  /**
   * @brief Coefficient (\f$K_{y}\f$) to compute \f$J\f$ voxel index for a
   * given \f$y\f$ coordinate
   *
   * \f[
   * J = (y - y_{min}) \cdot K_{y}
   * \f]
   */
  double yCoeff;
  /**
   * @brief Coefficient (\f$K_{z}\f$) to compute \f$K\f$ voxel index for a
   * given \f$z\f$ coordinate
   *
   * \f[
   * K = (z - z_{min}) \cdot K_{z}
   * \f]
   */
  double zCoeff;

  /**
   * @brief The voxel grid used to load a representation of the input point
   *  cloud.
   * @see IVoxelGrid
   * @see DenseVoxelGrid
   * @see SparseVoxelGrid
   */
  IVoxelGrid* voxelGrid = nullptr;
  /**
   * @brief Total size of full voxels grid
   * @see XYZPointCloudFileLoader::voxels
   */
  std::size_t maxNVoxels;
  /**
   * @brief How many batches are necessary to estimate normals
   */
  std::size_t numBatches;

  /**
   * @brief Used to correctly report number of voxels for each part
   * when reading multiple files at once (i.e. efilepath is given)
   */
  std::size_t lastNumVoxels;

  // ***  MAIN PARSING METHODS  *** //
  // ****************************** //
  /**
   * @brief Parse XYZ file
   * @param filePath Path to XYZ file to be parsed
   */
  void parse(std::string const& filePath);

  /**
   * @brief First pass of input file used to find essential information
   * needed to successfully abstract the point cloud to voxels
   * @param filePathString Path to the input file
   * @param ifs Stream used to read from input file
   */
  void firstPass(std::string const& filePathString, std::ifstream& ifs);
  /**
   * @brief Second pass where the input file is read as many times as needed
   * to build necessary voxels
   * @param filePathString Path to the input file
   * @param matName Name of the material to be used for voxels
   * @param ifs Stream used to read from input file
   */
  void secondPass(std::string const& filePathString,
                  std::string const& matName,
                  std::ifstream& ifs);
  /**
   * @brief Load the material for each primitive in a cyclic fashion.
   * It is, if \f$m\f$ materials are given then the \f$i\f$-th voxel
   *  is associated with the \f$i \mod m\f$ material.
   */
  void loadMaterial();

  // ***  AUXILIAR PARSING METHODS  *** //
  // ********************************** //
  /**
   * @brief Prepare voxels grid computation
   * @param[out] estimateNormals Used to return normals estimation
   * specification
   * @param[out] halfVoxelSize Used to return half of the voxel size
   */
  void prepareVoxelsGrid(int& estimateNormals, double& halfVoxelSize);
  /**
   * @brief Fill voxels grid
   * @param estimateNormals Used to specified normal estimation method.
   * A value of 0 means no normal estimation, a value of 1 means normal
   * estimation will be performed entirely in memory and a value of 2
   * means normal estimation will be performed through multiple runs over
   * the input file
   */
  void fillVoxelsGrid(std::ifstream& ifs,
                      int estimateNormals,
                      double halfVoxelSize,
                      std::string const& filePathString);
  /**
   * @brief Correct normal if necessary. Only non valid normals will be
   * corrected
   * @param x Normal x component
   * @param y Normal y component
   * @param z Normal z component
   * @return True if the voxel normal is valid, False otherwise.
   */
  bool correctNormal(double& x, double& y, double& z);
  /**
   * @brief Digest a voxel when filling voxels grid. This implies the voxel
   * will be created in the grid if it does not exist. If the voxel already
   * existed, then it will be populated with given new data.
   */
  void digestVoxel(int estimateNormals,
                   double halfVoxelSize,
                   double x,
                   double y,
                   double z,
                   double r,
                   double g,
                   double b,
                   double xnorm,
                   double ynorm,
                   double znorm);

  /**
   * @brief Some issues with normals might be solved during post processing.
   *  This function is called before XYZPointCloudFileLoader::postProcess
   *  to find possible errors and report them before they are automatically
   *  treated during post processing.
   * @param filePathString The path to the file where XYZ points are
   *  specified
   * @see XYZPointCloudFileLoader::postProcess
   */
  void warnAboutPotentialErrors(std::string const& filePathString);

  /**
   * @brief Post process already filled voxels grid
   * @param estimateNormals Used to specify if normals must be estimated (>0)
   * or not (0)
   */
  void postProcess(std::string const& matName, int estimateNormals);
  /**
   * @brief Estimate voxels normal as the orthonormal of best fitting plane
   * for points inside voxel
   */
  void estimateNormals(std::ifstream& ifs);
  /**
   * @brief Estimate voxels normals in batch mode, which implies reading
   * input file as many times as specified by numBatches variable.
   * The output generated by this function is the same than the one that
   * should be generated by estimateNormals function
   * @see XYZPointCloudFileLoader::estimateNormals
   * @see XYZPointCloudFileLoader::numBatches
   */
  void estimateNormalsBatch(std::ifstream& ifs);
  /**
   * @brief Assists estimateNormals function. Normals for filled matrices
   * are computed at this function
   *
   * @param start Specifies the start index in the voxel grid
   * @param end Specified the end index in the voxel grid
   */
  void _estimateNormals(std::size_t start, std::size_t end);
  /**
   * @brief Compose the scene part considering voxels at voxels grid
   */
  void voxelsGridToScenePart();

  /**
   * @brief Obtain voxel-grid index for given coordinates
   * @param[out] I Used to output I index (x axis)
   * @param[out] J Used to output J index (y axis)
   * @param[out] K Used to output K index (z axis)
   * @return Voxel-grid index
   */
  std::size_t indexFromCoordinates(double x,
                                   double y,
                                   double z,
                                   std::size_t& I,
                                   std::size_t& J,
                                   std::size_t& K);

  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Check if given line is a comment or not
   * @param line Line to be checked
   * @return True if line is a comment, false otherwise
   */
  static bool isLineComment(std::string const& line);
  /**
   * @brief Reset given input file stream so it points to file start
   * @param ifs Input file stream to be resetted.
   */
  static void restartInputFileStream(std::ifstream& ifs);

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for point cloud loader
   * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
   */
  explicit XYZPointCloudFileLoader()
    : AbstractGeometryFilter(new ScenePart())
  {
  }
  virtual ~XYZPointCloudFileLoader() = default;

  // ***  MAIN METHODS  *** //
  // ********************** //
  /**
   * @see AbstractGeometryFilter::run
   */
  ScenePart* run() override;
};
