#pragma once

#include <helios/assetloading/geometryfilter/IVoxelGrid.h>
#include <unordered_map>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Sparse voxel grid implementation of IVoxelGrid. It uses an explicit
 *  (key, value) map to store a sparse representation of the voxels.
 *
 * @see IVoxelGrid
 */
class SparseVoxelGrid : public IVoxelGrid
{
protected:
  // ***   ATTRIBUTES   *** //
  // ********************** //
  /**
   * @brief Map of voxels.
   * @see VoxelGridCell
   * @see Voxel
   */
  std::unordered_map<std::size_t, VoxelGridCell> map;
  /**
   * @brief Internal iterator to track the while loop.
   */
  std::unordered_map<std::size_t, VoxelGridCell>::iterator whileLoopIter;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for a sparse voxel grid.
   * @see IVoxelGrid::IVoxelGrid
   */
  SparseVoxelGrid(std::size_t const maxNVoxels);
  virtual ~SparseVoxelGrid() { release(); }

  // ***  VOXEL GRID INTERFACE  *** //
  // ****************************** //
  /**
   * @see IVoxelGrid::release
   */
  void release() override;

  /**
   * @see IVoxelGrid::hasVoxel
   */
  bool hasVoxel(std::size_t const key) override;
  /**
   * @see IVoxelGrid::getVoxel
   */
  Voxel* getVoxel(std::size_t const key) override;
  /**
   * @see IVoxelGrid::setVoxel
   */
  Voxel* setVoxel(std::size_t const key,
                  double const x,
                  double const y,
                  double const z,
                  double halfVoxelSize) override;
  /**
   * @see IVoxelGrid::deleteVoxel
   */
  void deleteVoxel(std::size_t const key) override;

  /**
   * @see IVoxelGrid::getMatrix
   */
  arma::Mat<double>* getMatrix(std::size_t const key) const override;
  /**
   * @see IVoxelGrid::getMatrixRef
   */
  arma::Mat<double>& getMatrixRef(std::size_t const key) const override;
  /**
   * @see IVoxelGrid::getMatrixConstRef
   */
  arma::Mat<double> const& getMatrixConstRef(
    std::size_t const key) const override;
  /**
   * @see IVoxelGrid::setMatrix
   */
  void setMatrix(std::size_t const key, arma::Mat<double>* mat) override;
  /**
   * @see IVoxelGrid::setNextMatrixCol
   */
  void setNextMatrixCol(std::size_t const key,
                        double const x,
                        double const y,
                        double const z) override;
  /**
   * @see IVoxelGrid::deleteMatrix
   */
  void deleteMatrix(std::size_t const key) override;
  /**
   * @see IVoxelGrid::deleteMatrices
   */
  void deleteMatrices() override;

  /**
   * @see IVoxelGrid::getCursor
   */
  std::size_t getCursor(std::size_t const key) const override;
  /**
   * @see IVoxelGrid::setCursor
   */
  void setCursor(std::size_t const key, std::size_t const cursor) override;
  /**
   * @see IVoxelGrid::incrementCursor
   */
  void incrementCursor(std::size_t const key) override;

  /**
   * @see IVoxelGird::getClosestPointDistance
   */
  double getClosestPointDistance(std::size_t const key) const override;
  /**
   * @see IVoxelGrid::setClosestPointDistance
   */
  void setClosestPointDistance(std::size_t const key,
                               double const distance) override;

  // ***   WHILE INTERFACE   *** //
  // *************************** //
  /**
   * @see IVoxelGrid::whileLoopStart
   */
  void whileLoopStart() override;
  /**
   * @see IVoxelGrid::whileLoopHasNext
   */
  bool whileLoopHasNext() override;
  /**
   * @see IVoxelGrid::whileLoopNext
   */
  Voxel* whileLoopNext(std::size_t* key = nullptr) override;
};
