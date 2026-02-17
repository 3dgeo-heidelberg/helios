#pragma once

#include <armadillo>

#include <helios/assetloading/geometryfilter/IVoxelGrid.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dense voxel grid implementation of IVoxelGrid. It uses the indices
 *  of an array to represent the (key, value) pairs as (index, voxel).
 *
 * @see IVoxelGrid
 */
class DenseVoxelGrid : public IVoxelGrid
{
protected:
  // ***   ATTRIBUTES   *** //
  // ********************** //
  /**
   * @brief Array of voxels.
   * @see VoxelGridCell
   * @see Voxel
   */
  VoxelGridCell* voxels;
  /**
   * @brief Used internally to track the current iteration of the while loop.
   */
  std::size_t whileLoopIter;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for a dense voxel grid.
   * @see IVoxelGrid::IVoxelGrid
   */
  DenseVoxelGrid(std::size_t const maxNVoxels);
  virtual ~DenseVoxelGrid() { release(); }

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
