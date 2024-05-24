#pragma once

#include <assetloading/geometryfilter/IVoxelGrid.h>
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
class SparseVoxelGrid : public IVoxelGrid {
protected:
    // ***   ATTRIBUTES   *** //
    // ********************** //
    /**
     * @brief Map of voxels.
     * @see VoxelGridCell
     * @see Voxel
     */
    std::unordered_map<size_t, VoxelGridCell> map;
    /**
     * @brief Internal iterator to track the while loop.
     */
    std::unordered_map<size_t, VoxelGridCell>::iterator whileLoopIter;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for a sparse voxel grid.
     * @see IVoxelGrid::IVoxelGrid
     */
    SparseVoxelGrid(size_t const maxNVoxels);
    virtual ~SparseVoxelGrid() {release();}

    // ***  VOXEL GRID INTERFACE  *** //
    // ****************************** //
    /**
     * @see IVoxelGrid::release
     */
    void release() override;


    /**
     * @see IVoxelGrid::hasVoxel
     */
    bool hasVoxel(size_t const key) override;
    /**
     * @see IVoxelGrid::getVoxel
     */
    Voxel* getVoxel(size_t const key) override;
    /**
     * @see IVoxelGrid::setVoxel
     */
    Voxel * setVoxel(
        size_t const key,
        double const x, double const y, double const z,
        double halfVoxelSize
    ) override;
    /**
     * @see IVoxelGrid::deleteVoxel
     */
    void deleteVoxel(size_t const key) override;


    /**
     * @see IVoxelGrid::getMatrix
     */
    Mat<double> * getMatrix(size_t const key) const override;
    /**
     * @see IVoxelGrid::getMatrixRef
     */
    Mat<double> & getMatrixRef(size_t const key) const override;
    /**
     * @see IVoxelGrid::getMatrixConstRef
     */
    Mat<double> const & getMatrixConstRef(size_t const key) const override;
    /**
     * @see IVoxelGrid::setMatrix
     */
    void setMatrix(size_t const key, Mat<double> *mat) override;
    /**
     * @see IVoxelGrid::setNextMatrixCol
     */
    void setNextMatrixCol(
        size_t const key,
        double const x, double const y, double const z
    ) override;
    /**
     * @see IVoxelGrid::deleteMatrix
     */
    void deleteMatrix(size_t const key) override;
    /**
     * @see IVoxelGrid::deleteMatrices
     */
    void deleteMatrices() override;


    /**
     * @see IVoxelGrid::getCursor
     */
    size_t getCursor(size_t const key) const override;
    /**
     * @see IVoxelGrid::setCursor
     */
    void setCursor(size_t const key, size_t const cursor) override;
    /**
     * @see IVoxelGrid::incrementCursor
     */
    void incrementCursor(size_t const key) override;

    /**
     * @see IVoxelGird::getClosestPointDistance
     */
    double getClosestPointDistance(size_t const key) const override;
    /**
     * @see IVoxelGrid::setClosestPointDistance
     */
    void setClosestPointDistance(
        size_t const key, double const distance
    ) override;

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
    Voxel * whileLoopNext() override;
};
