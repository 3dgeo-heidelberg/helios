#pragma once

#include <assetloading/geometryfilter/IVoxelGrid.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dense voxel grid implementation of IVoxelGrid. It uses the indices
 *  of an array to represent the (key, value) pairs as (index, voxel).
 *
 * @see IVoxelGrid
 */
class DenseVoxelGrid : public IVoxelGrid {
protected:
    // ***   ATTRIBUTES   *** //
    // ********************** //
    /**
     * @brief Array of voxels.
     * @see VoxelGridCell
     * @see Voxel
     */
    VoxelGridCell *voxels;
    /**
     * @brief Used internally to track the current iteration of the while loop.
     */
    size_t whileLoopIter;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for a dense voxel grid.
     * @see IVoxelGrid::IVoxelGrid
     */
    DenseVoxelGrid(size_t const maxNVoxels);
    virtual ~DenseVoxelGrid() {release();}

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