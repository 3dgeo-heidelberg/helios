#pragma once

#include <scene/primitives/Voxel.h>

#include <armadillo>
using arma::Mat;

#include <cstdlib>

/**
 * @brief VoxelGridCell is used to build and fill all necessary voxels to
 * represent an input point cloud
 */
struct VoxelGridCell{
    /**
     * @brief The voxel itself.
     */
    Voxel *voxel = nullptr;
    /**
     * @brief Matrix of point-wise coordinates.
     */
    Mat<double> *matrix = nullptr;
    /**
     * @brief The cursor to know where to write the next element in the matrix.
     */
    size_t cursor = 0;
    /**
     * @brief The distance between the voxel and the closest point inside it.
     */
    double closestPointDistance = std::numeric_limits<double>::max();
};

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Voxel grid interface that must be implemented by any class
 *  providing voxel grid based functionalities.
 */
class IVoxelGrid {
protected:
    // ***   ATTRIBUTES   *** //
    // ********************** //
    /**
     * @brief The max number of voxels supported by the grid.
     */
    size_t maxNVoxels;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for a voxel grid. The max number of voxels must be
     *  known in construction time.
     * @param maxNVoxels The max number of voxels that can be stored in the
     *  voxel grid.
     */
    IVoxelGrid(size_t const maxNVoxels) : maxNVoxels(maxNVoxels) {}
    virtual ~IVoxelGrid() = default;




    // ***  VOXEL GRID INTERFACE  *** //
    // ****************************** //
    /**
     * @brief Release all the data stored by the voxel grid.
     */
    virtual void release() = 0;


    /**
     * @brief Check whether the grid contains the voxel for the given key.
     * @param key The key univocally identifying the voxel.
     * @return True if the voxel is contained in the grid, false otherwise.
     */
    virtual bool hasVoxel(size_t const key) = 0;
    /**
     * @brief Obtain the voxel associated to the given key.
     * @param key The key univocally identifying the voxel.
     * @return The requested voxel.
     */
    virtual Voxel* getVoxel(size_t const key) = 0;
    /**
     * @brief Set the voxel associated to the given key.
     * @param key The key univocally identifying the voxel.
     * @param x The x coordinate for the voxel.
     * @param y The y coordinate for the voxel.
     * @param z The z coordinate for the voxel.
     * @param halfVoxelSize Half of the voxel size (in the same unit than the
     *  coordinates).
     * @return A pointer to the setted voxel
     */
    virtual Voxel * setVoxel(
        size_t const key,
        double const x, double const y, double const z,
        double halfVoxelSize
    ) = 0;
    /**
     * @brief Delete the given voxel, provided it exists.
     * @param key The key that univocally identifies the voxel to be deteled.
     */
    virtual void deleteVoxel(size_t const key) = 0;


    /**
     * @brief Obtain the matrix of point-wise coordinates for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @return Pointer to the matrix of point-wise coordinates.
     */
    virtual Mat<double> * getMatrix(size_t const key) const = 0;
    /**
     * @brief Obtain the matrix of point-wise coordinates for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @return Reference to the matrix of point-wise coordinates.
     */
    virtual Mat<double> & getMatrixRef(size_t const key) const = 0;
    /**
     * @brief Obtain the matrix of point-wise coordinates for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @return Constant reference to the matrix of point-wise coordinates.
     */
    virtual Mat<double> const & getMatrixConstRef(size_t const key) const = 0;
    /**
     * @brief Set the matrix of point-wise coordinates for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @param mat The new matrix of point-wise coordinates for the voxel.
     */
    virtual void setMatrix(size_t const key, Mat<double> *mat) = 0;
    /**
     * @brief Set the next column (point) of the matrix.
     *
     * This method must be called carefully as it will update the cursor for
     * each matrix to govern where the next call must be written. Thus, make
     * sure it is not called more times than what it is strictly expected.
     *
     * The method XYZPointCloudFileLoader::estimateNormals shows a good example
     *  of how this function must be called inside a loop.
     *
     * @param key The key of the voxel which matrix of coordinates must be
     *  updated.
     * @param x The first component for the new point.
     * @param y The second component for the new point.
     * @param z The third component for the new point.
     */
    virtual void setNextMatrixCol(
        size_t const key,
        double const x, double const y, double const z
    ) = 0;
    /**
     * @brief Delete the matrix of point-wise coordinates for a given voxel.
     * @param key The key that univocally identifies the voxel.
     */
    virtual void deleteMatrix(size_t const key) = 0;
    /**
     * @brief Delete all the matrices of point-wise coordinates (for all
     *  voxels).
     */
    virtual void deleteMatrices() = 0;


    /**
     * @brief Get the cursor for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @return The cursor of the voxel.
     */
    virtual size_t getCursor(size_t const key) const = 0;
    /**
     * @brief Set the cursor for a given voxel.
     * @param key The key that univocally identifies the voxel.
     * @param cursor The new cursor for the voxel.
     */
    virtual void setCursor(size_t const key, size_t const cursor) = 0;
    /**
     * @brief Increment the cursor in one (i.e., ++cursor).
     * @param key The key that univocally identifies the voxel.
     */
    virtual void incrementCursor(size_t const key) = 0;

    /**
     * @brief Obtain the distance to the closest point wrt the given voxel.
     * @param key The key that univocally identifies the voxel.
     * @return The distance between the voxel and the closest point.
     */
    virtual double getClosestPointDistance(size_t const key) const = 0;
    /**
     * @brief Set the distance to the closest point wrt the given voxel.
     * @param key The key that univocally identifies the voxel.
     */
    virtual void setClosestPointDistance(
        size_t const key, double const distance
    ) = 0;

    /**
     * @brief Obtain the max number of supported voxels.
     * @return Max number of supported voxels.
     * @see IVoxelGrid::maxNVoxels
     */
    inline size_t getMaxNVoxels() const {return maxNVoxels;}

    // ***   WHILE INTERFACE   *** //
    // *************************** //
    /**
     * @brief Prepare the grid so it can be iterated through a while loop.
     */
    virtual void whileLoopStart() = 0;
    /**
     * @brief Check whether the current iteration of the while loop is the
     *  last (false) or not (true).
     * @return True if the current iteration is not the last, false otherwise.
     */
    virtual bool whileLoopHasNext() = 0;
    /**
     * @brief Move to the next while iteration and obtain the corresponding
     *  VoxelGridCell.
     * @param[out] key When not null, it will be used to store the key
     *  associated to the returned voxel.
     * @return The Voxel of the next iteration.
     * @see VoxelGridCell
     */
    virtual Voxel * whileLoopNext(size_t *key=nullptr) = 0;
};