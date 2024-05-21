#pragma once

#include <assetloading/geometryfilter/XYZPointCloudFileLoader.h>


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Import point cloud files abstracting them to a set of voxels, similar
 *  to XYZPointCloudFileLoader. However, the voxel grid will be assumed to be
 *  sparse. Consequently, this loader can voxelize point clouds covering
 *  a greater spatial region in a sparse way, yet it can be slower because
 *  it needs to use (key, value) pairs instead of straightforward indexing.
 *
 * @see XYZPointCloudFileLoader
 */
class SparseXYZPointCloudFileLoader : public XYZPointCloudFileLoader {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for sparse point cloud loader.
     * @see XYZPointCloudFileLoader::XYZPointCloudFileLoader
     */
    explicit SparseXYZPointCloudFileLoader() : XYZPointCloudFileLoader() {};
    virtual ~SparseXYZPointCloudFileLoader() = default;

    // TODO Rethink : Implement

};