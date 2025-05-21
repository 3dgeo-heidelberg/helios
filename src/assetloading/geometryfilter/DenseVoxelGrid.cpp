#include <assetloading/geometryfilter/DenseVoxelGrid.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DenseVoxelGrid::DenseVoxelGrid(size_t const maxNVoxels)
  : IVoxelGrid(maxNVoxels)
{
  voxels = new VoxelGridCell[maxNVoxels];
}

// ***  VOXEL GRID INTERFACE  *** //
// ****************************** //
void
DenseVoxelGrid::release()
{
  if (voxels != nullptr) {
    delete[] voxels;
    voxels = nullptr;
  }
}

bool
DenseVoxelGrid::hasVoxel(size_t const key)
{
  return voxels[key].voxel != nullptr;
}

Voxel*
DenseVoxelGrid::getVoxel(size_t const key)
{
  return voxels[key].voxel;
}

Voxel*
DenseVoxelGrid::setVoxel(size_t const key,
                         double const x,
                         double const y,
                         double const z,
                         double halfVoxelSize)
{
  VoxelGridCell& vgc = voxels[key];
  vgc.voxel = new Voxel(x, y, z, halfVoxelSize);
  return vgc.voxel;
}

void
DenseVoxelGrid::deleteVoxel(size_t const key)
{
  Voxel* voxel = voxels[key].voxel;
  if (voxel == nullptr)
    return;
  delete voxel;
  voxels[key].voxel = nullptr;
}

Mat<double>*
DenseVoxelGrid::getMatrix(size_t const key) const
{
  return voxels[key].matrix;
}

Mat<double>&
DenseVoxelGrid::getMatrixRef(size_t const key) const
{
  return *voxels[key].matrix;
}

Mat<double> const&
DenseVoxelGrid::getMatrixConstRef(size_t const key) const
{
  return *voxels[key].matrix;
}

void
DenseVoxelGrid::setMatrix(size_t const key, Mat<double>* mat)
{
  VoxelGridCell& vgc = voxels[key];
  if (vgc.matrix != nullptr)
    deleteMatrix(key);
  vgc.matrix = mat;
}

void
DenseVoxelGrid::setNextMatrixCol(size_t const key,
                                 double const x,
                                 double const y,
                                 double const z)
{
  VoxelGridCell& vgc = voxels[key];
  size_t const startIdx = vgc.cursor * 3;
  (*vgc.matrix)[startIdx] = x;
  (*vgc.matrix)[startIdx + 1] = y;
  (*vgc.matrix)[startIdx + 2] = z;
  ++vgc.cursor;
}

void
DenseVoxelGrid::deleteMatrix(size_t const key)
{
  delete voxels[key].matrix;
  voxels[key].matrix = nullptr;
}
void
DenseVoxelGrid::deleteMatrices()
{
  for (size_t i = 0; i < maxNVoxels; ++i)
    deleteMatrix(i);
}

size_t
DenseVoxelGrid::getCursor(size_t const key) const
{
  return voxels[key].cursor;
}

void
DenseVoxelGrid::setCursor(size_t const key, size_t const cursor)
{
  voxels[key].cursor = cursor;
}

void
DenseVoxelGrid::incrementCursor(size_t const key)
{
  ++voxels[key].cursor;
}

double
DenseVoxelGrid::getClosestPointDistance(size_t const key) const
{
  return voxels[key].closestPointDistance;
}
void
DenseVoxelGrid::setClosestPointDistance(size_t const key, double const distance)
{
  voxels[key].closestPointDistance = distance;
}

// ***   WHILE INTERFACE   *** //
// *************************** //
void
DenseVoxelGrid::whileLoopStart()
{
  for (whileLoopIter = 0;
       whileLoopIter < maxNVoxels && voxels[whileLoopIter].voxel == nullptr;
       ++whileLoopIter)
    ;
}

bool
DenseVoxelGrid::whileLoopHasNext()
{
  return whileLoopIter < maxNVoxels;
}

Voxel*
DenseVoxelGrid::whileLoopNext(size_t* key)
{
  // Obtain current voxel
  Voxel* voxel = voxels[whileLoopIter].voxel;
  // Obtain the matrix, if requested
  if (key != nullptr)
    *key = whileLoopIter;
  // Iterate to find next voxel, if any, and update whileLoopIter
  for (++whileLoopIter;
       whileLoopIter < maxNVoxels && voxels[whileLoopIter].voxel == nullptr;
       ++whileLoopIter)
    ;
  // Return current voxel
  return voxel;
}
