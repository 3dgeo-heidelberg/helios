#include <algorithm>
#include <assetloading/geometryfilter/SparseVoxelGrid.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SparseVoxelGrid::SparseVoxelGrid(std::size_t const maxNVoxels)
  : IVoxelGrid(maxNVoxels)
  , map(std::max((std::size_t)1, (std::size_t)std::sqrt(maxNVoxels)))
{
}

// ***  VOXEL GRID INTERFACE  *** //
// ****************************** //
void
SparseVoxelGrid::release()
{
  map.clear();
}

bool
SparseVoxelGrid::hasVoxel(std::size_t const key)
{
  return map.find(key) != map.cend();
}

Voxel*
SparseVoxelGrid::getVoxel(std::size_t const key)
{
  try {
    return map.at(key).voxel;
  } catch (std::out_of_range& ex) {
    return nullptr;
  }
}

Voxel*
SparseVoxelGrid::setVoxel(std::size_t const key,
                          double const x,
                          double const y,
                          double const z,
                          double halfVoxelSize)
{
  Voxel* voxel = new Voxel(x, y, z, halfVoxelSize);
  VoxelGridCell& vgc = map[key];
  vgc.voxel = voxel;
  return voxel;
}

void
SparseVoxelGrid::deleteVoxel(std::size_t const key)
{
  VoxelGridCell& vgc = map.at(key);
  Voxel* voxel = vgc.voxel;
  if (voxel == nullptr)
    return;
  delete voxel;
  vgc.voxel = nullptr;
}

arma::Mat<double>*
SparseVoxelGrid::getMatrix(std::size_t const key) const
{
  return map.at(key).matrix;
}

arma::Mat<double>&
SparseVoxelGrid::getMatrixRef(std::size_t const key) const
{
  return *getMatrix(key);
}

arma::Mat<double> const&
SparseVoxelGrid::getMatrixConstRef(std::size_t const key) const
{
  return *getMatrix(key);
}

void
SparseVoxelGrid::setMatrix(std::size_t const key, arma::Mat<double>* mat)
{
  VoxelGridCell& vgc = map.at(key);
  if (vgc.matrix != nullptr)
    deleteMatrix(key);
  vgc.matrix = mat;
}

void
SparseVoxelGrid::setNextMatrixCol(std::size_t const key,
                                  double const x,
                                  double const y,
                                  double const z)
{
  VoxelGridCell& vgc = map.at(key);
  std::size_t const startIdx = vgc.cursor * 3;
  (*vgc.matrix)[startIdx] = x;
  (*vgc.matrix)[startIdx + 1] = y;
  (*vgc.matrix)[startIdx + 2] = z;
  ++vgc.cursor;
}

void
SparseVoxelGrid::deleteMatrix(std::size_t const key)
{
  try {
    VoxelGridCell& vgc = map.at(key);
    delete vgc.matrix;
    vgc.matrix = nullptr;
  } catch (std::out_of_range& oorex) {
  } // Ignore already deleted matrices
}
void
SparseVoxelGrid::deleteMatrices()
{
  for (size_t i = 0; i < maxNVoxels; ++i)
    deleteMatrix(i);
}

std::size_t
SparseVoxelGrid::getCursor(std::size_t const key) const
{
  return map.at(key).cursor;
}

void
SparseVoxelGrid::setCursor(std::size_t const key, std::size_t const cursor)
{
  map.at(key).cursor = cursor;
}

void
SparseVoxelGrid::incrementCursor(std::size_t const key)
{
  ++map.at(key).cursor;
}

double
SparseVoxelGrid::getClosestPointDistance(std::size_t const key) const
{
  return map.at(key).closestPointDistance;
}
void
SparseVoxelGrid::setClosestPointDistance(std::size_t const key,
                                         double const distance)
{
  map.at(key).closestPointDistance = distance;
}

// ***   WHILE INTERFACE   *** //
// *************************** //
void
SparseVoxelGrid::whileLoopStart()
{
  whileLoopIter = map.begin();
}

bool
SparseVoxelGrid::whileLoopHasNext()
{
  return whileLoopIter != map.end();
}

Voxel*
SparseVoxelGrid::whileLoopNext(std::size_t* key)
{
  // Obtain current voxel
  Voxel* voxel = whileLoopIter->second.voxel;
  // Obtain the matrix, if requested
  if (key != nullptr)
    *key = whileLoopIter->first;
  // Update iterator for next iteration
  ++whileLoopIter;
  // Return current voxel
  return voxel;
}
