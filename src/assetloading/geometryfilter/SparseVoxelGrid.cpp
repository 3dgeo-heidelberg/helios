#include <assetloading/geometryfilter/SparseVoxelGrid.h>
#include <algorithm>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SparseVoxelGrid::SparseVoxelGrid(size_t const maxNVoxels) :
    IVoxelGrid(maxNVoxels),
    map(std::max((size_t) 1, (size_t) std::sqrt(maxNVoxels)))
{}


// ***  VOXEL GRID INTERFACE  *** //
// ****************************** //
void SparseVoxelGrid::release(){
    map.clear();
}


bool SparseVoxelGrid::hasVoxel(size_t const key){
    return map.find(key) != map.cend();
}

Voxel* SparseVoxelGrid::getVoxel(size_t const key){
    return map.at(key).voxel;
}

void SparseVoxelGrid::setVoxel(
    size_t const key,
    double const x, double const y, double const z,
    double halfVoxelSize
){
    Voxel *voxel = new Voxel(x, y, z, halfVoxelSize);
    if(!hasVoxel(key)) {
        map.insert(std::pair<size_t, VoxelGridCell>(key, VoxelGridCell()));
    }
    map.at(key).voxel = voxel;
}

void SparseVoxelGrid::deleteVoxel(size_t const key){
    VoxelGridCell & vgc = map.at(key);
    Voxel *voxel = vgc.voxel;
    if(voxel == nullptr) return;
    delete voxel;
    vgc.voxel = nullptr;
}

Mat<double> * SparseVoxelGrid::getMatrix(size_t const key) const {
    return map.at(key).matrix;
}

Mat<double> & SparseVoxelGrid::getMatrixRef(size_t const key) const {
    return *getMatrix(key);
}

Mat<double> const & SparseVoxelGrid::getMatrixConstRef(size_t const key) const {
    return *getMatrix(key);
}

void SparseVoxelGrid::setMatrix(size_t const key, Mat<double> *mat){
    VoxelGridCell &vgc = map.at(key);
    if(vgc.matrix != nullptr) deleteMatrix(key);
    vgc.matrix = mat;
}

void SparseVoxelGrid::setNextMatrixCol(
    size_t const key,
    double const x, double const y, double const z
) {
    VoxelGridCell &vgc = map.at(key);
    size_t const startIdx = vgc.cursor*3;
    (*vgc.matrix)[startIdx] = x;
    (*vgc.matrix)[startIdx+1] = y;
    (*vgc.matrix)[startIdx+2] = z;
    ++vgc.cursor;
}

void SparseVoxelGrid::deleteMatrix(size_t const key){
    VoxelGridCell &vgc = map.at(key);
    delete vgc.matrix;
    vgc.matrix = nullptr;
}
void SparseVoxelGrid::deleteMatrices(){
    for(size_t i = 0 ; i < maxNVoxels ; ++i) deleteMatrix(i);
}


size_t SparseVoxelGrid::getCursor(size_t const key) const{
    return map.at(key).cursor;
}

void SparseVoxelGrid::setCursor(size_t const key, size_t const cursor) {
    map.at(key).cursor = cursor;
}

void SparseVoxelGrid::incrementCursor(size_t const key){
    ++map.at(key).cursor;
}

double SparseVoxelGrid::getClosestPointDistance(size_t const key) const {
    return map.at(key).closestPointDistance;
}
void SparseVoxelGrid::setClosestPointDistance(
    size_t const key, double const distance
) {
    map.at(key).closestPointDistance = distance;
}
