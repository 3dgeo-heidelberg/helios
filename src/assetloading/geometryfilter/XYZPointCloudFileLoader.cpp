#include <iostream>
using namespace std;

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/variant/get.hpp>

#include <glm/glm.hpp>
#include <boost/filesystem.hpp>
using namespace glm;
namespace fs = boost::filesystem;

#include <logging.hpp>
#include <util/HeliosException.h>
#include <util/FileUtils.h>

#include "Material.h"
#include "PlaneFitter.h"

#include "XYZPointCloudFileLoader.h"

// ***  R U N  *** //
// *************** //
ScenePart* XYZPointCloudFileLoader::run() {
    // Determine filepath
    std::vector<std::string> filePaths = FileUtils::handleFilePath(params);

    // Read separator
	string const & pSep = boost::get<string const &>(params["separator"]);
	if (!pSep.empty()) separator = pSep;

	// Read voxel size
    double pVoxelSize = boost::get<double>(params["voxelSize"]);
	if (pVoxelSize != 0) voxelSize = pVoxelSize;

	// Read max color value
	maxColorValue = 255.0;
    if(params.find("maxColorValue") != params.end()){
        double pMaxCol = boost::get<double>(params["maxColorValue"]);
        if (pMaxCol != 0) {
            maxColorValue = pMaxCol;
        }
    }

	// Default normal
	if(params.find("defaultNormal") != params.end()){
	    defaultNormal = boost::get<dvec3>(params["defaultNormal"]);
	    defaultNormal = glm::normalize(defaultNormal);
	    assignDefaultNormal = true;
	}

	// Parse
    for(std::string const & filePath : filePaths){
        if(!fs::exists(fs::path(filePath))){ // Error : file not found
            std::stringstream ss;
            ss << "File not found: " << filePath << endl;
            logging::ERR(ss.str());
            exit(1);
        }
        lastNumVoxels = primsOut->mPrimitives.size();
        parse(filePath);
        primsOut->subpartLimit.push_back(primsOut->mPrimitives.size());
    }

    // Return
	return primsOut;
}


// ***  MAIN PARSING METHODS  *** //
// ****************************** //
void XYZPointCloudFileLoader::parse(std::string const & filePath){
    logging::INFO(
        "Reading point cloud from XYZ file " + filePath + " ..."
    );

    // Initialize counters
    unsafeNormalEstimations = 0;
    discardedPointsByNormal = 0;

    // Material
    string matName = "default";
    logging::INFO("Adding default material");
    Material mat;
    mat.useVertexColors = true;
    mat.isGround = true;
    mat.name = matName;
    materials.insert(materials.end(), pair<string, Material>(matName, mat));

    // Open file input stream
    ifstream ifs;
    try {
        ifs = ifstream(filePath, ifstream::binary);
    }
    catch (std::exception &e) {
        logging::ERR("Failed to open xyz point cloud file: " + filePath +
                     "\nEXCEPTION: " + e.what());
        exit(-1);
    }

    // First pass
    firstPass(filePath, ifs);

    // Second pass
    secondPass(filePath, matName, ifs);

    // Release
    if(voxels != nullptr){
        delete[] voxels;
        voxels = nullptr;
    }

    // Load material
    loadMaterial();

    // Final report
    stringstream ss;
    ss << "Point cloud file read successful ("
       << primsOut->mPrimitives.size()-lastNumVoxels << " voxels)\n";
    if(unsafeNormalEstimations > 0) {
        ss  << "\t" << unsafeNormalEstimations
            << " voxels did not have enough points ("
            << minPointsForSafeNormalEstimation << ") to safely "
            << "estimate their normal\n";
    }
    if(discardedPointsByNormal > 0) {
        ss  << "\t" << discardedPointsByNormal << " points have a non valid "
            << "normal";
    }
    logging::INFO(ss.str());
    ss.str("");
}

void XYZPointCloudFileLoader::firstPass(
    string const & filePathString,
    ifstream &ifs
){
    // Initial values
    minX = numeric_limits<double>::max();
    minY = numeric_limits<double>::max();
    minZ = numeric_limits<double>::max();
    maxX = numeric_limits<double>::lowest();
    maxY = numeric_limits<double>::lowest();
    maxZ = numeric_limits<double>::lowest();

    // Parse
    string line;
    double x, y, z;
    try {
        n = 0;
        while (getline(ifs, line)) {
            if(isLineComment(line)) continue;
            boost::algorithm::trim(line);
            vector<string> lineParts;
            boost::split(lineParts, line, boost::is_any_of(separator));
            // ########## BEGIN Read vertex position ##########
            unsigned int size = lineParts.size();
            if (size >= 3) {
                x = boost::lexical_cast<double>(lineParts[0]);
                y = boost::lexical_cast<double>(lineParts[1]);
                z = boost::lexical_cast<double>(lineParts[2]);
                if(x < minX) minX = x;
                if(x > maxX) maxX = x;
                if(y < minY) minY = y;
                if(y > maxY) maxY = y;
                if(z < minZ) minZ = z;
                if(z > maxZ) maxZ = z;
                n++;
            }
            else{
                std::stringstream ss;
                ss << "XYZPointCloudFileLoader ERROR:\n"
                   << "\tRecord/row with no (x,y,z) has been found.";
                logging::ERR(ss.str());
                throw HeliosException(
                    "Exception at XYZPointCloudFileLoader::firstPass\n"
                    "\tRecord with no (x,y,z)"
                );
            }
        }

        // Compute number of necessary batches
        numBatches = std::ceil((double)n/(double)batchSize);

        // Restore file cursor position to beginning
        restartInputFileStream(ifs);
    }
    catch (std::exception &e) {
        std::stringstream ss;
        ss <<   "Failed to read xyz point cloud file \"" <<
                filePathString << "\"\nEXCEPTION: " << e.what();
        logging::ERR(ss.str());
        exit(-1);
    }
}

void XYZPointCloudFileLoader::secondPass(
    string const &filePathString,
    string const &matName,
    ifstream &ifs
){
    // Prepare voxels grid
    int estimateNormals;
    double halfVoxelSize;
    prepareVoxelsGrid(estimateNormals, halfVoxelSize);

    // Fill voxels grid
    fillVoxelsGrid(ifs, estimateNormals, halfVoxelSize, filePathString);

    // Post-processing of voxels
    postProcess(matName, estimateNormals);

    // Normals estimation
    if(estimateNormals>1) XYZPointCloudFileLoader::estimateNormalsBatch(ifs);
    else if(estimateNormals==1) XYZPointCloudFileLoader::estimateNormals(ifs);

    // Compose scene part with voxels
    voxelsGridToScenePart();
}

void XYZPointCloudFileLoader::loadMaterial(){
    // Parse materials
    std::vector<std::shared_ptr<Material>> matvec = parseMaterials();
    if(matvec.empty()) return;

    // Assign material to each detailed voxel
    size_t j, n = primsOut->mPrimitives.size(), m = matvec.size();
    for(size_t i = 0 ; i < n ; i++){
        Voxel *vxl = (Voxel *) primsOut->mPrimitives[i];
        j = i % m;
        vxl->material = matvec[j];
    }

}

// ***  AUXILIAR PARSING METHODS  *** //
// ********************************** //
void XYZPointCloudFileLoader::prepareVoxelsGrid(
    int &estimateNormals,
    double &halfVoxelSize
){
    // Compute voxel grid configuration
    halfVoxelSize = voxelSize/2.0;
    double deltaX = maxX - minX;
    if(deltaX == 0.0){
        minX = -halfVoxelSize;
        maxX = halfVoxelSize;
    }
    double deltaY = maxY - minY;
    if(deltaY == 0.0){
        minY = -halfVoxelSize;
        maxY = halfVoxelSize;
    }
    double deltaZ = maxZ - minZ;
    if(deltaZ == 0.0){
        minZ = -halfVoxelSize;
        maxZ = halfVoxelSize;
    }
    nx = std::ceil(deltaX / voxelSize);
    if(nx == 0) nx = 1;
    ny = std::ceil(deltaY / voxelSize);
    if(ny == 0) ny = 1;
    nz = std::ceil(deltaZ / voxelSize);
    if(nz == 0) nz = 1;
    nynz = ny * nz;
    maxNVoxels = nx*nynz;
    xCoeff = nx / deltaX;
    yCoeff = ny / deltaY;
    zCoeff = nz / deltaZ;

    // Allocate voxel grid
    voxels = new VoxelGridCell[maxNVoxels];

    // Check if voxel grid needs normal estimation or not
    estimateNormals = 0;
    if(params.find("estimateNormals") != params.end()){
        estimateNormals = boost::get<int>(params["estimateNormals"]);
    }

}

void XYZPointCloudFileLoader::fillVoxelsGrid(
    ifstream &ifs,
    int estimateNormals,
    double halfVoxelSize,
    string const &filePathString
){
    // Retrieve voxel grid parameters
    size_t rgbRIndex = 3;
    size_t rgbGIndex = 4;
    size_t rgbBIndex = 5;
    size_t normalXIndex = 6;
    size_t normalYIndex = 7;
    size_t normalZIndex = 8;
    if(params.find("rgbRIndex") != params.end()){
        rgbRIndex = (size_t) boost::get<int>(params["rgbRIndex"]);
        rgbGIndex = (size_t) boost::get<int>(params["rgbGIndex"]);
        rgbBIndex = (size_t) boost::get<int>(params["rgbBIndex"]);
    }
    if(params.find("normalXIndex") != params.end()){
        normalXIndex = (size_t) boost::get<int>(params["normalXIndex"]);
        normalYIndex = (size_t) boost::get<int>(params["normalYIndex"]);
        normalZIndex = (size_t) boost::get<int>(params["normalZIndex"]);
    }
    if(params.find("snapNeighborNormal") != params.end()){
        snapNeighborNormal = boost::get<bool>(params["snapNeighborNormal"]);
    }

    // Fill voxel grid
    try {
        string line;
        double x=0, y=0, z=0, r=0, g=0, b=0, xnorm=0, ynorm=0, znorm=0;
        while(getline(ifs, line)){
            // Parse
            if(isLineComment(line)) continue;
            boost::algorithm::trim(line);
            vector<string> lineParts;
            boost::split(lineParts, line, boost::is_any_of(separator));
            unsigned int size = lineParts.size();

            // Coordinates
            if(size >= 3) {
                x = boost::lexical_cast<double>(lineParts[0]);
                y = boost::lexical_cast<double>(lineParts[1]);
                z = boost::lexical_cast<double>(lineParts[2]);
            }

            // RGB
            if(size >= rgbBIndex){
                r = boost::lexical_cast<double>(lineParts[rgbRIndex]) /
                    this->maxColorValue;
                g = boost::lexical_cast<double>(lineParts[rgbGIndex]) /
                    this->maxColorValue;
                b = boost::lexical_cast<double>(lineParts[rgbBIndex]) /
                    this->maxColorValue;
            }

            // Normal
            if(!estimateNormals) {
                if (size >= normalZIndex){
                    xnorm = boost::lexical_cast<double>(
                        lineParts[normalXIndex]);
                    ynorm = boost::lexical_cast<double>(
                        lineParts[normalYIndex]);
                    znorm = boost::lexical_cast<double>(
                        lineParts[normalZIndex]);
                    // Correct voxel normal if necessary
                    if (!correctNormal(xnorm, ynorm, znorm)) {
                        // Ignore points which normals are not correct and
                        // cannot be corrected neither
                        discardedPointsByNormal += 1;
                        continue;
                    }
                }
            }

            // Digest voxel
            digestVoxel(
                estimateNormals,
                halfVoxelSize,
                x, y, z,
                r, g, b,
                xnorm, ynorm, znorm
            );

        }
        // Restore file cursor position to beginning
        restartInputFileStream(ifs);
    }
    catch (std::exception &e) {
        std::stringstream ss;
        ss <<   "Failed to read (2nd pass, fillVoxelsGrid) xyz point cloud"
                "file \"" <<
                filePathString << "\"\nEXCEPTION: " << e.what();
        logging::ERR(ss.str());
        exit(-1);
    }

}

bool XYZPointCloudFileLoader::correctNormal(
    double &x,
    double &y,
    double &z
){
    bool valid = !std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
        (x!=0 || y!=0 || z!=0);

    if(!valid){
        // Try to correct non valid normal if possible
        if(assignDefaultNormal){
            x = defaultNormal.x;
            y = defaultNormal.y;
            z = defaultNormal.z;
            valid = true;
        }
    }

    return valid;
}

void XYZPointCloudFileLoader::digestVoxel(
    int estimateNormals,
    double halfVoxelSize,
    double x,
    double y,
    double z,
    double r,
    double g,
    double b,
    double xnorm,
    double ynorm,
    double znorm
){
    // Compute index and indices
    size_t IDX, I, J, K;
    IDX = indexFromCoordinates(x, y, z, I, J, K);

    // If voxel does not exist, create it
    if (voxels[IDX].voxel == nullptr) {
        voxels[IDX].voxel = new Voxel(
            minX + halfVoxelSize * (double) (2 * I + 1),
            minY + halfVoxelSize * (double) (2 * J + 1),
            minZ + halfVoxelSize * (double) (2 * K + 1),
            halfVoxelSize
        );
    }

    // Populate voxel
    Voxel *voxel = voxels[IDX].voxel;
    voxel->numPoints++;
    voxel->r += r*r; // averaging RGB colors can be approximated by averaging the squares
    voxel->g += g*g; // see https://sighack.com/post/averaging-rgb-colors-the-right-way
    voxel->b += b*b;
    if(!estimateNormals) {
        if(snapNeighborNormal){
            // Snap closest neighbor normal
            double xDiff = x-voxel->v.getX();
            double yDiff = y-voxel->v.getY();
            double zDiff = z-voxel->v.getZ();
            double distance = std::sqrt(
                xDiff*xDiff + yDiff*yDiff + zDiff*zDiff
            );
            if(distance < voxels[IDX].closestPointDistance){
                voxels[IDX].closestPointDistance = distance;
                voxel->normal[0] = xnorm;
                voxel->normal[1] = ynorm;
                voxel->normal[2] = znorm;
            }
        }
        else {
            // Aggregate all points normals to compute voxel normal
            voxel->normal[0] += xnorm;
            voxel->normal[1] += ynorm;
            voxel->normal[2] += znorm;
        }
    }

}

void XYZPointCloudFileLoader::postProcess(
    std::string const &matName,
    int estimateNormals
){
    bool tooPopulatedWarning = false;

    // Post processing loop
    Voxel *voxel;
    for(size_t i = 0 ; i < maxNVoxels ; i++){
        voxel = voxels[i].voxel;
        if(voxel == nullptr) continue; // Ignore non occupied voxels
        if(voxel->numPoints > voxelPopulationThreshold){
            tooPopulatedWarning = true; // Check voxel population is in bounds
        }
        // Post-process voxel
        voxel->v.color = Color4f(
            std::sqrt(((float)voxel->r) / ((float)voxel->numPoints)),
            std::sqrt(((float)voxel->g) / ((float)voxel->numPoints)),
            std::sqrt(((float)voxel->b) / ((float)voxel->numPoints)),
            1.0
        );
        if(!estimateNormals && voxel->hasNormal() && !snapNeighborNormal)
            voxel->normal = glm::normalize(voxel->normal);
        voxel->material = getMaterial(matName);
    }

    // Warning about too populated voxels
    if(tooPopulatedWarning) {
        std::stringstream ss;
        ss << "There are too populated voxels (those with more than "
           << voxelPopulationThreshold << " points)\n"
           << "Computation might be to slow or even run out of memory.\n"
           << "Please, consider reducing voxel size.";
        logging::WARN(ss.str());
    }
}

void XYZPointCloudFileLoader::estimateNormals(ifstream &ifs){
    // Prepare voxel matrices
    for(size_t i = 0 ; i < maxNVoxels ; i++){
        if(voxels[i].voxel != nullptr){
            voxels[i].matrix = new Mat<double>(3, voxels[i].voxel->numPoints);
        }
    }

    // Populate voxel matrices
    try {
        std::string line;
        size_t I, J, K, IDX;
        double x, y, z;
        while (getline(ifs, line)) {
            // Parse
            if (isLineComment(line)) continue;
            boost::algorithm::trim(line);
            vector<string> lineParts;
            boost::split(lineParts, line, boost::is_any_of(separator));
            unsigned int size = lineParts.size();
            // Extract coordinates
            if (size >= 3) {
                x = boost::lexical_cast<double>(lineParts[0]);
                y = boost::lexical_cast<double>(lineParts[1]);
                z = boost::lexical_cast<double>(lineParts[2]);
                // Populate coordinates matrix
                IDX = indexFromCoordinates(x, y, z, I, J, K);
                VoxelGridCell &vgc = voxels[IDX];
                (*vgc.matrix)[vgc.cursor*3] = x;
                (*vgc.matrix)[vgc.cursor*3+1] = y;
                (*vgc.matrix)[vgc.cursor*3+2] = z;
                vgc.cursor++;
            }
        }
        // Restore file cursor position to beginning
        restartInputFileStream(ifs);
    }
    catch (std::exception &e) {
        std::stringstream ss;
        ss <<  "Failed to read (estimate normals) xyz point cloud file \n"
               "EXCEPTION: " << e.what();
        logging::ERR(ss.str());
        exit(-1);
    }

    // Estimate normals
    _estimateNormals(0, maxNVoxels);

    // Free matrices
    for(size_t i = 0 ; i < maxNVoxels ; i++){
        delete voxels[i].matrix;
    }
}

void XYZPointCloudFileLoader::estimateNormalsBatch(ifstream &ifs){
    // Prepare variables
    size_t startIdx = 0; // Start index for current batch [INCLUSIVE]
    size_t endIdx; // End index of current batch [EXCLUSIVE]

    // Estimate normals by batches
    for(size_t batch=0 ; batch < numBatches ; batch++){
        // Initialize points count to 0  (How many points considered for batch)
        size_t pointsCount = 0;

        // Prepare voxel matrices
        for(endIdx = startIdx ; endIdx < maxNVoxels ; endIdx++){
            // Skip empty cells (grid cells with no voxel)
            if(voxels[endIdx].voxel == nullptr) continue;
            // Handle number of points for this batch
            size_t voxelNumPoints = voxels[endIdx].voxel->numPoints;
            size_t newPointsCount = pointsCount + voxelNumPoints;
            if(newPointsCount > batchSize){
                // If batch size has been exceeded then finish
                break;
            }
            // Continue batch
            pointsCount = newPointsCount;
            voxels[endIdx].matrix = new Mat<double>(3, voxelNumPoints);
        }

        // Populate voxel matrices
        try {
            std::string line;
            size_t I, J, K, IDX;
            double x, y, z;
            while (getline(ifs, line)) {
                // Parse
                if (isLineComment(line)) continue;
                boost::algorithm::trim(line);
                vector<string> lineParts;
                boost::split(lineParts, line, boost::is_any_of(separator));
                unsigned int size = lineParts.size();
                // Extract coordinates
                if (size >= 3) {
                    x = boost::lexical_cast<double>(lineParts[0]);
                    y = boost::lexical_cast<double>(lineParts[1]);
                    z = boost::lexical_cast<double>(lineParts[2]);
                    // Check point is inside batch (ignore if it is not)
                    IDX = indexFromCoordinates(x, y, z, I, J, K);
                    if(IDX < startIdx || IDX >= endIdx) continue;
                    // Populate coordinates matrix
                    VoxelGridCell &vgc = voxels[IDX];
                    (*vgc.matrix)[vgc.cursor*3] = x;
                    (*vgc.matrix)[vgc.cursor*3+1] = y;
                    (*vgc.matrix)[vgc.cursor*3+2] = z;
                    vgc.cursor++;
                }
            }
            // Restore file cursor position to beginning
            restartInputFileStream(ifs);
        }
        catch (std::exception &e) {
            std::stringstream ss;
            ss <<  "Failed to read (estimate normals batch) xyz point cloud"
                   "file \nEXCEPTION: " << e.what();
            logging::ERR(ss.str());
            exit(-1);
        }

        // Esimate normals for current batch
        _estimateNormals(startIdx, endIdx);

        // Free matrices
        for(size_t i = startIdx ; i < endIdx ; i++){
            delete voxels[i].matrix;
        }

        // Update startIdx for next batch
        startIdx = endIdx;
    }
}

void XYZPointCloudFileLoader::_estimateNormals(size_t start, size_t end){
    // Find populated matrices
    for(size_t i = start ; i < end ; i++){
        VoxelGridCell &vgc = voxels[i];
        if(vgc.matrix != nullptr){
            if(vgc.voxel->numPoints <
                XYZPointCloudFileLoader::minPointsForSafeNormalEstimation
            ){
                // Not enough points for normal estimation
                unsafeNormalEstimations += 1;
                if(assignDefaultNormal) {
                    // Use default normal
                    vgc.voxel->normal = XYZPointCloudFileLoader::defaultNormal;
                }
                else{
                    // Discard voxel
                    delete vgc.voxel;
                    delete vgc.matrix;
                    vgc.voxel = nullptr;
                    vgc.matrix = nullptr;
                }
            }
            else {
                // Estimate normal
                std::vector<double> orthonormal =
                    PlaneFitter::bestFittingPlaneOrthoNormal(*vgc.matrix, true);
                vgc.voxel->normal.x = orthonormal[0];
                vgc.voxel->normal.y = orthonormal[1];
                vgc.voxel->normal.z = orthonormal[2];
            }
        }
    }
}

void XYZPointCloudFileLoader::voxelsGridToScenePart(){
    Voxel *voxel;
    for(size_t i = 0 ; i < maxNVoxels ; i++){
        voxel = voxels[i].voxel;
        if(voxel != nullptr){
            primsOut->mPrimitives.push_back(voxel);
        }
    }
}

size_t XYZPointCloudFileLoader::indexFromCoordinates(
    double x, double y, double z,
    size_t &I, size_t &J, size_t &K
){
    I = (size_t) ((x - minX) * xCoeff);
    if (I >= nx) I = nx - 1;
    J = (size_t) ((y - minY) * yCoeff);
    if (J >= ny) J = ny - 1;
    K = (size_t) ((z - minZ) * zCoeff);
    if (K >= nz) K = nz - 1;
    return I * nynz + J * nz + K;
}

bool XYZPointCloudFileLoader::isLineComment(string const & line){
    return line[0] == '/' && line[1] == '/';
}

void XYZPointCloudFileLoader::restartInputFileStream(ifstream &ifs){
    // Restore file cursor position to beginning
    ifs.clear();
    ifs.seekg(0, ios::beg);

}
