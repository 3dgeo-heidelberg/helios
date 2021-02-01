#pragma once

#include <VoxelFileParser.h>

namespace HeliosTests{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test for voxel parsing
 */
class VoxelParsingTest : public BaseTest {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Where required test files are stored.
     * For VoxelParsingTest it is required that a file named
     * semitransparent_voxel.vox is inside the test folder so voxels can be
     * parsed
     */
    std::string testDir;

   // ***  CONSTRUCTION  *** //
   // ********************** //
   /**
    * @brief Constructor for voxel parsing test
    */
    VoxelParsingTest(std::string testDir="data/test/") :
        BaseTest("Voxel parsing test"),
        testDir(testDir)
    {}

   // ***  R U N  *** //
   // *************** //
   /**
    * @see BaseTest::run
    */
   bool run() override;
};


bool VoxelParsingTest::run(){
    double eps = 0.0001;
    std::string vfPath = testDir + "semitransparent_voxels.vox";
    VoxelFileParser vfp;
    std::vector<std::shared_ptr<DetailedVoxel>> dvoxels = vfp.parseDetailed(
        vfPath,
        2,
        false
    );
    if(dvoxels.size()!=36) return false;
    std::shared_ptr<DetailedVoxel> dv;
    double xDiff, yDiff, zDiff;

    // Expected int values
    int EXPECTED_nbEchos[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    };
    int EXPECTED_nbSampling[36] = {
        49, 120, 169, 180, 153, 139, 302, 216, 116, 38,
        35, 153, 168, 124, 253, 219, 69, 69, 69, 216,
        116, 38, 35, 153, 168, 124, 253, 219, 69, 50,
        16, 8, 38, 54, 174, 48
    };

    // Test expected int values
    for(size_t i = 0 ; i < dvoxels.size() ; i++){
        dv = dvoxels[i];
        if(dv->getNbEchos() != EXPECTED_nbEchos[i]) return false;
        if(dv->getNbSampling() != EXPECTED_nbSampling[i]) return false;
    }

    // Expected double values
    double EXPECTED_PadBVTotal[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    };
    double EXPECTED_angleMean[36] = {
        89.7859174, 89.9096456, 89.6270244, 89.3006538, 88.9270184,
        88.6899252, 88.3412947, 88.0299087, 87.6877344, 87.3695616,
        87.120519,  86.7490226, 86.382687,  86.0816272, 85.7616811,
        85.4599773, 85.1357778, 85.1357778, 85.1357778, 88.0299087,
        87.6877344, 87.3695616, 87.120519,  86.7490226, 86.382687,
        86.0816272, 85.7616811, 85.4599773, 85.1357778, 84.7967107,
        84.606863,  84.2140568, 83.8143775, 83.5161812, 83.234543,
        82.9336984
    };
    double EXPECTED_bsPotential[36] = {
        0.1843913,  0.1772976,  0.1854843,  0.1794923,  0.186054,
        0.180072,   0.1877376,  0.1861319,  0.1861689,  0.1845732,
        0.1884445,  0.1901379,  0.1885587,  0.1875307,  0.1897892,
        0.1915078,  0.1910439,  0.1910439,  0.1910439,  0.1861319,
        0.1861689,  0.1845732,  0.1884445,  0.1901379,  0.1885587,
        0.1875307,  0.1897892,  0.1915078,  0.1910439,  0.1889422,
        0.1939671,  0.1940688,  0.1952733,  0.1931918,  0.1982526,
        0.1972807
    };
    double EXPECTED_x[36] = {
        5.0,        5.0,        5.0,        5.0,
        5.0,        5.0,        5.0,        5.0,
        5.0,        5.0,        5.0,        5.0,
        15.0,       15.0,       15.0,       15.0,
        15.0,       15.0,       15.0,       15.0,
        15.0,       15.0,       15.0,       15.0,
        25.0,       25.0,       25.0,       25.0,
        25.0,       25.0,       25.0,       25.0,
        25.0,       25.0,       25.0,       25.0
    };
    double EXPECTED_y[36] = {
        5.0,        5.0,        5.0,        5.0,
        15.0,       15.0,       15.0,       15.0,
        25.0,       25.0,       25.0,       25.0,
        5.0,        5.0,        5.0,        5.0,
        15.0,       15.0,       15.0,       15.0,
        25.0,       25.0,       25.0,       25.0,
        5.0,        5.0,        5.0,        5.0,
        15.0,       15.0,       15.0,       15.0,
        25.0,       25.0,       25.0,       25.0
    };
    double EXPECTED_z[36] = {
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0,
        5.0,    15.0,    25.0,    35.0
    };

    // Test expected double values
    for(size_t i = 0 ; i < dvoxels.size() ; i++){
        dv = dvoxels[i];
        if((*dv)[0] != EXPECTED_PadBVTotal[i]) return false;
        if((*dv)[1] != EXPECTED_angleMean[i]) return false;
        if((*dv)["bsPotential"] != EXPECTED_bsPotential[i]) return false;
        xDiff = dv->getCentroid().x - EXPECTED_x[i];
        if(xDiff < -eps || xDiff > eps) return false;
        yDiff = dv->getCentroid().y - EXPECTED_y[i];
        if(yDiff < -eps || yDiff > eps) return false;
        zDiff = dv->getCentroid().z - EXPECTED_z[i];
        if(zDiff < -eps || zDiff > eps) return false;
        if(dv->halfSize != 5.0) return false;
    }

    // Successfully reached end of test
    return true;
}


}