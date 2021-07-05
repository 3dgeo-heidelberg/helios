#pragma once

#include "BaseTest.h"
#include <LadLut.h>
#include <LadLutLoader.h>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test look-up table for leaf angle distribution
 */
class LadLutTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for validation purposes
     */
    double eps = 0.00001; // Decimal precision for validation purposes
    /**
     * @brief Where required test files are stored.
     * For LadLutTest it is required that a file named spherical.txt
     * is inside the test folder so ladlut can be parsed
     */
    std::string testDir;

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Leaf angle distribution look-up table test constructor
     */
    LadLutTest(std::string testDir="data/test/") :
        BaseTest("LadLut test"),
        testDir(testDir)
    {}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  INNER METHODS  *** //
    // *********************** //
    /**
     * @brief Validate given leaf angle distribution look-up table
     * @param ladlut Leaf angle distribution look-up table to be validated
     * @return True if given leaf angle distribution look-up table was valid,
     *  false otherwise
     * @see LadLut
     */
    bool validateLadLut(LadLut const &ladlut);
    /**
     * @brief Validate a transformation
     *
     * A transformation will be considered to be valid when the difference
     *  between obtained and expected value does not exceed eps
     *
     * @param x Obtained X coordinate
     * @param y Obtained Y coordinate
     * @param z Obtained Z coordinate
     * @param ex Expected X coordinate
     * @param ey Expected Y coordinate
     * @param ez Expected Z coordinate
     * @return True if transformation was valid, false otherwise
     * @see LadLutTest::eps
     */
    bool validateTransformation(
        double x, double y, double z,
        double ex, double ey, double ez
    );
};


bool LadLutTest::run(){
    // Input / expected output
    double u1[3] = {0.79259392, 0.22645541, 0.56613852};
    double g1 = 0.582513;
    double u2[3] = {-0.440759, 0.000000, 0.897626};
    double g2 = 0.720515;
    double u3[3] = {-0.36999991, 0.46129036, -0.80641879};
    double g3 = 0.673911;

    // Load spherical
    std::string llPath = testDir + "spherical.txt";
    LadLutLoader loader;
    std::shared_ptr<LadLut> ladlut = loader.load(llPath);
    if(!validateLadLut(*ladlut)) return false;

    // Transform to LadLut domain
    double wx, wy, wz;
    ladlut->transformToLadLutDomain(u1[0], u1[1], u1[2], wx, wy, wz);
    if(!validateTransformation(wx, wy, wz, 0.824310, 0.0, 0.566139))
        return false;

    // Interpolate 1
    double g = ladlut->interpolate(u1[0], u1[1], u1[2]);
    if(fabs(g-g1) > eps) return false;

    // Interpolate w
    g = ladlut->interpolate(wx, wy, wz);
    if(fabs(g-g1) > eps) return false;

    // Interpolate 2
    g = ladlut->interpolate(u2[0], u2[1], u2[2]);
    if(fabs(g-g2) > eps) return false;

    // Interpolate 3
    g = ladlut->interpolate(u3[0], u3[1], u3[2]);
    if(fabs(g-g3) > eps) return false;

    // Return
    return true;
}


bool LadLutTest::validateLadLut(LadLut const &ladlut){
    // Validation data
    size_t indices[4] = {0, 1, 498, 499};
    double expectedX[4] = {
        1.000000, 0.999921, 0.999921, 1.000000
    };
    double expectedY[4] = {
        0.000000, 0.000000, 0.000000, 0.000000
    };
    double expectedZ[4] = {
        0.000000, 0.012591, -0.012591, -0.000000
    };
    double expectedG[4] = {
        0.500000, 0.500040, 0.500040, 0.500000
    };

    // Validate
    for(size_t i = 0 ; i < 4 ; i++){
        size_t idx = indices[i];
        // Compute differences
        double diffX = ladlut.X[idx] - expectedX[i];
        double diffY = ladlut.Y[idx] - expectedY[i];
        double diffZ = ladlut.Z[idx] - expectedZ[i];
        double diffG = ladlut.G[idx] - expectedG[i];
        // Validate differences
        if(diffX < -eps || diffX > eps) return false;
        if(diffY < -eps || diffY > eps) return false;
        if(diffZ < -eps || diffZ > eps) return false;
        if(diffG < -eps || diffG > eps) return false;
    }
    return true;
}

bool LadLutTest::validateTransformation(
    double x, double y, double z,
    double ex, double ey, double ez
){
    double xDiff = x-ex;
    double yDiff = y-ey;
    double zDiff = z-ez;
    return  xDiff > -eps && xDiff < eps &&
            yDiff > -eps && yDiff < eps &&
            zDiff > -eps && zDiff < eps;
}

}
