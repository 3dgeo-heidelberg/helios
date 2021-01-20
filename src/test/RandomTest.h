#pragma once

#include <noise/RandomnessGenerator.h>

#if defined(_WIN32) || defined(_WIN64)
#define exND1 0.300623, -1.42744, 0.0473341, -0.51204, -1.43744
#else
#define exND1 -1.42744, 0.300623, -0.51204, 0.0473341, 0.500384
#endif

namespace HeliosTests {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test for randomness generation
 */
class RandomTest : public BaseTest {
public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Random test constructor
     */
    RandomTest() : BaseTest("Randomness generation test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;
};

bool RandomTest::run(){
    const double eps = 0.0001;
    double diff;

    // Test random uniform real distribution generation with long seed
    double expectedURD1[] = {0.997185, 0.932557, 0.128124, 0.999041, 0.236089};
    RandomnessGenerator<double> rg1(1L);
    rg1.computeUniformRealDistribution(0.0, 1.0);
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg1.uniformRealDistributionNext() - expectedURD1[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test random uniform real distribution generation with double seed
    double expectedURD2[] = {
        -0.629836, 0.863082, 0.895461, -0.0305018, -0.358927
    };
    RandomnessGenerator<double> rg2(2.5);
    rg2.computeUniformRealDistribution(-1.0, 1.0);
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg2.uniformRealDistributionNext() - expectedURD2[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test random uniform real distribution with automatically generation
    RandomnessGenerator<double> rg3;
    diff = rg3.uniformRealDistributionNext();
    for(size_t i = 0 ; i < 4 ; i++){
        // This comparison may lead to false fails, but not too often
        if(rg3.uniformRealDistributionNext()==diff) return false;
    }

    // Test random uniform real distribution with parse long seed
    double expectedURD4[] = {
        0.834397,
        0.529073,
        0.497834,
        0.685763,
        0.97071
    };
    RandomnessGenerator<double> rg4("256");
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg4.uniformRealDistributionNext() - expectedURD4[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test random uniform real distribution with parse double seed
    double expectedURD5[] = {
        22.7339,
        31.8972,
        97.8223,
        45.5585,
        30.8013
    };
    RandomnessGenerator<double> rg5("7.9");
    rg5.computeUniformRealDistribution(0.0, 100.0);
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg5.uniformRealDistributionNext() - expectedURD5[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test random uniform real distribution with parse string timestamp seed
    double expectedURD6[] = {
        19.3035,
        -78.9788,
        34.9037,
        -90.7673,
        -50.6285
    };
    RandomnessGenerator<double> rg6("2000-08-22 11:30:27");
    rg6.computeUniformRealDistribution(-100.0, 50.0);
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg6.uniformRealDistributionNext() - expectedURD6[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test normal distribution
    double expectedND1[] = {exND1};
    RandomnessGenerator<double> rg7(1337);
    for(size_t i = 0 ; i < 5 ; i++){
        diff = rg7.normalDistributionNext() - expectedND1[i];
        if(diff < -eps || diff > eps) return false;
    }

    // Test swap, copy/move constructor and copy/move assignment
    double expectedURD8[] = {
        0.150989,
        0.995869,
        0.214838,
        0.0899676,
        0.18734,
        0.0592914
    };
    RandomnessGenerator<double> rg8(999);
    diff = rg8.uniformRealDistributionNext() - expectedURD8[0];
    if(diff < -eps || diff > eps) return false;
    RandomnessGenerator<double> rg8c(rg8);
    diff = rg8c.uniformRealDistributionNext() - expectedURD8[1];
    if(diff < -eps || diff > eps) return false;
    rg8 = rg8c;
    diff = rg8.uniformRealDistributionNext() - expectedURD8[2];
    if(diff < -eps || diff > eps) return false;
    rg8c = std::move(rg8);
    diff = rg8c.uniformRealDistributionNext() - expectedURD8[3];
    if(diff < -eps || diff > eps) return false;
    RandomnessGenerator<double> rg8m(std::move(rg8c));
    diff = rg8m.uniformRealDistributionNext() - expectedURD8[4];
    if(diff < -eps || diff > eps) return false;
    std::swap(rg8, rg8m);
    diff = rg8.uniformRealDistributionNext() - expectedURD8[5];
    if(diff < -eps || diff > eps) return false;

    // Test DEFAULT_RG behavior
    RandomnessGenerator<double> drgCopy1(*DEFAULT_RG);
    RandomnessGenerator<double> drgCopy2(*DEFAULT_RG);
    for(size_t i = 0 ; i < 5 ; i++){
        if(drgCopy1.uniformRealDistributionNext() ==
            drgCopy2.uniformRealDistributionNext()
        ){
            // It could lead to false fails, but not too often
            return false;
        }
        if(drgCopy1.normalDistributionNext() ==
           drgCopy2.normalDistributionNext()
            ){
            // It could lead to false fails, but not too often
            return false;
        }
    }
    DEFAULT_RG->computeUniformRealDistribution(-1.0, 1.0);
    DEFAULT_RG->computeNormalDistribution(4.0, 2.0);
    RandomnessGenerator<double> drgCopy3(*DEFAULT_RG);
    RandomnessGenerator<double> drgCopy4(*DEFAULT_RG);
    for(size_t i = 0 ; i < 5 ; i++){
        if(drgCopy3.uniformRealDistributionNext() !=
           drgCopy4.uniformRealDistributionNext()
            ){
            return false;
        }
        if(drgCopy3.normalDistributionNext() !=
           drgCopy4.normalDistributionNext()
            ){
            return false;
        }
    }



    // Successfully reached end of test
    return true;
}


}
