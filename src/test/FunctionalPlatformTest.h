#pragma once

#include "BaseTest.h"
#include <platform/InterpolatedMovingPlatform.h>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Functional platform test
 *
 * This test checks that function based platforms, such as the
 *  InterpolatedMovingPlatform, work properly
 *
 * @see InterpolatedMovingPlatform
 */
class FunctionalPlatformTest : public BaseTest{
public:
    /**
     * @brief Decimal precision for validation purposes
     */
    double eps = 0.0001; // Decimal precision for validation purposes

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Functional platform test constructor
     */
    FunctionalPlatformTest() : BaseTest("Functional platform test") {}
    virtual ~FunctionalPlatformTest() = default;

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test the InterpolatingMovingPlatform
     * @return True if it worked as expected, false otherwise
     */
    bool testInterpolatedMovingPlatform();
};

// ***  R U N  *** //
// *************** //
bool FunctionalPlatformTest::run(){
    if(!testInterpolatedMovingPlatform()) return false;
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool FunctionalPlatformTest::testInterpolatedMovingPlatform(){
    // Prepare what is necessary to build the InterpolatedMovingPlatform
    std::function<void(void)> simulationFunction = [] (void) -> void {};
    SimulationStepLoop stepLoop(simulationFunction);
    TemporalDesignMatrix<double, double> tdm(
        arma::Mat<double>(
            "0.0 -6 -2  0;"
            "0.1 -4  2  0;"
            "0.2 -3  3  0;"
            "0.3 -1  4  0;"
            "0.4  1  4  0;"
            "0.5  2  2  1;"
            "0.6  3  1  1;"
            "0.7  5  1  1;"
            "0.8  6 -1  1"
        ),
        0,
        "t",
        vector<string>({"t", "x", "y", "z"})
    );
    DiffDesignMatrix<double, double> ddm = tdm.toDiffDesignMatrix();
    InterpolatedMovingPlatform imp(
        stepLoop,
        tdm,
        ddm,
        InterpolatedMovingPlatform::InterpolationScope::POSITION
    );
    // TODO Rethink : Implement test

    // Return true on success
    return true;
}

}