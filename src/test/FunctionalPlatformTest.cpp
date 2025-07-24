#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"
#include <platform/InterpolatedMovingPlatform.h>

TEST_CASE( "Functional platform test ") {
    double eps = 0.0001; // Decimal precision for validation purposes
    int simFreq = 10;
    std::function<void(void)> simulationFunction = [] (void) -> void {};
    SimulationStepLoop stepLoop(simulationFunction);
    stepLoop.setFrequency(simFreq);
    fluxionum::TemporalDesignMatrix<double, double> tdm(
        arma::Mat<double>(
            "0.0  -6  -2   0;"
            "0.2  -4   2   0;"
            "0.3  -3   3   0;"
            "0.5  -1   4   0;"
            "0.7   1   4   0;"
            "0.8   2   2   1;"
            "0.9   3   1   1;"
            "1.1   5   1   1;"
            "1.2   6  -1   1"
        ),
        0,
        "t",
        vector<string>({"t", "x", "y", "z"})
    );
    fluxionum::DiffDesignMatrix<double, double> ddm = tdm.toDiffDesignMatrix();
    InterpolatedMovingPlatform imp(
        stepLoop,
        tdm,
        ddm,
        InterpolatedMovingPlatform::InterpolationScope::POSITION,
        false,
        0.0
    );

    // Validate InterpolatedMovingPlatform
    arma::Mat<double> impE( // Expected output
        "-6  -2   0;"       // t=0.0
        "-5   0   0;"       // t=0.1
        "-4   2   0;"       // t=0.2
        "-3   3   0;"       // t=0.3
        "-2  3.5  0;"       // t=0.4
        "-1   4   0;"       // t=0.5
        "0    4   0;"       // t=0.6
        "1    4   0;"       // t=0.7
        "2    2   1;"       // t=0.8
        "3    1   1;"       // t=0.9
        "4    1   1;"       // t=1.0
        "5    1   1;"       // t=1.1
        "6    -1  1;"       // t=1.2
    );
    for(size_t i = 0 ; i < impE.n_rows ; ++i){
        imp.doSimStep(simFreq);
        REQUIRE( std::fabs(stepLoop.getCurrentTime()-((double)i)/10.0) <= eps);

        glm::dvec3 _impPos = imp.getPosition();
        arma::Col<double> impPos(3);
        impPos[0] = _impPos.x;  impPos[1] = _impPos.y;  impPos[2] = _impPos.z;
        REQUIRE(arma::all((impPos-impE.row(i).as_col()) <= eps));
        stepLoop.nextStep();
    }
}