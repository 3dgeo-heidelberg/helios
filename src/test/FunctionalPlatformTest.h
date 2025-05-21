#pragma once

#include "BaseTest.h"
#include <platform/InterpolatedMovingPlatform.h>

namespace HeliosTests {

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
class FunctionalPlatformTest : public BaseTest
{
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
  FunctionalPlatformTest()
    : BaseTest("Functional platform test")
  {
  }
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
bool
FunctionalPlatformTest::run()
{
  if (!testInterpolatedMovingPlatform())
    return false;
  return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool
FunctionalPlatformTest::testInterpolatedMovingPlatform()
{
  // Prepare what is necessary to build the InterpolatedMovingPlatform
  int simFreq = 10;
  std::function<void(void)> simulationFunction = [](void) -> void {};
  SimulationStepLoop stepLoop(simulationFunction);
  stepLoop.setFrequency(simFreq);
  TemporalDesignMatrix<double, double> tdm(
    arma::Mat<double>("0.0  -6  -2   0;"
                      "0.2  -4   2   0;"
                      "0.3  -3   3   0;"
                      "0.5  -1   4   0;"
                      "0.7   1   4   0;"
                      "0.8   2   2   1;"
                      "0.9   3   1   1;"
                      "1.1   5   1   1;"
                      "1.2   6  -1   1"),
    0,
    "t",
    vector<string>({ "t", "x", "y", "z" }));
  DiffDesignMatrix<double, double> ddm = tdm.toDiffDesignMatrix();
  InterpolatedMovingPlatform imp(
    stepLoop,
    tdm,
    ddm,
    InterpolatedMovingPlatform::InterpolationScope::POSITION,
    false,
    0.0);

  // Validate InterpolatedMovingPlatform
  arma::Mat<double> impE( // Expected output
    "-6  -2   0;"         // t=0.0
    "-5   0   0;"         // t=0.1
    "-4   2   0;"         // t=0.2
    "-3   3   0;"         // t=0.3
    "-2  3.5  0;"         // t=0.4
    "-1   4   0;"         // t=0.5
    "0    4   0;"         // t=0.6
    "1    4   0;"         // t=0.7
    "2    2   1;"         // t=0.8
    "3    1   1;"         // t=0.9
    "4    1   1;"         // t=1.0
    "5    1   1;"         // t=1.1
    "6    -1  1;"         // t=1.2
  );
  for (size_t i = 0; i < impE.n_rows; ++i) {
    imp.doSimStep(simFreq);
    if (std::fabs(stepLoop.getCurrentTime() - ((double)i) / 10.0) > eps)
      return false;
    glm::dvec3 _impPos = imp.getPosition();
    arma::Col<double> impPos(3);
    impPos[0] = _impPos.x;
    impPos[1] = _impPos.y;
    impPos[2] = _impPos.z;
    if (arma::any((impPos - impE.row(i).as_col()) > eps))
      return false;
    stepLoop.nextStep();
  }

  // Return true on success
  return true;
}

}
