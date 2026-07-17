#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO

#include <MathConstants.h>
#include <assetloading/SwapOnRepeatHandler.h>
#include <maths/rigidmotion/RigidMotionR3Factory.h>
#include <scene/dynamic/DynMotion.h>
#include <scene/dynamic/DynScene.h>
#include <scene/dynamic/DynSequence.h>
#include <scene/dynamic/DynSequentiableMovingObject.h>
#include <scene/primitives/Triangle.h>

namespace {

std::shared_ptr<DynScene>
makeResetScene()
{
  Vertex a, b, c;
  a.pos = { 10.0, 20.0, 30.0 };
  b.pos = { 12.0, 20.0, 30.0 };
  c.pos = { 10.0, 22.0, 31.0 };
  a.normal = glm::normalize(glm::dvec3(1.0, 2.0, 3.0));
  b.normal = glm::normalize(glm::dvec3(-2.0, 1.0, 4.0));
  c.normal = glm::normalize(glm::dvec3(3.0, -1.0, 2.0));

  Triangle* triangle = new Triangle(a, b, c);
  auto object = std::make_shared<DynSequentiableMovingObject>(
    "reset-object", std::vector<Primitive*>{ triangle });
  triangle->part = object;

  rigidmotion::RigidMotionR3Factory factory;
  auto rotation =
    std::make_shared<DynMotion>(factory.makeRotationZ(PI_QUARTER), true);
  auto translation = std::make_shared<DynMotion>(
    factory.makeTranslation(arma::colvec({ 0.75, -0.25, 0.5 })));

  auto finite =
    std::make_shared<DynSequence<DynMotion>>("rotate", "translate", 2);
  finite->append(rotation);
  auto infinite = std::make_shared<DynSequence<DynMotion>>("translate", "", 0);
  infinite->append(translation);
  object->addSequence(finite);
  object->addSequence(infinite);
  object->setStepInterval(2);
  object->setObserverStepInterval(3);

  auto scene = std::make_shared<DynScene>(2);
  scene->appendDynObject(object);
  scene->parts.push_back(object);
  scene->primitives.push_back(triangle);
  REQUIRE(scene->finalizeLoading());
  return scene;
}

void
requireMatrixApprox(arma::mat const& actual,
                    arma::mat const& expected,
                    double const epsilon = 1e-12)
{
  REQUIRE(actual.n_rows == expected.n_rows);
  REQUIRE(actual.n_cols == expected.n_cols);
  REQUIRE(arma::approx_equal(actual, expected, "absdiff", epsilon));
}

} // namespace

TEST_CASE("Dynamic scene simulation state resets without geometry snapshots")
{
  auto scene = makeResetScene();
  auto object = std::dynamic_pointer_cast<DynSequentiableMovingObject>(
    scene->getDynObject(0));

  arma::mat const baselinePositions = object->positionMatrixFromPrimitives();
  arma::mat const baselineNormals = object->normalMatrixFromPrimitives();
  arma::colvec const baselineCentroid = object->getCentroid();

  scene->prepareSimulation(100);
  std::vector<bool> firstPhase;
  for (std::size_t i = 0; i < 10; ++i)
    firstPhase.push_back(scene->doSimStep());
  arma::mat const progressedPositions = object->positionMatrixFromPrimitives();
  REQUIRE_FALSE(arma::approx_equal(
    progressedPositions, baselinePositions, "absdiff", 1e-9));

  scene->prepareSimulation(100);
  requireMatrixApprox(object->positionMatrixFromPrimitives(),
                      baselinePositions);
  requireMatrixApprox(object->normalMatrixFromPrimitives(), baselineNormals);
  requireMatrixApprox(object->getCentroid(), baselineCentroid);

  // The scene, object, and observer intervals all restart at their original
  // phase, and every sequence (including the no-longer-current finite one)
  // starts over.
  std::vector<bool> repeatedPhase;
  for (std::size_t i = 0; i < 10; ++i)
    repeatedPhase.push_back(scene->doSimStep());
  REQUIRE(repeatedPhase == firstPhase);
  requireMatrixApprox(object->positionMatrixFromPrimitives(),
                      progressedPositions);

  // Leave the observer deliberately behind geometry, then verify that reset
  // synchronizes its KD tree before the first subsequent intersection.
  scene->resetSimulationState();
  glm::dvec3 const origin(0.0, 0.0, 10.0);
  glm::dvec3 const direction(0.0, 0.0, -1.0);
  REQUIRE(scene->getIntersection(origin, direction, false) != nullptr);
}

TEST_CASE("Dynamic scene repeat rejects consumed swap recipes")
{
  auto scene = makeResetScene();
  scene->parts[0]->sorh = std::make_shared<SwapOnRepeatHandler>();

  scene->prepareSimulation(100);
  try {
    scene->prepareSimulation(100);
    FAIL("A second playback with swap-on-repeat should be rejected");
  } catch (HeliosException const& error) {
    REQUIRE(std::string(error.what()).find("swap-on-repeat") !=
            std::string::npos);
  }
}
