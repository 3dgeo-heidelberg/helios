#pragma once

#include <BaseTest.h>
#include <platform/LinearPathPlatform.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>
#include <scene/Material.h>
#include <scene/Scene.h>
#include <scene/primitives/Voxel.h>

namespace HeliosTests {

/**
 * @brief Test for full-waveform snapToSurface behavior
 */
class SnapToSurfaceTest : public BaseTest
{
public:
  double const eps = 1e-6;

  SnapToSurfaceTest()
    : BaseTest("Snap to surface test")
  {
  }

  bool run() override;

protected:
  struct RunnableFixture
  {
    std::shared_ptr<Scanner> scanner;
    std::shared_ptr<LinearPathPlatform> platform;
    std::shared_ptr<Scene> scene;
    std::shared_ptr<FullWaveformPulseDetector> detector;
    std::shared_ptr<std::vector<Measurement>> allMeasurements;
    std::shared_ptr<std::mutex> allMeasurementsMutex;
    SimulatedPulse pulse;
    std::unique_ptr<FullWaveformPulseRunnable> runnable;

    RunnableFixture(bool const snapToSurface, int const beamSampleQuality)
      : scanner(std::make_shared<SingleScanner>(0.0003,
                                                glm::dvec3(0, 0, 0),
                                                Rotation(),
                                                std::list<int>({ 100000 }),
                                                5.0,
                                                "scanDev0",
                                                4.0,
                                                1.0,
                                                0.99,
                                                0.15,
                                                1.0,
                                                1064,
                                                nullptr,
                                                false,
                                                false,
                                                false,
                                                false,
                                                false))
      , platform(std::make_shared<LinearPathPlatform>())
      , scene(std::make_shared<Scene>())
      , detector(
          std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01))
      , allMeasurements(std::make_shared<std::vector<Measurement>>())
      , allMeasurementsMutex(std::make_shared<std::mutex>())
      , pulse(glm::dvec3(0, 0, 0), Rotation(), 0.0, 0u, 0, 0u)
    {
      FWFSettings fwfSettings;
      fwfSettings.snapToSurface = snapToSurface;
      fwfSettings.beamSampleQuality = beamSampleQuality;
      fwfSettings.winSize_ns = 1.0;
      scanner->setFWFSettings(fwfSettings);
      scanner->setDetector(detector);
      scanner->setReceivedEnergyMin(0.1);
      scanner->setMaxNOR(1);
      scanner->setTimeWave(std::vector<double>({ 1.0 }));
      scanner->platform = platform;
      scanner->allMeasurements = allMeasurements;
      scanner->allMeasurementsMutex = allMeasurementsMutex;
      platform->scene = scene;
      runnable = std::make_unique<FullWaveformPulseRunnable>(scanner, pulse);
    }
  };

  bool testSnapToSurfaceSelectsInnermostContributingSurface();
  bool testSnapToSurfaceBreaksSameRingTiesByShortestDistance();
  bool testSnapToSurfaceFallsBackWhenBeamSampleQualityIsOne();

  static glm::dvec3 normalize(glm::dvec3 const& v)
  {
    return v / glm::length(v);
  }

  bool validateMeasurement(Measurement const& measurement,
                           glm::dvec3 const& expectedDirection,
                           double const expectedDistance,
                           std::string const& expectedHitObjectId,
                           int const expectedClassification)
  {
    if (measurement.hitObjectId != expectedHitObjectId)
      return false;
    if (measurement.classification != expectedClassification)
      return false;
    if (std::fabs(measurement.distance - expectedDistance) > eps)
      return false;
    if (glm::length(measurement.beamDirection - expectedDirection) > eps)
      return false;
    return true;
  }
};

bool
SnapToSurfaceTest::run()
{
  if (!testSnapToSurfaceSelectsInnermostContributingSurface())
    return false;
  if (!testSnapToSurfaceBreaksSameRingTiesByShortestDistance())
    return false;
  if (!testSnapToSurfaceFallsBackWhenBeamSampleQualityIsOne())
    return false;
  return true;
}

bool
SnapToSurfaceTest::testSnapToSurfaceSelectsInnermostContributingSurface()
{
  RunnableFixture fixture(true, 3);

  auto innerPart = std::make_shared<ScenePart>();
  innerPart->mId = "inner_ring_candidate";
  auto innerMaterial = std::make_shared<Material>();
  innerMaterial->classification = 3;
  Voxel innerVoxel(0.8, 10.3, 0.0, 0.5);
  innerVoxel.part = innerPart;
  innerVoxel.material = innerMaterial;

  auto outerPart = std::make_shared<ScenePart>();
  outerPart->mId = "outer_ring_candidate";
  auto outerMaterial = std::make_shared<Material>();
  outerMaterial->classification = 7;
  Voxel outerVoxel(0.1, 10.1, 0.0, 0.5);
  outerVoxel.part = outerPart;
  outerVoxel.material = outerMaterial;

  std::vector<RaySceneIntersection> intersects(2);
  intersects[0].prim = &innerVoxel;
  intersects[0].point = glm::dvec3(0.8, 10.3, 0.0);
  intersects[1].prim = &outerVoxel;
  intersects[1].point = glm::dvec3(0.1, 10.1, 0.0);

  std::vector<DiscreteSubrayReturn> discreteSubrayReturns(2);
  discreteSubrayReturns[0].intensity = 1.0;
  discreteSubrayReturns[0].subrayRadiusStep = 1;
  discreteSubrayReturns[0].intersectsIndex = 0;
  discreteSubrayReturns[1].intensity = 1.0;
  discreteSubrayReturns[1].subrayRadiusStep = 2;
  discreteSubrayReturns[1].intersectsIndex = 1;

  std::vector<Measurement> pointsMeasurement;
  int numReturns = 0;
  std::vector<std::vector<double>> apMatrix;
  std::vector<double> fullwave({ 10.0, 0.0, 0.0 });

  fixture.runnable->digestFullWaveform(pointsMeasurement,
                                       numReturns,
                                       apMatrix,
                                       fullwave,
                                       intersects,
                                       glm::dvec3(0.0, 1.0, 0.0),
                                       1.0,
                                       3,
                                       0,
                                       33.5,
                                       discreteSubrayReturns);

  if (numReturns != 1 || pointsMeasurement.size() != 1)
    return false;

  glm::dvec3 const expectedDirection = normalize(intersects[0].point);
  double const expectedDistance = glm::length(intersects[0].point);
  return validateMeasurement(pointsMeasurement[0],
                             expectedDirection,
                             expectedDistance,
                             "inner_ring_candidate",
                             3);
}

bool
SnapToSurfaceTest::testSnapToSurfaceBreaksSameRingTiesByShortestDistance()
{
  RunnableFixture fixture(true, 3);

  auto nearPart = std::make_shared<ScenePart>();
  nearPart->mId = "near_candidate";
  auto nearMaterial = std::make_shared<Material>();
  nearMaterial->classification = 5;
  Voxel nearVoxel(0.4, 10.1, 0.0, 0.5);
  nearVoxel.part = nearPart;
  nearVoxel.material = nearMaterial;

  auto farPart = std::make_shared<ScenePart>();
  farPart->mId = "far_candidate";
  auto farMaterial = std::make_shared<Material>();
  farMaterial->classification = 9;
  Voxel farVoxel(0.1, 10.3, 0.0, 0.5);
  farVoxel.part = farPart;
  farVoxel.material = farMaterial;

  std::vector<RaySceneIntersection> intersects(2);
  intersects[0].prim = &nearVoxel;
  intersects[0].point = glm::dvec3(0.4, 10.1, 0.0);
  intersects[1].prim = &farVoxel;
  intersects[1].point = glm::dvec3(0.1, 10.3, 0.0);

  std::vector<DiscreteSubrayReturn> discreteSubrayReturns(2);
  discreteSubrayReturns[0].intensity = 1.0;
  discreteSubrayReturns[0].subrayRadiusStep = 1;
  discreteSubrayReturns[0].intersectsIndex = 0;
  discreteSubrayReturns[1].intensity = 1.0;
  discreteSubrayReturns[1].subrayRadiusStep = 1;
  discreteSubrayReturns[1].intersectsIndex = 1;

  std::vector<Measurement> pointsMeasurement;
  int numReturns = 0;
  std::vector<std::vector<double>> apMatrix;
  std::vector<double> fullwave({ 10.0, 0.0, 0.0 });

  fixture.runnable->digestFullWaveform(pointsMeasurement,
                                       numReturns,
                                       apMatrix,
                                       fullwave,
                                       intersects,
                                       glm::dvec3(0.0, 1.0, 0.0),
                                       1.0,
                                       3,
                                       0,
                                       33.5,
                                       discreteSubrayReturns);

  if (numReturns != 1 || pointsMeasurement.size() != 1)
    return false;

  glm::dvec3 const expectedDirection = normalize(intersects[0].point);
  double const expectedDistance = glm::length(intersects[0].point);
  return validateMeasurement(pointsMeasurement[0],
                             expectedDirection,
                             expectedDistance,
                             "near_candidate",
                             5);
}

bool
SnapToSurfaceTest::testSnapToSurfaceFallsBackWhenBeamSampleQualityIsOne()
{
  RunnableFixture fixture(true, 1);

  auto fallbackPart = std::make_shared<ScenePart>();
  fallbackPart->mId = "fallback_candidate";
  auto fallbackMaterial = std::make_shared<Material>();
  fallbackMaterial->classification = 3;
  Voxel fallbackVoxel(2.0, 9.95, 0.0, 0.5);
  fallbackVoxel.part = fallbackPart;
  fallbackVoxel.material = fallbackMaterial;

  auto snapPart = std::make_shared<ScenePart>();
  snapPart->mId = "snap_candidate";
  auto snapMaterial = std::make_shared<Material>();
  snapMaterial->classification = 7;
  Voxel snapVoxel(0.1, 10.3, 0.0, 0.5);
  snapVoxel.part = snapPart;
  snapVoxel.material = snapMaterial;

  std::vector<RaySceneIntersection> intersects(2);
  intersects[0].prim = &fallbackVoxel;
  intersects[0].point = glm::dvec3(2.0, 9.95, 0.0);
  intersects[1].prim = &snapVoxel;
  intersects[1].point = glm::dvec3(0.1, 10.3, 0.0);

  std::vector<DiscreteSubrayReturn> discreteSubrayReturns(2);
  discreteSubrayReturns[0].intensity = 1.0;
  discreteSubrayReturns[0].subrayRadiusStep = 0;
  discreteSubrayReturns[0].intersectsIndex = 0;
  discreteSubrayReturns[1].intensity = 1.0;
  discreteSubrayReturns[1].subrayRadiusStep = 1;
  discreteSubrayReturns[1].intersectsIndex = 1;

  std::vector<Measurement> pointsMeasurement;
  int numReturns = 0;
  std::vector<std::vector<double>> apMatrix;
  std::vector<double> fullwave({ 10.0, 0.0, 0.0 });

  fixture.runnable->digestFullWaveform(pointsMeasurement,
                                       numReturns,
                                       apMatrix,
                                       fullwave,
                                       intersects,
                                       glm::dvec3(0.0, 1.0, 0.0),
                                       1.0,
                                       3,
                                       0,
                                       33.5,
                                       discreteSubrayReturns);

  if (numReturns != 1 || pointsMeasurement.size() != 1)
    return false;

  glm::dvec3 const expectedDirection = glm::dvec3(0.0, 1.0, 0.0);
  double const expectedDistance = SPEEDofLIGHT_mPerNanosec * 33.5;
  return validateMeasurement(pointsMeasurement[0],
                             expectedDirection,
                             expectedDistance,
                             "fallback_candidate",
                             3);
}

} // namespace HeliosTests
