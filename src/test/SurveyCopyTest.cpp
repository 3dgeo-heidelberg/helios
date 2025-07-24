#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <FullWaveformPulseDetector.h>
#include <HelicopterPlatform.h>
#include <OscillatingMirrorBeamDeflector.h>
#include <Survey.h>
#include <scanner/SingleScanner.h>

TEST_CASE("Survey Copy Test")
{
  // Build base Survey
  std::shared_ptr<Survey> survey = std::make_shared<Survey>();
  survey->name = "MySurvey";
  survey->numRuns = 1;
  survey->simSpeedFactor = 1;

  std::list<int> pulseFreqs = { 100, 30, 70 };
  survey->scanner =
    std::make_shared<SingleScanner>(0.1,
                                    glm::dvec3(2.0, 3.0, 0.0),
                                    Rotation(0.0, 0.0, 0.0, 0.0, true),
                                    pulseFreqs,
                                    4.0,
                                    "MyScanner",
                                    80.5,
                                    3.0,
                                    0.9,
                                    0.7,
                                    0.8,
                                    100,
                                    false,
                                    false,
                                    false,
                                    true,
                                    false);
  survey->scanner->setScannerHead(
    std::make_shared<ScannerHead>(glm::dvec3(0.4, 0.7, 0.1), 0.067));
  survey->scanner->setBeamDeflector(
    std::make_shared<OscillatingMirrorBeamDeflector>(
      3.141592, 1400.5, 70.8, 1));
  survey->scanner->platform = std::make_shared<HelicopterPlatform>();
  survey->scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(survey->scanner, 1.5, 0.1));
  survey->legs.push_back(std::make_shared<Leg>());
  survey->legs[0]->mPlatformSettings = std::make_shared<PlatformSettings>();
  survey->legs[0]->mPlatformSettings->onGround = false;

  std::shared_ptr<Scene> baseScene = std::make_shared<Scene>();
  survey->scanner->platform->scene = baseScene;

  baseScene->primitives.push_back(new Triangle(Vertex(), Vertex(), Vertex()));
  baseScene->primitives[0]->part = std::make_shared<ScenePart>();
  baseScene->primitives[0]->part->mPrimitives.push_back(
    baseScene->primitives[0]);
  baseScene->primitives[0]->part->onRayIntersectionMode = "TRANSMITTIVE";

  baseScene->primitives.push_back(
    new DetailedVoxel(glm::dvec3(0.0, 0.0, 0.5),
                      2.15,
                      std::vector<int>({ 1, 2 }),
                      std::vector<double>({ 0.1, 0.2, 0.3 })));
  baseScene->primitives[1]->material = std::make_shared<Material>();
  baseScene->primitives[1]->material->ka[0] = 1.1;
  baseScene->primitives[1]->material->ks[1] = 1.2;

  // Copy base Survey
  std::shared_ptr<Survey> copy = std::make_shared<Survey>(*survey, true);

  // Modify the copy
  copy->name = "CopiedSurvey";
  copy->numRuns = 0;
  copy->scanner->name = "CopiedScanner";
  Rotation& copyMRA =
    copy->scanner->getScannerHead()->getMountRelativeAttitudeByReference();
  copyMRA.setQ3(copyMRA.getQ3() + 0.1);
  copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz += 1.0;
  copy->scanner->platform->cfg_device_relativeMountPosition.x += 0.01;

  HelicopterPlatform* hp = ((HelicopterPlatform*)copy->scanner->platform.get());
  glm::dvec3& speedXy = hp->getSpeedXyByReference();
  speedXy.x += 0.1;
  Rotation& r = hp->getRotationByReference();
  r.setQ2(r.getQ2() + 0.1);

  copy->scanner->getFWFSettings().minEchoWidth += 0.001;
  copy->legs[0]->mPlatformSettings->onGround = true;

  std::shared_ptr<Scene> copyScene = copy->scanner->platform->scene;
  copyScene->primitives[0]->getVertices()[0].pos.x += 0.1;
  copyScene->primitives[0]->part->onRayIntersectionArgument += 0.034;
  copyScene->primitives[1]->material->ks[1] += 0.1;
  DetailedVoxel* copyDv = (DetailedVoxel*)copyScene->primitives[1];
  (*copyDv)[1] += 0.1;

  // Validate the copy
  REQUIRE(copy->name != survey->name);
  REQUIRE(copy->numRuns != survey->numRuns);
  REQUIRE(copy->simSpeedFactor == survey->simSpeedFactor);

  REQUIRE(copy->scanner->name != survey->scanner->name);
  REQUIRE(copy->scanner->getNumTimeBins() == survey->scanner->getNumTimeBins());
  REQUIRE(copy->scanner->isCalcEchowidth() ==
          survey->scanner->isCalcEchowidth());
  REQUIRE(copy->scanner->getFWFSettings().minEchoWidth !=
          survey->scanner->getFWFSettings().minEchoWidth);
  REQUIRE(copy->scanner->getFWFSettings().apertureDiameter ==
          survey->scanner->getFWFSettings().apertureDiameter);
  REQUIRE(copy->scanner->getScannerHead()->getRotatePerSecMax() ==
          survey->scanner->getScannerHead()->getRotatePerSecMax());
  Rotation& baseMRA =
    survey->scanner->getScannerHead()->getMountRelativeAttitudeByReference();
  REQUIRE(copyMRA.getQ0() == baseMRA.getQ0());
  REQUIRE(copyMRA.getQ3() != baseMRA.getQ3());

  REQUIRE(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz ==
          survey->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz);
  REQUIRE(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz !=
          survey->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz);
  REQUIRE(copy->scanner->platform->cfg_device_relativeMountPosition.x !=
          survey->scanner->platform->cfg_device_relativeMountPosition.x);
  REQUIRE(copy->scanner->platform->cfg_device_relativeMountPosition.y ==
          survey->scanner->platform->cfg_device_relativeMountPosition.y);

  HelicopterPlatform* copyHp =
    static_cast<HelicopterPlatform*>(copy->scanner->platform.get());
  HelicopterPlatform* baseHp =
    static_cast<HelicopterPlatform*>(survey->scanner->platform.get());

  glm::dvec3& copySpeedXy = copyHp->getSpeedXyByReference();
  glm::dvec3& baseSpeedXy = baseHp->getSpeedXyByReference();
  REQUIRE(copySpeedXy.x != baseSpeedXy.x);
  REQUIRE(copySpeedXy.y == baseSpeedXy.y);

  Rotation& copyRot = copyHp->getRotationByReference();
  Rotation& baseRot = baseHp->getRotationByReference();
  REQUIRE(copyRot.getQ1() == baseRot.getQ1());
  REQUIRE(copyRot.getQ2() != baseRot.getQ2());

  REQUIRE(copy->legs[0]->mPlatformSettings->onGround !=
          survey->legs[0]->mPlatformSettings->onGround);
  REQUIRE(copy->legs[0]->mPlatformSettings->stopAndTurn ==
          survey->legs[0]->mPlatformSettings->stopAndTurn);

  REQUIRE(copyScene->primitives[0]->getVertices()[0].pos.x !=
          baseScene->primitives[0]->getVertices()[0].pos.x);
  REQUIRE(copyScene->primitives[0]->getVertices()[0].pos.y ==
          baseScene->primitives[0]->getVertices()[0].pos.y);
  REQUIRE(copyScene->primitives[0]->part->onRayIntersectionArgument !=
          baseScene->primitives[0]->part->onRayIntersectionArgument);
  REQUIRE(copyScene->primitives[1]->material->ks[0] ==
          baseScene->primitives[1]->material->ks[0]);
  REQUIRE(copyScene->primitives[1]->material->ks[1] !=
          baseScene->primitives[1]->material->ks[1]);

  DetailedVoxel* baseDv = (DetailedVoxel*)baseScene->primitives[1];
  REQUIRE((*copyDv)[1] != (*baseDv)[1]);
  REQUIRE((*copyDv)[0] == (*baseDv)[0]);
}
