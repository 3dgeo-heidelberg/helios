#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#undef INFO
#undef WARN

#include <AABB.h>
#include <AxisSAHKDTreeFactory.h>
#include <FastSAHKDTreeFactory.h>
#include <KDGroveFactory.h>
#include <KDTreeFactory.h>
#include <KDTreeFactoryMaker.h>
#include <KDTreeNodeRoot.h>
#include <Material.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>
#include <SAHKDTreeFactory.h>
#include <Scene.h>
#include <ScenePart.h>
#include <Triangle.h>
#include <Voxel.h>
#include <scene/primitives/DetailedVoxel.h>
#include <util/HeliosException.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace {

void
populateMixedScene(Scene& scene)
{
  scene.id = "scene-cereal-id";
  scene.name = "Scene Cereal Roundtrip";
  scene.sourceFilePath = "synthetic_scene";
  scene.setDefaultReflectance(47.5);

  auto partA = std::make_shared<ScenePart>();
  partA->primitiveType = ScenePart::PrimitiveType::NONE;
  partA->mId = "partA";
  partA->onRayIntersectionMode = "TRANSMITTIVE";
  partA->onRayIntersectionArgument = 0.35;
  partA->randomShift = true;
  partA->subpartLimit = { 2 };
  partA->mOrigin = glm::dvec3(0.1, -0.2, 0.3);
  partA->mRotation = Rotation(1.0, 0.0, 0.0, 0.0, false);
  partA->mScale = 1.25;
  partA->forceOnGround = 1;
  partA->ladlut = std::make_shared<LadLut>();
  partA->ladlut->X = { 1.0, 2.0 };
  partA->ladlut->Y = { 3.0, 4.0 };
  partA->ladlut->Z = { 5.0, 6.0 };
  partA->ladlut->G = { 7.0, 8.0 };
  partA->ladlut->angles = { 0.25, 0.75 };

  auto partB = std::make_shared<ScenePart>();
  partB->primitiveType = ScenePart::PrimitiveType::VOXEL;
  partB->mId = "partB";
  partB->onRayIntersectionMode = "";
  partB->onRayIntersectionArgument = 0.0;
  partB->randomShift = false;
  partB->mOrigin = glm::dvec3(-0.3, 0.4, -0.1);
  partB->mRotation = Rotation(1.0, 0.0, 0.0, 0.0, false);
  partB->mScale = 1.0;
  partB->forceOnGround = 0;

  auto sharedMat = std::make_shared<Material>();
  sharedMat->name = "shared";
  sharedMat->reflectance = 18.5;
  sharedMat->specularity = 0.23;
  sharedMat->specularExponent = 6.0;
  sharedMat->classification = 2;
  sharedMat->isGround = false;
  sharedMat->useVertexColors = true;
  sharedMat->ka[0] = 0.1f;
  sharedMat->kd[1] = 0.2f;
  sharedMat->ks[2] = 0.3f;

  auto detailedMat = std::make_shared<Material>();
  detailedMat->name = "detailed";
  detailedMat->reflectance = 42.0;
  detailedMat->classification = 6;
  detailedMat->isGround = true;
  detailedMat->ka[0] = 0.7f;
  detailedMat->kd[1] = 0.8f;
  detailedMat->ks[2] = 0.9f;

  Triangle* tri = new Triangle(
    Vertex(-0.6, -0.6, 0.0), Vertex(0.7, -0.4, 0.0), Vertex(0.0, 0.8, 0.0));
  tri->part = partA;
  tri->material = sharedMat;

  Voxel* vox = new Voxel(2.0, 0.0, 0.0, 0.5);
  vox->part = partA;
  vox->material = sharedMat;
  vox->numPoints = 12;
  vox->r = 0.4;
  vox->g = 0.6;
  vox->b = 0.8;
  vox->color = Color4f(0.4f, 0.6f, 0.8f, 1.0f);
  vox->update();

  DetailedVoxel* detailed = new DetailedVoxel(
    -2.0,
    0.0,
    0.0,
    0.5,
    std::vector<int>({ 7, 11 }),
    std::vector<double>(
      { 0.2, 0.5, 0.9, 1.3, 1.7, 2.1, 2.5, 2.9, 3.3, 3.7, 4.1 }));
  detailed->part = partB;
  detailed->material = detailedMat;
  detailed->setMaxPad(9.1);
  detailed->update();

  partA->mPrimitives = { tri, vox };
  partB->mPrimitives = { detailed };

  partA->computeCentroid(true);
  partB->computeCentroid(true);

  scene.primitives = { tri, vox, detailed };
  scene.parts = { partA, partB };

  std::shared_ptr<AABB> bbox = AABB::getForPrimitives(scene.primitives);
  scene.setBBox(bbox);
  scene.setBBoxCRS(bbox);
}

struct TempArchivePath
{
  std::filesystem::path path;

  explicit TempArchivePath(std::string const& name)
  {
    auto const nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           (name + "_" + std::to_string(nonce) + ".bin");
  }

  ~TempArchivePath()
  {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
};

} // namespace

TEST_CASE("SceneSerialization: round-trip without built KD structures")
{
  Scene scene;
  populateMixedScene(scene);
  scene.setKDGroveFactory(nullptr);

  TempArchivePath archive("scene_serialization_no_kd");
  scene.saveCereal(archive.path.string());

  Scene loaded;
  loaded.loadCereal(archive.path.string());

  REQUIRE(loaded.getKDGroveFactory() == nullptr);
  REQUIRE(loaded.getKDGrove() == nullptr);
  REQUIRE(loaded.getRaycaster() == nullptr);

  REQUIRE(loaded.id == scene.id);
  REQUIRE(loaded.name == scene.name);
  REQUIRE(loaded.sourceFilePath == scene.sourceFilePath);
  REQUIRE(loaded.getDefaultReflectance() ==
          Catch::Approx(scene.getDefaultReflectance()));
  REQUIRE(loaded.primitives.size() == 3);
  REQUIRE(loaded.parts.size() == 2);

  REQUIRE(dynamic_cast<Triangle*>(loaded.primitives[0]) != nullptr);
  REQUIRE(dynamic_cast<Voxel*>(loaded.primitives[1]) != nullptr);
  REQUIRE(dynamic_cast<DetailedVoxel*>(loaded.primitives[2]) != nullptr);

  REQUIRE(loaded.primitives[0]->part == loaded.parts[0]);
  REQUIRE(loaded.primitives[1]->part == loaded.parts[0]);
  REQUIRE(loaded.primitives[2]->part == loaded.parts[1]);
  REQUIRE(loaded.parts[0]->mPrimitives.size() == 2);
  REQUIRE(loaded.parts[1]->mPrimitives.size() == 1);
  REQUIRE(loaded.parts[0]->ladlut != nullptr);
  REQUIRE(loaded.parts[0]->ladlut->angles.size() == 2);
  REQUIRE(loaded.parts[0]->ladlut->angles[0] == Catch::Approx(0.25));

  REQUIRE(loaded.primitives[0]->material != nullptr);
  REQUIRE(loaded.primitives[1]->material != nullptr);
  REQUIRE(loaded.primitives[2]->material != nullptr);
  REQUIRE(loaded.primitives[0]->material == loaded.primitives[1]->material);
  REQUIRE(loaded.primitives[0]->material != loaded.primitives[2]->material);

  auto* loadedDetailed = dynamic_cast<DetailedVoxel*>(loaded.primitives[2]);
  REQUIRE(loadedDetailed != nullptr);
  REQUIRE(loadedDetailed->getNumberOfIntValues() == 2);
  REQUIRE(loadedDetailed->getIntValue(0) == 7);
  REQUIRE(loadedDetailed->getNumberOfDoubleValues() == 11);
  REQUIRE(loadedDetailed->getDoubleValue(3) == Catch::Approx(1.3));
  REQUIRE(loadedDetailed->getMaxPad() == Catch::Approx(9.1));
}

TEST_CASE("SceneSerialization: built KD round-trip with factory variants")
{
  struct Variant
  {
    std::string name;
    std::function<std::shared_ptr<KDTreeFactory>()> makeFactory;
    std::function<void(std::shared_ptr<KDTreeFactory> const&)> assertFactory;
  };

  std::vector<Variant> const variants = {
    { "Simple",
      []() -> std::shared_ptr<KDTreeFactory> {
        return KDTreeFactoryMaker::makeSimple();
      },
      [](std::shared_ptr<KDTreeFactory> const& factory) {
        REQUIRE(factory != nullptr);
        REQUIRE(dynamic_cast<MultiThreadKDTreeFactory*>(factory.get()) ==
                nullptr);
        REQUIRE(dynamic_cast<SimpleKDTreeFactory*>(factory.get()) != nullptr);
      } },
    { "SAH",
      []() -> std::shared_ptr<KDTreeFactory> {
        auto sah = KDTreeFactoryMaker::makeSAH(13);
        sah->setInteriorCost(1.25);
        sah->setLeafCost(0.75);
        sah->setObjectCost(1.50);
        return sah;
      },
      [](std::shared_ptr<KDTreeFactory> const& factory) {
        auto* sah = dynamic_cast<SAHKDTreeFactory*>(factory.get());
        REQUIRE(sah != nullptr);
        REQUIRE(dynamic_cast<MultiThreadKDTreeFactory*>(factory.get()) ==
                nullptr);
        REQUIRE(dynamic_cast<AxisSAHKDTreeFactory*>(factory.get()) == nullptr);
        REQUIRE(dynamic_cast<FastSAHKDTreeFactory*>(factory.get()) == nullptr);
        REQUIRE(sah->getLossNodes() == 13);
        REQUIRE(sah->getInteriorCost() == Catch::Approx(1.25));
        REQUIRE(sah->getLeafCost() == Catch::Approx(0.75));
        REQUIRE(sah->getObjectCost() == Catch::Approx(1.50));
      } },
    { "FastSAH-MT",
      []() -> std::shared_ptr<KDTreeFactory> {
        auto fastMt = KDTreeFactoryMaker::makeFastSAHMultiThread(17, 2, 2);
        auto fastBase =
          std::dynamic_pointer_cast<FastSAHKDTreeFactory>(fastMt->getKdtf());
        fastBase->setInteriorCost(2.0);
        fastBase->setLeafCost(1.0);
        fastBase->setObjectCost(3.0);
        return fastMt;
      },
      [](std::shared_ptr<KDTreeFactory> const& factory) {
        auto* mt = dynamic_cast<MultiThreadSAHKDTreeFactory*>(factory.get());
        REQUIRE(mt != nullptr);
        REQUIRE(mt->getNumJobs() == 2);
        REQUIRE(mt->getGeomJobs() == 2);
        auto base =
          std::dynamic_pointer_cast<FastSAHKDTreeFactory>(mt->getKdtf());
        REQUIRE(base != nullptr);
        REQUIRE(base->getLossNodes() == 17);
        REQUIRE(base->getInteriorCost() == Catch::Approx(2.0));
        REQUIRE(base->getLeafCost() == Catch::Approx(1.0));
        REQUIRE(base->getObjectCost() == Catch::Approx(3.0));
      } },
  };

  for (Variant const& variant : variants) {
    SECTION(variant.name)
    {
      Scene scene;
      populateMixedScene(scene);
      scene.setKDGroveFactory(
        std::make_shared<KDGroveFactory>(variant.makeFactory()));
      scene.buildKDGrove(true);

      glm::dvec3 const rayOrigin(0.0, 0.0, 2.0);
      glm::dvec3 const rayDir(0.0, 0.0, -1.0);
      std::shared_ptr<RaySceneIntersection> const before =
        scene.getIntersection(rayOrigin, rayDir, false);
      REQUIRE(before != nullptr);
      REQUIRE(dynamic_cast<Triangle*>(before->prim) != nullptr);

      auto const beforeRoot = dynamic_cast<KDTreeNodeRoot const*>(
        scene.getKDGrove()->getTreeShared(0)->root.get());
      REQUIRE(beforeRoot != nullptr);

      TempArchivePath archive("scene_serialization_with_kd");
      scene.saveCereal(archive.path.string());

      Scene loaded;
      loaded.loadCereal(archive.path.string());

      REQUIRE(loaded.getKDGroveFactory() != nullptr);
      REQUIRE(loaded.getKDGroveFactory()->getKdtf() != nullptr);
      variant.assertFactory(loaded.getKDGroveFactory()->getKdtf());

      REQUIRE(loaded.getKDGrove() != nullptr);
      REQUIRE(loaded.getRaycaster() != nullptr);
      REQUIRE(loaded.getKDGrove()->getNumTrees() ==
              scene.getKDGrove()->getNumTrees());

      auto const afterRoot = dynamic_cast<KDTreeNodeRoot const*>(
        loaded.getKDGrove()->getTreeShared(0)->root.get());
      REQUIRE(afterRoot != nullptr);
      REQUIRE(afterRoot->stats_numLeaves == beforeRoot->stats_numLeaves);
      REQUIRE(afterRoot->stats_numInterior == beforeRoot->stats_numInterior);
      REQUIRE(afterRoot->stats_maxDepthReached ==
              beforeRoot->stats_maxDepthReached);

      std::shared_ptr<RaySceneIntersection> const after =
        loaded.getIntersection(rayOrigin, rayDir, false);
      REQUIRE(after != nullptr);
      REQUIRE(dynamic_cast<Triangle*>(after->prim) != nullptr);
      REQUIRE(after->hitDistance ==
              Catch::Approx(before->hitDistance).margin(1e-9));
      REQUIRE(after->point.x == Catch::Approx(before->point.x).margin(1e-9));
      REQUIRE(after->point.y == Catch::Approx(before->point.y).margin(1e-9));
      REQUIRE(after->point.z == Catch::Approx(before->point.z).margin(1e-9));
    }
  }
}

TEST_CASE("SceneSerialization: compression levels and validation")
{
  struct CompressionVariant
  {
    int level;
    std::string name;
  };

  std::vector<CompressionVariant> const variants = {
    { 0, "none" },
    { 1, "fast" },
    { 6, "default" },
    { 9, "best" },
  };

  for (CompressionVariant const& variant : variants) {
    SECTION(variant.name)
    {
      Scene scene;
      populateMixedScene(scene);
      scene.setKDGroveFactory(nullptr);

      TempArchivePath archive("scene_serialization_compression_" +
                              variant.name);
      scene.saveCereal(archive.path.string(), variant.level);

      Scene loaded;
      loaded.loadCereal(archive.path.string());

      REQUIRE(loaded.id == scene.id);
      REQUIRE(loaded.name == scene.name);
      REQUIRE(loaded.sourceFilePath == scene.sourceFilePath);
      REQUIRE(loaded.getDefaultReflectance() ==
              Catch::Approx(scene.getDefaultReflectance()));
      REQUIRE(loaded.primitives.size() == scene.primitives.size());
      REQUIRE(loaded.parts.size() == scene.parts.size());
      REQUIRE(loaded.getKDGrove() == nullptr);
      REQUIRE(loaded.getRaycaster() == nullptr);
    }
  }

  SECTION("invalid negative level throws")
  {
    Scene scene;
    populateMixedScene(scene);
    TempArchivePath archive("scene_serialization_compression_invalid_neg");
    REQUIRE_THROWS_AS(scene.saveCereal(archive.path.string(), -1),
                      HeliosException);
  }

  SECTION("invalid large level throws")
  {
    Scene scene;
    populateMixedScene(scene);
    TempArchivePath archive("scene_serialization_compression_invalid_large");
    REQUIRE_THROWS_AS(scene.saveCereal(archive.path.string(), 10),
                      HeliosException);
  }
}
