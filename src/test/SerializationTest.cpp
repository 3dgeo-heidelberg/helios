#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <DetailedVoxel.h>
#include <Scene.h>
#include <SerialIO.h>
#include <cstdio>
#include <string>
#include <vector>

// validation functions
void validate(DetailedVoxel& dv1, DetailedVoxel& dv2) {
  glm::dvec3 dv1c = dv1.getCentroid();
  glm::dvec3 dv2c = dv2.getCentroid();
  REQUIRE(dv1c.x == dv2c.x);
  REQUIRE(dv1c.y == dv2c.y);
  REQUIRE(dv1c.z == dv2c.z);
  REQUIRE(dv1.halfSize == dv2.halfSize);
  for (size_t i = 0; i < dv1.getNumberOfIntValues(); i++) {
    REQUIRE(dv1.getIntValue(i) == dv2.getIntValue(i));
  }
  for (size_t i = 0; i < dv1.getNumberOfDoubleValues(); i++) {
    REQUIRE(dv1[i] == dv2[i]);
  }
  if (dv1.material != nullptr) REQUIRE(dv2.material != nullptr);
  if (dv2.material != nullptr) REQUIRE(dv1.material != nullptr);
  if (dv1.material != nullptr && dv2.material != nullptr) {
    for (size_t i = 0; i < 4; i++) {
      REQUIRE(dv1.material->ka[i] == dv2.material->ka[i]);
    }
  }
}

void validate(Voxel& v1, Voxel& v2) {
  glm::dvec3 v1c = v1.getCentroid();
  glm::dvec3 v2c = v2.getCentroid();
  REQUIRE(v1c.x == v2c.x);
  REQUIRE(v1c.y == v2c.y);
  REQUIRE(v1c.z == v2c.z);
  REQUIRE(v1.halfSize == v2.halfSize);
}

void validate(AABB& box1, AABB& box2) {
  double minX1 = box1.getMin().x;
  double maxX1 = box1.getMax().x;
  double minY1 = box1.getMin().y;
  double maxY1 = box1.getMax().y;
  double minZ1 = box1.getMin().z;
  double maxZ1 = box1.getMax().z;
  double minX2 = box2.getMin().x;
  double maxX2 = box2.getMax().x;
  double minY2 = box2.getMin().y;
  double maxY2 = box2.getMax().y;
  double minZ2 = box2.getMin().z;
  double maxZ2 = box2.getMax().z;

  REQUIRE(minX1 == minX2);
  REQUIRE(minY1 == minY2);
  REQUIRE(minZ1 == minZ2);
  REQUIRE(maxX1 == maxX2);
  REQUIRE(maxY1 == maxY2);
  REQUIRE(maxZ1 == maxZ2);
}

void validate(Triangle& t1, Triangle& t2) {
  double x1, x2, y1, y2, z1, z2;
  for (size_t i = 0; i < t1.getNumVertices(); i++) {
    x1 = t1.verts[i].getX();
    x2 = t2.verts[i].getX();
    REQUIRE(x1 == x2);
    y1 = t1.verts[i].getY();
    y2 = t2.verts[i].getY();
    REQUIRE(y1 == y2);
    z1 = t1.verts[i].getZ();
    z2 = t2.verts[i].getZ();
    REQUIRE(z1 == z2);
  }
}

TEST_CASE("Serialization Test") {
    std::string path = "SerializationTest.tmp";
    DetailedVoxel dv1(1.0, 2.0, 1.0, 1.0, 
        std::vector<int>({0, 1, 2}),
        std::vector<double>({1.0, 1.5, 2.0, 2.5, 3.0}));
    dv1.part = std::make_shared<ScenePart>();
    dv1.part->mPrimitives.push_back(&dv1);
    dv1.part->onRayIntersectionMode = "TRANSMITTIVE";
    dv1.material = std::make_shared<Material>();
    dv1.material->ka[0] = 1.0;
    dv1.material->ka[1] = 2.0;
    dv1.material->ka[2] = 3.0;
    dv1.material->ka[3] = 4.0;

    SECTION("DetailedVoxel Serialization") {
        SerialIO* sio = SerialIO::getInstance();
        sio->write<DetailedVoxel>(path, &dv1);
        DetailedVoxel* dv2 = sio->read<DetailedVoxel>(path);
        
        // validate detailed voxel
        validate(dv1, *dv2);
        delete dv2;
    }

    SECTION("Scene Serialization") {
        std::size_t nRepeats = 32;
        Scene scene1;
        Vertex t1v1, t1v2, t1v3;
        t1v1.pos = glm::dvec3(0.0, 0.0, 0.0);
        t1v2.pos = glm::dvec3(0.0, 0.0, 4.0);
        t1v3.pos = glm::dvec3(2.0, 0.0, 0.0);
        Triangle t1(t1v1, t1v2, t1v3);
        Voxel v1(1.0, 1.0, 1.0, 1.0);
        AABB box1(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(5.0, 5.0, 5.0));

        scene1.primitives.push_back(dv1.clone());
        scene1.primitives.push_back(t1.clone());
        scene1.primitives.push_back(v1.clone());
        scene1.primitives.push_back(box1.clone());
        for (std::size_t i = 0; i < nRepeats; i++) {
            scene1.primitives.push_back(dv1.clone());
        }

        std::shared_ptr<ScenePart> part = std::make_shared<ScenePart>();
        for (Primitive* prim : scene1.primitives) {
            prim->part = part;
            part->mPrimitives.push_back(prim);
        }
        scene1.finalizeLoading(true);
        std::shared_ptr<KDGroveFactory> kdgf = scene1.getKDGroveFactory();
        scene1.writeObject(path);
        scene1.setKDGroveFactory(kdgf);
        Scene* scene2 = Scene::readObject(path);
        validate(*(DetailedVoxel*)scene1.primitives[0], *(DetailedVoxel*)scene2->primitives[0]);
        validate(*(Triangle*)scene1.primitives[1], *(Triangle*)scene2->primitives[1]);
        validate(*(Voxel*)scene1.primitives[2], *(Voxel*)scene2->primitives[2]);
        validate(*(AABB*)scene1.primitives[3], *(AABB*)scene2->primitives[3]);

        for (size_t i = 0; i < nRepeats; i++) {
            validate(*(DetailedVoxel*)scene1.primitives[i + 4],
                     *(DetailedVoxel*)scene2->primitives[i + 4]);
        }

        delete scene2;
    }

    // Cleanup
    std::remove(path.c_str());
}