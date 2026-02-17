#include <catch2/catch_test_macros.hpp>

#include <helios/adt/grove/KDGrove.h>
#include <helios/adt/kdtree/FastSAHKDTreeFactory.h>
#include <helios/alg/raycast/GroveKDTreeRaycaster.h>
#include <helios/scene/primitives/Primitive.h>
#include <helios/scene/primitives/Triangle.h>

#include <memory>
#include <vector>

struct GroveTestFixture
{
  KDGrove kdg;
  std::vector<std::shared_ptr<GroveKDTreeRaycaster>> trees;
  std::vector<std::vector<Primitive*>> primitives;

  GroveTestFixture()
  {
    buildPrimitives();
    buildTrees();
    buildGrove();
  }

  ~GroveTestFixture()
  {
    for (auto& tris : primitives) {
      for (Primitive* tri : tris) {
        delete static_cast<Triangle*>(tri);
      }
    }
  }

  void buildPrimitives()
  {
    // First tree
    std::vector<Primitive*> tris1;
    tris1.push_back(
      new Triangle(Vertex(-1, -1, -1), Vertex(1, -1, -1), Vertex(0, 1, -1)));
    tris1.push_back(
      new Triangle(Vertex(-1, -1, -1), Vertex(1, -1, -1), Vertex(0, 0, 1)));
    tris1.push_back(
      new Triangle(Vertex(1, -1, -1), Vertex(0, 0, 1), Vertex(0, 1, -1)));
    tris1.push_back(
      new Triangle(Vertex(0, 1, -1), Vertex(0, 0, 1), Vertex(-1, -1, -1)));
    primitives.push_back(tris1);

    // Second tree
    std::vector<Primitive*> tris2;
    tris2.push_back(
      new Triangle(Vertex(0, 0, -1), Vertex(2, 0, -1), Vertex(1, 2, -1)));
    tris2.push_back(
      new Triangle(Vertex(0, 0, -1), Vertex(2, 0, -1), Vertex(1, 1, 1)));
    tris2.push_back(
      new Triangle(Vertex(2, 0, -1), Vertex(1, 1, 1), Vertex(1, 2, -1)));
    tris2.push_back(
      new Triangle(Vertex(1, 2, -1), Vertex(1, 1, 1), Vertex(0, 0, -1)));
    primitives.push_back(tris2);

    // Third tree
    std::vector<Primitive*> tris3;
    tris3.push_back(
      new Triangle(Vertex(4, 4, -1), Vertex(6, 4, -1), Vertex(5, 6, -1)));
    tris3.push_back(
      new Triangle(Vertex(4, 4, -1), Vertex(6, 4, -1), Vertex(5, 5, 1)));
    tris3.push_back(
      new Triangle(Vertex(6, 4, -1), Vertex(5, 5, 1), Vertex(5, 6, -1)));
    tris3.push_back(
      new Triangle(Vertex(5, 6, -1), Vertex(5, 5, 1), Vertex(4, 4, -1)));
    primitives.push_back(tris3);

    // Fourth tree
    std::vector<Primitive*> tris4;
    tris4.push_back(
      new Triangle(Vertex(9, 9, 0), Vertex(11, 9, 0), Vertex(10, 11, 0)));
    tris4.push_back(
      new Triangle(Vertex(9, 9, 0), Vertex(11, 9, 0), Vertex(10, 10, -3)));
    tris4.push_back(
      new Triangle(Vertex(11, 9, 0), Vertex(10, 10, -3), Vertex(10, 11, 0)));
    tris4.push_back(
      new Triangle(Vertex(10, 11, 0), Vertex(10, 10, -3), Vertex(9, 9, 0)));
    primitives.push_back(tris4);
  }

  void buildTrees()
  {
    FastSAHKDTreeFactory kdtf(32, 1, 1, 1);
    for (auto& tris : primitives) {
      std::shared_ptr<LightKDTreeNode> tree(
        kdtf.makeFromPrimitives(tris, true, false));
      trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree));
    }
  }

  void buildGrove()
  {
    for (auto& tree : trees) {
      kdg.addTree(tree);
    }
  }
};

TEST_CASE("GroveTest: Loop mechanics")
{
  GroveTestFixture fixture;
  auto& kdg = fixture.kdg;
  auto& trees = fixture.trees;

  size_t i, j, iMax, m;

  // For loop test
  m = kdg.getNumTrees();
  for (j = 0; j < 3; ++j) {
    for (i = 0, iMax = 0; i < m; ++i) {
      REQUIRE(trees[i] == kdg[i]);
      if (i > iMax)
        iMax = i;
    }
  }
  REQUIRE(iMax == m - 1);
  REQUIRE(m == trees.size());

  // While loop test
  for (j = 0; j < 3; ++j) {
    i = 0;
    kdg.toZeroTree();
    while (kdg.hasNextTree()) {
      REQUIRE(trees[i] == kdg.nextTreeShared());
      ++i;
    }
    REQUIRE(i == m);
  }

  // For-each loop test
  for (j = 0; j < 3; ++j) {
    i = 0;
    for (auto gkdt : kdg) {
      REQUIRE(trees[i] == gkdt);
      ++i;
    }
    REQUIRE(i == m);
  }
}

TEST_CASE("GroveTest: Observer pattern mechanics")
{
  GroveTestFixture fixture;
  auto& kdg = fixture.kdg;

  auto tree = kdg.getTreeShared(0);
  DynMovingObject dmo1("dmo1");
  DynMovingObject dmo2("dmo2");

  // Add dmo1, dmo2 and validate
  kdg.addSubject(&dmo1, tree);
  kdg.addSubject(&dmo2, tree);
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[4])->getId() ==
          "dmo1");
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[5])->getId() ==
          "dmo2");

  // Remove dmo1 and validate
  kdg.removeSubject(&dmo1);
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[4])->getId() ==
          "dmo2");

  // Remove dmo2 and validate
  kdg.removeSubject(&dmo2);
  REQUIRE(kdg.getSubjects().size() <= 4);
}
