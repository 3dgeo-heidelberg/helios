#include <catch2/catch_test_macros.hpp>

#include <helios/assetloading/LadLutLoader.h>
#include <helios/maths/LadLut.h>
#include <string>

constexpr double eps = 0.00001;
const std::string testDir = "data/test/";

bool
validateLadLut(const LadLut& ladlut)
{
  // Validation data
  size_t indices[4] = { 0, 1, 498, 499 };
  double expectedX[4] = { 1.000000, 0.999921, 0.999921, 1.000000 };
  double expectedY[4] = { 0.000000, 0.000000, 0.000000, 0.000000 };
  double expectedZ[4] = { 0.000000, 0.012591, -0.012591, -0.000000 };
  double expectedG[4] = { 0.500000, 0.500040, 0.500040, 0.500000 };

  for (size_t i = 0; i < 4; i++) {
    size_t idx = indices[i];
    double diffX = ladlut.X[idx] - expectedX[i];
    double diffY = ladlut.Y[idx] - expectedY[i];
    double diffZ = ladlut.Z[idx] - expectedZ[i];
    double diffG = ladlut.G[idx] - expectedG[i];
    if (diffX < -eps || diffX > eps)
      return false;
    if (diffY < -eps || diffY > eps)
      return false;
    if (diffZ < -eps || diffZ > eps)
      return false;
    if (diffG < -eps || diffG > eps)
      return false;
  }
  return true;
}

bool
validateTransformation(double x,
                       double y,
                       double z,
                       double ex,
                       double ey,
                       double ez)
{
  double xDiff = x - ex;
  double yDiff = y - ey;
  double zDiff = z - ez;
  return xDiff > -eps && xDiff < eps && yDiff > -eps && yDiff < eps &&
         zDiff > -eps && zDiff < eps;
}

TEST_CASE("LadLut: Look-up table and transformation tests")
{
  // Input / expected output
  double u1[3] = { 0.79259392, 0.22645541, 0.56613852 };
  double g1 = 0.582513;
  double u2[3] = { -0.440759, 0.000000, 0.897626 };
  double g2 = 0.720515;
  double u3[3] = { -0.36999991, 0.46129036, -0.80641879 };
  double g3 = 0.673911;

  // Load spherical
  std::string llPath = testDir + "spherical.txt";
  LadLutLoader loader;
  std::shared_ptr<LadLut> ladlut = loader.load(llPath);
  REQUIRE(ladlut != nullptr);
  REQUIRE(validateLadLut(*ladlut));

  // Transform to LadLut domain
  double wx, wy, wz;
  ladlut->transformToLadLutDomain(u1[0], u1[1], u1[2], wx, wy, wz);
  REQUIRE(validateTransformation(wx, wy, wz, 0.824310, 0.0, 0.566139));

  // Interpolate 1
  double g = ladlut->interpolate(u1[0], u1[1], u1[2]);
  REQUIRE(std::fabs(g - g1) <= eps);

  // Interpolate w
  g = ladlut->interpolate(wx, wy, wz);
  REQUIRE(std::fabs(g - g1) <= eps);

  // Interpolate 2
  g = ladlut->interpolate(u2[0], u2[1], u2[2]);
  REQUIRE(std::fabs(g - g2) <= eps);

  // Interpolate 3
  g = ladlut->interpolate(u3[0], u3[1], u3[2]);
  REQUIRE(std::fabs(g - g3) <= eps);
}
