#include <catch2/catch_test_macros.hpp>

#include <helios/noise/RandomnessGenerator.h>

TEST_CASE("Randomness generation test")
{
  const double eps = 0.0001;
  double diff;

  SECTION("Uniform real distribution with long seed")
  {
    double expectedURD1[] = {
      0.417022, 0.997185, 0.720324, 0.932557, 0.000114
    };
    RandomnessGenerator<double> rg1(1L);
    rg1.computeUniformRealDistribution(0.0, 1.0);
    for (size_t i = 0; i < 5; i++) {
      diff = rg1.uniformRealDistributionNext() - expectedURD1[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Uniform real distribution with double seed")
  {
    double expectedURD2[] = {
      -0.12801, -0.629836, -0.948148, 0.863082, 0.099325
    };
    RandomnessGenerator<double> rg2(2.5);
    rg2.computeUniformRealDistribution(-1.0, 1.0);
    for (size_t i = 0; i < 5; i++) {
      diff = rg2.uniformRealDistributionNext() - expectedURD2[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Uniform real distribution with automatic seed")
  {
    RandomnessGenerator<double> rg3;
    diff = rg3.uniformRealDistributionNext();
    for (size_t i = 0; i < 4; i++) {
      // This comparison may lead to false fails, but not too often
      REQUIRE(rg3.uniformRealDistributionNext() != diff);
    }
  }

  SECTION("Uniform real distribution with parsed long seed")
  {
    double expectedURD4[] = { 0.045783, 0.834397, 0.586121, 0.529073, 0.20324 };
    RandomnessGenerator<double> rg4("256");
    for (size_t i = 0; i < 5; i++) {
      diff = rg4.uniformRealDistributionNext() - expectedURD4[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Uniform real distribution with parsed double seed")
  {
    double expectedURD5[] = { 7.63083, 22.7339, 77.9919, 31.8972, 43.8409 };
    RandomnessGenerator<double> rg5("7.9");
    rg5.computeUniformRealDistribution(0.0, 100.0);
    for (size_t i = 0; i < 5; i++) {
      diff = rg5.uniformRealDistributionNext() - expectedURD5[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Uniform real distribution with parsed string timestamp seed")
  {
    double expectedURD6[] = {
      -0.451698, 19.3035, -68.2896, -78.9788, -91.9135
    };
    RandomnessGenerator<double> rg6("2000-08-22 11:30:27");
    rg6.computeUniformRealDistribution(-100.0, 50.0);
    for (size_t i = 0; i < 5; i++) {
      diff = rg6.uniformRealDistributionNext() - expectedURD6[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Normal distribution")
  {
    double expectedND1[] = { 0.670678, -1.25429, 0.440081, 0.347027, -1.43913 };
    RandomnessGenerator<double> rg7(1337);
    for (size_t i = 0; i < 5; i++) {
      diff = rg7.normalDistributionNext() - expectedND1[i];
      REQUIRE(diff >= -eps);
      REQUIRE(diff <= eps);
    }
  }

  SECTION("Swap, copy/move constructor and assignment")
  {
    double expectedURD8[] = { 0.803428, 0.150989, 0.527522,
                              0.995869, 0.119111, 0.214838 };
    RandomnessGenerator<double> rg8(999);
    diff = rg8.uniformRealDistributionNext() - expectedURD8[0];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
    RandomnessGenerator<double> rg8c(rg8);
    diff = rg8c.uniformRealDistributionNext() - expectedURD8[1];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
    rg8 = rg8c;
    diff = rg8.uniformRealDistributionNext() - expectedURD8[2];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
    rg8c = std::move(rg8);
    diff = rg8c.uniformRealDistributionNext() - expectedURD8[3];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
    RandomnessGenerator<double> rg8m(std::move(rg8c));
    diff = rg8m.uniformRealDistributionNext() - expectedURD8[4];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
    std::swap(rg8, rg8m);
    diff = rg8.uniformRealDistributionNext() - expectedURD8[5];
    REQUIRE(diff >= -eps);
    REQUIRE(diff <= eps);
  }

  SECTION("DEFAULT_RG behavior")
  {
    RandomnessGenerator<double> drgCopy1(*DEFAULT_RG);
    RandomnessGenerator<double> drgCopy2(*DEFAULT_RG);
    for (size_t i = 0; i < 5; i++) {
      // It could lead to false fails, but not too often
      REQUIRE(drgCopy1.uniformRealDistributionNext() !=
              drgCopy2.uniformRealDistributionNext());
      REQUIRE(drgCopy1.normalDistributionNext() !=
              drgCopy2.normalDistributionNext());
    }
    DEFAULT_RG->computeUniformRealDistribution(-1.0, 1.0);
    DEFAULT_RG->computeNormalDistribution(4.0, 2.0);
    RandomnessGenerator<double> drgCopy3(*DEFAULT_RG);
    RandomnessGenerator<double> drgCopy4(*DEFAULT_RG);
    for (size_t i = 0; i < 5; i++) {
      REQUIRE(drgCopy3.uniformRealDistributionNext() ==
              drgCopy4.uniformRealDistributionNext());
      REQUIRE(drgCopy3.normalDistributionNext() ==
              drgCopy4.normalDistributionNext());
    }
  }
}
