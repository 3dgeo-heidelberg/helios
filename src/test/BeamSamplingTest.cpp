#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <maths/Directions.h>
#include <scanner/BeamSampling.h>

#include <cmath>
#include <limits>
#include <stdexcept>

TEST_CASE("Cartesian sampling integrates rectangular Gaussian cells", "[beam]")
{
  int const quality = GENERATE(1, 3, 8);
  double const cutoff = GENERATE(0.1, 0.5, 2.0, 4.0);
  double const dx = GENERATE(0.0003, 0.2);
  double const dy = GENERATE(0.0003, 0.001, 0.2);
  BeamSamplingSettings const settings{ BeamSamplingMethod::Cartesian,
                                       quality,
                                       cutoff };
  auto const rays = makeSubrayPattern({ dx, dy }, settings);
  auto const swapped = makeSubrayPattern({ dy, dx }, settings);
  auto const repeated = makeSubrayPattern({ dx, dy }, settings);
  std::size_t const n = 2 * quality - 1;
  REQUIRE(rays.size() == n * n);
  double const tx = std::tan(dx / 2), ty = std::tan(dy / 2);
  double totalPower = 0, totalArea = 0;
  for (std::size_t j = 0; j < n; ++j) {
    for (std::size_t i = 0; i < n; ++i) {
      auto const& ray = rays[j * n + i];
      auto const& cell = std::get<CartesianCell>(ray.cell);
      double const u0 = -cutoff + 2 * cutoff * i / n;
      double const u1 = -cutoff + 2 * cutoff * (i + 1) / n;
      double const v0 = -cutoff + 2 * cutoff * j / n;
      double const v1 = -cutoff + 2 * cutoff * (j + 1) / n;
      REQUIRE(cell.uMin == Catch::Approx(u0).margin(1e-14));
      REQUIRE(cell.uMax == Catch::Approx(u1).margin(1e-14));
      REQUIRE(cell.vMin == Catch::Approx(v0).margin(1e-14));
      REQUIRE(cell.vMax == Catch::Approx(v1).margin(1e-14));
      if (i)
        REQUIRE(cell.uMin ==
                std::get<CartesianCell>(rays[j * n + i - 1].cell).uMax);
      if (j)
        REQUIRE(cell.vMin ==
                std::get<CartesianCell>(rays[(j - 1) * n + i].cell).vMax);
      auto rotation = ray.rotation;
      auto const direction = rotation.applyTo(Directions::forward);
      REQUIRE(glm::length(direction) == Catch::Approx(1.0));
      REQUIRE(direction.x / direction.y ==
              Catch::Approx(tx * (u0 + u1) / 2).margin(1e-14));
      REQUIRE(direction.z / direction.y ==
              Catch::Approx(ty * (v0 + v1) / 2).margin(1e-14));
      double const expected =
        0.25 * (std::erf(std::sqrt(2.) * u1) - std::erf(std::sqrt(2.) * u0)) *
        (std::erf(std::sqrt(2.) * v1) - std::erf(std::sqrt(2.) * v0));
      REQUIRE(ray.share == Catch::Approx(expected).margin(1e-15));
      REQUIRE(ray.share >= 0);
      REQUIRE(ray.areaFactor == Catch::Approx(tx * ty * (u1 - u0) * (v1 - v0)));
      REQUIRE(
        ray.share ==
        Catch::Approx(rays[(n - 1 - j) * n + n - 1 - i].share).margin(1e-15));
      REQUIRE(ray.share == swapped[i * n + j].share);
      REQUIRE(ray.areaFactor == Catch::Approx(swapped[i * n + j].areaFactor));
      REQUIRE(ray.angle_rad ==
              Catch::Approx(swapped[i * n + j].angle_rad).margin(1e-14));
      REQUIRE(ray.share == repeated[j * n + i].share);
      REQUIRE(ray.angle_rad == repeated[j * n + i].angle_rad);
      totalPower += ray.share;
      totalArea += ray.areaFactor;
    }
  }
  auto const& center = rays[rays.size() / 2];
  REQUIRE(center.angle_rad == 0);
  REQUIRE(std::get<CartesianCell>(center.cell).u == 0);
  REQUIRE(std::get<CartesianCell>(center.cell).v == 0);
  REQUIRE(totalPower ==
          Catch::Approx(std::pow(std::erf(std::sqrt(2.) * cutoff), 2))
            .epsilon(1e-12));
  REQUIRE(totalArea ==
          Catch::Approx(4 * cutoff * cutoff * tx * ty).epsilon(1e-12));
}

TEST_CASE("Cartesian validation rejects unrepresentable patterns", "[beam]")
{
  BeamSamplingSettings settings{ BeamSamplingMethod::Cartesian, 3, 2 };
  BeamGeometry beam{ 0.0003, 0.001 };
  SECTION("Count overflow")
  {
    settings.quality = std::numeric_limits<int>::max();
  }
  SECTION("Invalid quality")
  {
    settings.quality = GENERATE(0, -1);
  }
  SECTION("Invalid cutoff")
  {
    settings.cutoff = GENERATE(0.,
                               -1.,
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN());
  }
  SECTION("Invalid divergence")
  {
    beam.divergenceX_rad =
      GENERATE(0., -1., 4., std::numeric_limits<double>::quiet_NaN());
  }
  SECTION("Unrepresentable corners")
  {
    settings.cutoff = 1e20;
  }
  SECTION("Patch underflow")
  {
    settings.cutoff = 1e-200;
  }
  SECTION("Invalid method")
  {
    settings.method = static_cast<BeamSamplingMethod>(99);
  }
  REQUIRE_THROWS_AS(makeSubrayPattern(beam, settings), std::invalid_argument);
}

TEST_CASE("Cartesian tail integration avoids erf cancellation", "[beam]")
{
  auto const rays = makeSubrayPattern({ 0.0003, 0.001 },
                                      { BeamSamplingMethod::Cartesian, 3, 8 });
  // The first cell covers [-8,-4.8] on each axis: erf at both ends rounds to
  // -1.
  double const expected =
    0.5 * (std::erfc(std::sqrt(2.) * 4.8) - std::erfc(std::sqrt(2.) * 8));
  REQUIRE(rays.front().share > 0);
  REQUIRE(rays.front().share ==
          Catch::Approx(expected * expected).epsilon(1e-12));
  auto const wide = makeSubrayPattern(
    { 0.0003, 0.001 }, { BeamSamplingMethod::Cartesian, 3, 100 });
  REQUIRE(wide.front().share == 0);
}

TEST_CASE("Sampling method names round trip", "[beam]")
{
  for (auto method :
       { BeamSamplingMethod::Annular, BeamSamplingMethod::Cartesian })
    REQUIRE(parseBeamSamplingMethod(toString(method)) == method);
  REQUIRE_THROWS_AS(parseBeamSamplingMethod("Cartesian"),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(parseBeamSamplingMethod(""), std::invalid_argument);
}
