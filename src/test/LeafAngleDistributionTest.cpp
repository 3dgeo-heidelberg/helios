#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <HeliosException.h>
#include <LeafAngleDistribution.h>
#include <MathConstants.h>
#include <noise/UniformNoiseSource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using Catch::Approx;

namespace {

double
deg2rad(double deg)
{
  return deg * PI / 180.0;
}

glm::dvec3
directionFromZenith(double zenith_rad, double azimuth_rad = 0.0)
{
  return { std::sin(zenith_rad) * std::cos(azimuth_rad),
           std::sin(zenith_rad) * std::sin(azimuth_rad),
           std::cos(zenith_rad) };
}

/**
 * Uniform noise source that counts how many values were drawn
 */
class CountingNoiseSource : public NoiseSource<double>
{
public:
  explicit CountingNoiseSource(std::string const& seed)
    : inner(seed, 0.0, 1.0)
  {
  }
  std::size_t count = 0;

protected:
  double noiseFunction() override
  {
    ++count;
    return inner.next();
  }

private:
  UniformNoiseSource<double> inner;
};

/**
 * Unique path in the temporary directory, removed on destruction (also when
 * an assertion fails)
 */
struct TempFilePath
{
  std::filesystem::path path;

  explicit TempFilePath(std::string const& name)
  {
    auto const nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           (name + "_" + std::to_string(nonce) + ".txt");
  }

  ~TempFilePath()
  {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
};

struct Reference
{
  std::string name;
  double G[4];    // at zenith 0, 30, 60, 90 degrees
  double Ecos[4]; // mean cosine of incidence angle of hit leaves
};

// Computed independently with scipy.integrate.quad
std::vector<Reference> const references{
  { "planophile",
    { 0.848826, 0.738098, 0.472882, 0.270190 },
    { 0.883573, 0.804433, 0.594757, 0.462638 } },
  { "erectophile",
    { 0.424413, 0.451382, 0.508763, 0.540380 },
    { 0.589049, 0.623086, 0.675659, 0.693957 } },
  { "plagiophile",
    { 0.679061, 0.599002, 0.472453, 0.432304 },
    { 0.736311, 0.730382, 0.661441, 0.578297 } },
  { "extremophile",
    { 0.594178, 0.590478, 0.509192, 0.378266 },
    { 0.841498, 0.740925, 0.613718, 0.660911 } },
  { "spherical",
    { 0.500000, 0.500000, 0.500000, 0.500000 },
    { 0.666667, 0.666667, 0.666667, 0.666667 } },
  { "uniform",
    { 0.636620, 0.594740, 0.490823, 0.405285 },
    { 0.785398, 0.735616, 0.636686, 0.616850 } },
};

double const zenithsDeg[4] = { 0.0, 30.0, 60.0, 90.0 };

} // namespace

TEST_CASE("LeafAngleDistribution: named types are normalized")
{
  double const w = PI_HALF / LeafAngleDistribution::INCLINATION_BINS;
  for (auto const& ref : references) {
    auto const lad = LeafAngleDistribution::fromName(ref.name);
    double sum = 0.0;
    for (double const g : lad.getDensity())
      sum += g * w;
    REQUIRE(sum == Approx(1.0).margin(1e-12));
  }
}

TEST_CASE("LeafAngleDistribution: G matches reference values")
{
  for (auto const& ref : references) {
    auto const lad = LeafAngleDistribution::fromName(ref.name);
    for (std::size_t k = 0; k < 4; ++k) {
      INFO(ref.name << " at zenith " << zenithsDeg[k]);
      double const zeta = deg2rad(zenithsDeg[k]);
      REQUIRE(lad.G(zeta) == Approx(ref.G[k]).margin(1e-5));
      REQUIRE(lad.G(directionFromZenith(zeta)) ==
              Approx(ref.G[k]).margin(1e-5));
    }
  }
}

TEST_CASE("LeafAngleDistribution: G symmetries")
{
  auto const lad = LeafAngleDistribution::fromName("planophile");
  for (double zDeg : { 5.0, 37.0, 71.0 }) {
    double const zeta = deg2rad(zDeg);
    double const g = lad.G(zeta);
    // Up and down are equivalent
    REQUIRE(lad.G(PI - zeta) == Approx(g).margin(1e-12));
    REQUIRE(lad.G(-directionFromZenith(zeta)) == Approx(g).margin(1e-12));
    // Beam azimuth does not matter
    for (double aDeg : { 0.0, 45.0, 123.0, 270.0 })
      REQUIRE(lad.G(directionFromZenith(zeta, deg2rad(aDeg))) ==
              Approx(g).margin(1e-12));
    // Direction does not need to be normalized
    REQUIRE(lad.G(3.5 * directionFromZenith(zeta)) == Approx(g).margin(1e-12));
  }
  REQUIRE_THROWS_AS(lad.G(glm::dvec3(0.0)), HeliosException);
}

TEST_CASE("LeafAngleDistribution: psi")
{
  // Vertical beam: psi = cos(thetaL)
  REQUIRE(LeafAngleDistribution::psi(0.0, deg2rad(40.0)) ==
          Approx(std::cos(deg2rad(40.0))));
  // Horizontal leaves: psi = |cos(zeta)|
  REQUIRE(LeafAngleDistribution::psi(deg2rad(60.0), 0.0) == Approx(0.5));
  // Horizontal beam, vertical leaves: mean |cos(phi)| = 2/pi
  REQUIRE(LeafAngleDistribution::psi(PI_HALF, PI_HALF) == Approx(2.0 / PI));
}

TEST_CASE("LeafAngleDistribution: names")
{
  REQUIRE(LeafAngleDistribution::fromName("Spherical").getName() ==
          "spherical");
  REQUIRE(LeafAngleDistribution::isNamedType("PLANOPHILE"));
  REQUIRE_FALSE(LeafAngleDistribution::isNamedType("lut/spherical.txt"));
  REQUIRE_THROWS_AS(LeafAngleDistribution::fromName("conical"),
                    HeliosException);
}

TEST_CASE("LeafAngleDistribution: tabulated input")
{
  // Planophile tabulated at 1 degree steps, unnormalized
  std::vector<double> t, g;
  for (int deg = 0; deg <= 90; ++deg) {
    t.push_back(deg2rad(deg));
    g.push_back(5.0 * (1.0 + std::cos(2.0 * deg2rad(deg))));
  }
  LeafAngleDistribution const lad(t, g);
  auto const& ref = references[0];
  for (std::size_t k = 0; k < 4; ++k)
    REQUIRE(lad.G(deg2rad(zenithsDeg[k])) == Approx(ref.G[k]).margin(1e-4));

  SECTION("Invalid input")
  {
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.0 }, { 1.0 }), HeliosException);
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.0, 1.0 }, { 1.0 }),
                      HeliosException);
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.0, 2.0 }, { 1.0, 1.0 }),
                      HeliosException);
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.5, 0.2 }, { 1.0, 1.0 }),
                      HeliosException);
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.0, 1.0 }, { 1.0, -1.0 }),
                      HeliosException);
    REQUIRE_THROWS_AS(LeafAngleDistribution({ 0.0, 1.0 }, { 0.0, 0.0 }),
                      HeliosException);
  }
}

TEST_CASE("LeafAngleDistribution: file input")
{
  TempFilePath const tmp("helios_lad_test");
  auto const& path = tmp.path;
  {
    std::ofstream ofs(path);
    ofs << "# inclination_deg,g_L\n\n";
    for (int deg = 0; deg <= 90; ++deg)
      ofs << deg << "," << (1.0 - std::cos(2.0 * deg2rad(deg))) << "\n";
  }
  auto const lad = LeafAngleDistribution::fromFile(path.string());
  auto const& ref = references[1];
  for (std::size_t k = 0; k < 4; ++k)
    REQUIRE(lad.G(deg2rad(zenithsDeg[k])) == Approx(ref.G[k]).margin(1e-4));

  {
    std::ofstream ofs(path);
    ofs << "0,1\n45;1\n";
  }
  REQUIRE_THROWS_AS(LeafAngleDistribution::fromFile(path.string()),
                    HeliosException);

  REQUIRE_THROWS_AS(LeafAngleDistribution::fromFile("does/not/exist/lad.txt"),
                    HeliosException);
}

TEST_CASE("LeafAngleDistribution: incidence angle sampling")
{
  std::size_t const n = 200000;
  for (auto const& ref : references) {
    auto const lad = LeafAngleDistribution::fromName(ref.name);
    for (std::size_t k = 0; k < 4; ++k) {
      INFO(ref.name << " at zenith " << zenithsDeg[k]);
      glm::dvec3 const dir =
        directionFromZenith(deg2rad(zenithsDeg[k]), deg2rad(17.0));
      CountingNoiseSource noise("42");
      double sumCos = 0.0;
      double minAlpha = PI, maxAlpha = -PI;
      for (std::size_t s = 0; s < n; ++s) {
        double const alpha = lad.sampleIncidenceAngle(dir, noise);
        minAlpha = std::min(minAlpha, alpha);
        maxAlpha = std::max(maxAlpha, alpha);
        sumCos += std::cos(alpha);
      }
      REQUIRE(minAlpha >= 0.0);
      REQUIRE(maxAlpha <= PI_HALF);
      // Three noise values per attempt; acceptance rate converges to G
      double const attempts = static_cast<double>(noise.count) / 3.0;
      REQUIRE(static_cast<double>(n) / attempts ==
              Approx(ref.G[k]).margin(0.005));
      REQUIRE(sumCos / static_cast<double>(n) ==
              Approx(ref.Ecos[k]).margin(0.005));
    }
  }
}

TEST_CASE("LeafAngleDistribution: example file (ellipsoidal, x = 2.7)")
{
  // Shipped example of the custom file format. Reference G computed with
  // scipy.integrate.quad from the analytic ellipsoidal distribution.
  auto const lad =
    LeafAngleDistribution::fromFile("data/lad/ellipsoidal_tomato.txt");
  double const G[4] = { 0.804056, 0.712075, 0.477639, 0.297799 };
  for (std::size_t k = 0; k < 4; ++k) {
    INFO("zenith " << zenithsDeg[k]);
    REQUIRE(lad.G(deg2rad(zenithsDeg[k])) == Approx(G[k]).margin(2e-4));
  }
}
