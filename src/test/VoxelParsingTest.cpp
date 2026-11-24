#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include <DetailedVoxelLoader.h>
#include <HeliosException.h>
#include <VoxelFileParser.h>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <logger_core.hpp>
#include <MathConstants.h>
#include <noise/UniformNoiseSource.h>
#include <string>
#include <vector>

TEST_CASE("Voxel Parsing Test ")
{
  std::string testDir("data/test/");

  double eps = 0.0001;
  std::string vfPath = testDir + "semitransparent_voxels.vox";
  if (!std::filesystem::exists(vfPath)) {
    FAIL("Voxel file not found: " + vfPath);
  }
  VoxelFileParser vfp;
  std::vector<std::shared_ptr<DetailedVoxel>> dvoxels =
    vfp.parseDetailed(vfPath, 2, false);
  REQUIRE(dvoxels.size() == 36);

  std::shared_ptr<DetailedVoxel> dv;
  double xDiff, yDiff, zDiff;

  SECTION("Expected int values")
  {
    // Expected int values
    int EXPECTED_nbEchos[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int EXPECTED_nbSampling[36] = {
      49,  120, 169, 180, 153, 139, 302, 216, 116, 38, 35,  153,
      168, 124, 253, 219, 69,  69,  69,  216, 116, 38, 35,  153,
      168, 124, 253, 219, 69,  50,  16,  8,   38,  54, 174, 48
    };
    // Test expected int values
    for (size_t i = 0; i < dvoxels.size(); i++) {
      dv = dvoxels[i];
      REQUIRE(dv->getNbEchos() == EXPECTED_nbEchos[i]);
      REQUIRE(dv->getNbSampling() == EXPECTED_nbSampling[i]);
    }
  }

  SECTION("Expected double values")
  {

    // Expected double values
    double EXPECTED_PadBVTotal[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    double EXPECTED_angleMean[36] = {
      89.7859174, 89.9096456, 89.6270244, 89.3006538, 88.9270184, 88.6899252,
      88.3412947, 88.0299087, 87.6877344, 87.3695616, 87.120519,  86.7490226,
      86.382687,  86.0816272, 85.7616811, 85.4599773, 85.1357778, 85.1357778,
      85.1357778, 88.0299087, 87.6877344, 87.3695616, 87.120519,  86.7490226,
      86.382687,  86.0816272, 85.7616811, 85.4599773, 85.1357778, 84.7967107,
      84.606863,  84.2140568, 83.8143775, 83.5161812, 83.234543,  82.9336984
    };
    double EXPECTED_bsPotential[36] = {
      0.1843913, 0.1772976, 0.1854843, 0.1794923, 0.186054,  0.180072,
      0.1877376, 0.1861319, 0.1861689, 0.1845732, 0.1884445, 0.1901379,
      0.1885587, 0.1875307, 0.1897892, 0.1915078, 0.1910439, 0.1910439,
      0.1910439, 0.1861319, 0.1861689, 0.1845732, 0.1884445, 0.1901379,
      0.1885587, 0.1875307, 0.1897892, 0.1915078, 0.1910439, 0.1889422,
      0.1939671, 0.1940688, 0.1952733, 0.1931918, 0.1982526, 0.1972807
    };
    double EXPECTED_x[36] = { 5.0,  5.0,  5.0,  5.0,  5.0,  5.0,  5.0,  5.0,
                              5.0,  5.0,  5.0,  5.0,  15.0, 15.0, 15.0, 15.0,
                              15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0,
                              25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0,
                              25.0, 25.0, 25.0, 25.0 };
    double EXPECTED_y[36] = { 5.0,  5.0,  5.0,  5.0,  15.0, 15.0, 15.0, 15.0,
                              25.0, 25.0, 25.0, 25.0, 5.0,  5.0,  5.0,  5.0,
                              15.0, 15.0, 15.0, 15.0, 25.0, 25.0, 25.0, 25.0,
                              5.0,  5.0,  5.0,  5.0,  15.0, 15.0, 15.0, 15.0,
                              25.0, 25.0, 25.0, 25.0 };
    double EXPECTED_z[36] = { 5.0, 15.0, 25.0, 35.0, 5.0, 15.0, 25.0, 35.0,
                              5.0, 15.0, 25.0, 35.0, 5.0, 15.0, 25.0, 35.0,
                              5.0, 15.0, 25.0, 35.0, 5.0, 15.0, 25.0, 35.0,
                              5.0, 15.0, 25.0, 35.0, 5.0, 15.0, 25.0, 35.0,
                              5.0, 15.0, 25.0, 35.0 };

    // Test expected double values
    for (size_t i = 0; i < dvoxels.size(); i++) {
      dv = dvoxels[i];
      REQUIRE((*dv)[0] == EXPECTED_PadBVTotal[i]);
      REQUIRE((*dv)[1] == EXPECTED_angleMean[i]);
      REQUIRE((*dv)["bsPotential"] == EXPECTED_bsPotential[i]);
      xDiff = dv->getCentroid().x - EXPECTED_x[i];
      REQUIRE((xDiff >= -eps && xDiff <= eps));
      yDiff = dv->getCentroid().y - EXPECTED_y[i];
      REQUIRE((yDiff >= -eps && yDiff <= eps));
      zDiff = dv->getCentroid().z - EXPECTED_z[i];
      REQUIRE((zDiff >= -eps && zDiff <= eps));
      REQUIRE(dv->halfSize == 5.0);
    }
  }
}

namespace {

// 36 voxels: 20 with PadBVTotal > 0, 15 with 0 and 1 with NaN
std::string const testVoxFile = "data/test/transmittive_voxels.vox";

std::shared_ptr<ScenePart>
loadVoxels(std::string const& path,
           std::map<std::string, ObjectT> params)
{
  DetailedVoxelLoader loader;
  loader.params["filepath"] = path;
  loader.setAssetsDir({ "." });
  for (auto const& [key, value] : params)
    loader.params[key] = value;
  std::shared_ptr<ScenePart> sp(loader.run());
  for (auto p : sp->mPrimitives)
    p->part = sp;
  loader.primsOut = nullptr;
  return sp;
}

std::shared_ptr<ScenePart>
loadTransmittiveVoxels(std::string const& path,
                       std::map<std::string, ObjectT> extraParams = {})
{
  extraParams["intersectionMode"] = std::string("TRANSMITTIVE");
  return loadVoxels(path, extraParams);
}

/**
 * Copy of the test voxel file with "#lad_type:Planophile" replaced, removed
 * on destruction (also when an assertion fails)
 */
struct TempVoxFile
{
  std::filesystem::path path;

  explicit TempVoxFile(std::string const& ladTypeReplacement)
  {
    auto const nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           ("helios_vox_test_" + std::to_string(nonce) + ".vox");
    std::ifstream ifs(testVoxFile);
    std::ofstream ofs(path);
    std::string const key = "#lad_type:Planophile";
    std::string line;
    while (std::getline(ifs, line)) {
      std::size_t const pos = line.find(key);
      if (pos != std::string::npos)
        line.replace(pos, key.size(), ladTypeReplacement);
      ofs << line << "\n";
    }
  }

  ~TempVoxFile()
  {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
};

/**
 * Check kappa = PAD * G for all voxels
 */
void
checkExtinction(ScenePart const& sp, double gDown, double gHorizontal)
{
  glm::dvec3 const down(0.0, 0.0, -1.0);
  glm::dvec3 const horizontal(1.0, 0.0, 0.0);
  REQUIRE_FALSE(sp.mPrimitives.empty());
  for (auto p : sp.mPrimitives) {
    auto* dv = dynamic_cast<DetailedVoxel*>(p);
    REQUIRE(dv != nullptr);
    double const pad = (*dv)["PadBVTotal"];
    REQUIRE(pad > 0.0);
    REQUIRE(dv->computeExtinctionCoefficient(down) ==
            Catch::Approx(gDown * pad).margin(1e-5));
    REQUIRE(dv->computeExtinctionCoefficient(horizontal) ==
            Catch::Approx(gHorizontal * pad).margin(1e-5));
  }
}

} // namespace

TEST_CASE("Detailed voxels: extinction coefficient from leaf angle distribution")
{
  SECTION("Default from file header (#lad_type)")
  {
    auto const sp = loadTransmittiveVoxels(testVoxFile);
    REQUIRE(sp->lad != nullptr);
    REQUIRE(sp->lad->getName() == "planophile");
    checkExtinction(*sp, 0.848826, 0.270190);
  }

  SECTION("Explicit distribution overrides the file header")
  {
    auto const sp = loadTransmittiveVoxels(
      testVoxFile, { { "lad", std::string("spherical") } });
    REQUIRE(sp->lad != nullptr);
    REQUIRE(sp->lad->getName() == "spherical");
    checkExtinction(*sp, 0.5, 0.5);
  }

  SECTION("No distribution in the file header: spherical")
  {
    TempVoxFile const vox("");
    auto const sp = loadTransmittiveVoxels(vox.path.string());
    REQUIRE(sp->lad == nullptr);
    checkExtinction(*sp, 0.5, 0.5);
  }

  SECTION("Unsupported distribution in the file header: spherical")
  {
    TempVoxFile const vox("#lad_type:Ellipsoidal");
    auto const sp = loadTransmittiveVoxels(vox.path.string());
    REQUIRE(sp->lad == nullptr);
    checkExtinction(*sp, 0.5, 0.5);
  }

  SECTION("Removed ladlut parameter")
  {
    REQUIRE_THROWS_AS(
      loadTransmittiveVoxels(testVoxFile,
                             { { "ladlut", std::string("lut/spherical.txt") } }),
      HeliosException);
  }
}

TEST_CASE("Detailed voxels: discard voxels without plant area")
{
  // Default mode is transmittive
  REQUIRE(loadVoxels(testVoxFile, {})->mPrimitives.size() == 20);
  // Explicit mode, as passed by the Python API
  REQUIRE(loadVoxels(testVoxFile,
                     { { "intersectionMode", std::string("transmittive") } })
            ->mPrimitives.size() == 20);
  // Scaled mode discards PAD = NaN only (size cannot be derived)
  REQUIRE(
    loadVoxels(testVoxFile, { { "intersectionMode", std::string("scaled") } })
      ->mPrimitives.size() == 35);
  // Fixed mode keeps all voxels
  REQUIRE(
    loadVoxels(testVoxFile, { { "intersectionMode", std::string("fixed") } })
      ->mPrimitives.size() == 36);
}

TEST_CASE("Detailed voxels: leaf angle distribution only in transmittive mode")
{
  SECTION("Transmittive modes")
  {
    REQUIRE(DetailedVoxel::isTransmittiveMode(""));
    REQUIRE(DetailedVoxel::isTransmittiveMode("TRANSMITTIVE"));
    REQUIRE_FALSE(DetailedVoxel::isTransmittiveMode("FIXED"));
    REQUIRE_FALSE(DetailedVoxel::isTransmittiveMode("SCALED"));
  }

  SECTION("Explicit distribution and header are ignored in other modes")
  {
    for (std::string const mode : { "fixed", "scaled" }) {
      auto const sp = loadVoxels(
        testVoxFile,
        { { "intersectionMode", mode }, { "lad", std::string("erectophile") } });
      REQUIRE(sp->lad == nullptr);
      auto const spHeader =
        loadVoxels(testVoxFile, { { "intersectionMode", mode } });
      REQUIRE(spHeader->lad == nullptr);
    }
  }

  SECTION("Voxels in other modes are not transmittive")
  {
    auto const sp = loadVoxels(testVoxFile,
                               { { "intersectionMode", std::string("fixed") } });
    // Distribution assigned afterwards, e.g. by a scene part split
    sp->lad = std::make_shared<LeafAngleDistribution const>(
      LeafAngleDistribution::fromName("spherical"));
    REQUIRE_FALSE(sp->mPrimitives.empty());
    for (auto p : sp->mPrimitives) {
      auto* dv = dynamic_cast<DetailedVoxel*>(p);
      REQUIRE(dv != nullptr);
      REQUIRE_FALSE(dv->isTransmittive());
    }

    auto const spT = loadTransmittiveVoxels(testVoxFile);
    for (auto p : spT->mPrimitives) {
      auto* dv = dynamic_cast<DetailedVoxel*>(p);
      REQUIRE(dv->isTransmittive());
    }
  }
}

TEST_CASE("Detailed voxels: hit point and incidence angle in transmittive mode")
{
  auto const sp = loadTransmittiveVoxels(testVoxFile);
  REQUIRE_FALSE(sp->mPrimitives.empty());
  auto* dv = dynamic_cast<DetailedVoxel*>(sp->mPrimitives[0]);
  REQUIRE(dv != nullptr);

  // Vertical ray through the voxel centre
  glm::dvec3 const c = dv->getCentroid();
  glm::dvec3 const inside(c.x, c.y, c.z + dv->halfSize);
  glm::dvec3 const outside(c.x, c.y, c.z - dv->halfSize);
  glm::dvec3 dir(0.0, 0.0, -1.0);
  UniformNoiseSource<double> noise("7", 0.0, 1.0);

  SECTION("Dense voxel: hit inside with sampled incidence angle")
  {
    (*dv)["PadBVTotal"] = 1e6;
    std::size_t const n = 20000;
    double sumCos = 0.0;
    std::size_t notHit = 0, noAngle = 0, badAngle = 0, outsideVoxel = 0;
    for (std::size_t i = 0; i < n; ++i) {
      IntersectionHandlingResult ihr =
        dv->onRayIntersection(noise, dir, inside, outside);
      notHit += ihr.canRayContinue();
      noAngle += !ihr.hasIncidenceAngle();
      double const alpha = ihr.getIncidenceAngle_rad();
      badAngle += !(alpha >= 0.0 && alpha <= PI_HALF);
      glm::dvec3 const p = ihr.getIntersectionPoint();
      outsideVoxel += !(p.z <= inside.z && p.z >= outside.z);
      sumCos += std::cos(alpha);
    }
    REQUIRE(notHit == 0);
    REQUIRE(noAngle == 0);
    REQUIRE(badAngle == 0);
    REQUIRE(outsideVoxel == 0);
    // Planophile from the file header, vertical beam: E[cos(alpha)]
    REQUIRE(sumCos / static_cast<double>(n) ==
            Catch::Approx(0.883573).margin(0.01));
  }

  SECTION("Pass through: no incidence angle")
  {
    (*dv)["PadBVTotal"] = 1e-12;
    IntersectionHandlingResult ihr =
      dv->onRayIntersection(noise, dir, inside, outside);
    REQUIRE(ihr.canRayContinue());
    REQUIRE_FALSE(ihr.hasIncidenceAngle());
  }
}

TEST_CASE("Detailed voxels: default material")
{
  // Leaves of a turbid medium are Lambertian reflectors: the default
  // material is purely diffuse (no specular part) and direction dependent
  auto const spT = loadTransmittiveVoxels(testVoxFile);
  REQUIRE_FALSE(spT->mPrimitives.empty());
  for (auto p : spT->mPrimitives) {
    REQUIRE(p->material->ks == 0.0);
    REQUIRE_FALSE(p->material->isDirectionIndependent);
  }

  // Opaque voxels use the same default material
  auto const spF =
    loadVoxels(testVoxFile, { { "intersectionMode", std::string("fixed") } });
  REQUIRE_FALSE(spF->mPrimitives.empty());
  for (auto p : spF->mPrimitives) {
    REQUIRE(p->material->ks == 0.0);
    REQUIRE_FALSE(p->material->isDirectionIndependent);
  }
}

TEST_CASE("Detailed voxels: transmittive mode follows Beer-Lambert")
{
  auto const sp = loadTransmittiveVoxels(testVoxFile);
  REQUIRE(sp->lad != nullptr); // Planophile from the file header
  auto* dv = dynamic_cast<DetailedVoxel*>(sp->mPrimitives[0]);
  REQUIRE(dv != nullptr);
  double const pad = 0.15;
  (*dv)["PadBVTotal"] = pad;
  glm::dvec3 const c = dv->getCentroid();
  double const h = dv->halfSize;
  double const L = 2.0 * h;
  std::size_t const n = 40000;

  // Vertical and horizontal ray through the voxel centre. Planophile G:
  // 0.848826 (vertical), 0.270190 (horizontal)
  struct Case
  {
    glm::dvec3 dir, inside, outside;
    double G;
  };
  std::vector<Case> const cases{
    { { 0.0, 0.0, -1.0 }, { c.x, c.y, c.z + h }, { c.x, c.y, c.z - h }, 0.848826 },
    { { 1.0, 0.0, 0.0 }, { c.x - h, c.y, c.z }, { c.x + h, c.y, c.z }, 0.270190 },
  };
  for (auto const& tc : cases) {
    double const kappa = pad * tc.G;
    double const pPass = std::exp(-kappa * L);
    // Mean depth of a hit, given a hit within [0, L]
    double const meanDepth = 1.0 / kappa - L * pPass / (1.0 - pPass);

    UniformNoiseSource<double> noise("11", 0.0, 1.0);
    glm::dvec3 dir = tc.dir;
    std::size_t passed = 0;
    double sumDepth = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      IntersectionHandlingResult ihr =
        dv->onRayIntersection(noise, dir, tc.inside, tc.outside);
      if (ihr.canRayContinue())
        ++passed;
      else
        sumDepth += glm::distance(ihr.getIntersectionPoint(), tc.inside);
    }
    double const fPass = static_cast<double>(passed) / static_cast<double>(n);
    // Binomial standard deviation of fPass is below 0.0025 here
    REQUIRE(fPass == Catch::Approx(pPass).margin(0.01));
    REQUIRE(sumDepth / static_cast<double>(n - passed) ==
            Catch::Approx(meanDepth).margin(0.1));
  }

  SECTION("Same seed, same result")
  {
    UniformNoiseSource<double> a("5", 0.0, 1.0), b("5", 0.0, 1.0);
    glm::dvec3 dir = cases[0].dir;
    for (std::size_t i = 0; i < 1000; ++i) {
      IntersectionHandlingResult ra =
        dv->onRayIntersection(a, dir, cases[0].inside, cases[0].outside);
      IntersectionHandlingResult rb =
        dv->onRayIntersection(b, dir, cases[0].inside, cases[0].outside);
      REQUIRE(ra.canRayContinue() == rb.canRayContinue());
      REQUIRE(ra.getIntersectionPoint() == rb.getIntersectionPoint());
      if (ra.hasIncidenceAngle())
        REQUIRE(ra.getIncidenceAngle_rad() == rb.getIncidenceAngle_rad());
    }
  }
}
