#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include <DetailedVoxelLoader.h>
#include <HeliosException.h>
#include <VoxelFileParser.h>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <logger_core.hpp>
#include <string>

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

TEST_CASE("Detailed voxels: discard voxels without plant area in transmittive "
          "mode")
{
  // Default mode is transmittive
  REQUIRE(loadVoxels(testVoxFile, {})->mPrimitives.size() == 20);
  // Explicit mode, as passed by the Python API
  REQUIRE(loadVoxels(testVoxFile,
                     { { "intersectionMode", std::string("transmittive") } })
            ->mPrimitives.size() == 20);
  // Other modes keep all voxels
  REQUIRE(
    loadVoxels(testVoxFile, { { "intersectionMode", std::string("fixed") } })
      ->mPrimitives.size() == 36);
}
