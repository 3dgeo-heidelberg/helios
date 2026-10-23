#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <limits>
#include <maths/EnergyModel.h>
#include <maths/WaveMaths.h>
#include <noise/UniformNoiseSource.h>
#include <scanner/MultiScanner.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>
#include <scene/primitives/Voxel.h>
#include <utility>

namespace {
std::shared_ptr<Scanner>
energyScanner(double amplitude = 4.0)
{
  auto scanner = std::make_shared<SingleScanner>(0.0003,
                                                 glm::dvec3(0),
                                                 Rotation(),
                                                 std::list<int>{ 100000 },
                                                 5.0,
                                                 "energy-test",
                                                 amplitude,
                                                 0.99,
                                                 0.15,
                                                 23.0,
                                                 1064,
                                                 false,
                                                 false,
                                                 false,
                                                 false,
                                                 false);
  scanner->setAtmosphericExtinction(0.0, 0);
  return scanner;
}

double
received(EnergyModel& model,
         double range,
         Material const& mat,
         std::size_t index)
{
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> records;
  return model.computeReceivedPower(1.0, range, mat, index, records);
#else
  return model.computeReceivedPower(1.0, range, mat, index);
#endif
}

double
area(EnergyModel& model, double rangeSquared, std::size_t index)
{
  return model.computeTargetArea(std::sqrt(rangeSquared), index);
}

double
ratio(EnergyModel const& model,
      double cosine,
      double range,
      Material const& material)
{
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> records;
  return model.computePowerRatio(cosine, range, material, 0, records);
#else
  return model.computePowerRatio(cosine, range, material, 0);
#endif
}

double
atmosphericTransmission(Scanner& scanner, double range)
{
  scanner.prepareSimulation();
  auto& device = scanner.getScanningDevice(0);
  Material material;
  material.reflectance = 1.0;
  double const attenuated =
    ratio(*device.getEnergyModel(), 1.0, range, material);
  double const extinction = scanner.getAtmosphericExtinction();
  scanner.setAtmosphericExtinction(0.0, 0);
  double const unattenuated =
    ratio(*device.getEnergyModel(), 1.0, range, material);
  scanner.setAtmosphericExtinction(extinction, 0);
  return attenuated / unattenuated;
}
}

TEST_CASE("Atmospheric extinction uses micrometres and inverse metres",
          "[energy]")
{
  // Reference values at 1064 nm cover all visibility branches and boundaries.
  auto const sample =
    GENERATE(std::make_pair(5.0, 0.00040556958615247865),
             std::make_pair(6.0, 0.0003244902684506537),
             std::make_pair(6.0001, 0.000276353932424009),
             std::make_pair(23.0, 0.0000720935317364042),
             std::make_pair(50.0, 0.00003316302459874593),
             std::make_pair(50.0001, 0.000027206932737294658));
  auto scanner = energyScanner();
  auto& device = scanner->getScanningDevice(0);
  scanner->setVisibility(sample.first);
  double const extinction = device.calcAtmosphericAttenuation();
  scanner->setAtmosphericExtinction(extinction, 0);
  REQUIRE(extinction == Catch::Approx(sample.second).epsilon(1e-12));
  if (sample.first == 23.0) {
    REQUIRE(atmosphericTransmission(*scanner, 1000.0) ==
            Catch::Approx(0.8657257872389399));
    REQUIRE(atmosphericTransmission(*scanner, 3000.0) ==
            Catch::Approx(0.6488451488135547));
  }
}

TEST_CASE("Kruse wavelength bounds include endpoints", "[energy]")
{
  double const wavelength =
    GENERATE(0.499e-6, 0.5e-6, 0.55e-6, 2.0e-6, 2.001e-6);
  auto scanner = energyScanner();
  auto& device = scanner->getScanningDevice(0);
  scanner->setWavelength(wavelength);
  double const extinction = device.calcAtmosphericAttenuation();
  scanner->setAtmosphericExtinction(extinction, 0);
  if (wavelength < 0.5e-6 || wavelength > 2.0e-6) {
    REQUIRE(extinction == 0.0);
    REQUIRE(atmosphericTransmission(*scanner, 1000.0) == 1.0);
  } else {
    REQUIRE(extinction > 0.0);
    if (wavelength == 0.55e-6)
      REQUIRE(extinction == Catch::Approx(3.91 / 23000.0));
  }
}

TEST_CASE("Incidence cosine handles both sides and voxel faces", "[energy]")
{
  double const angle = GENERATE(0.0, 0.2, PI / 4.0, 1.2, PI / 2.0);
  double const side = GENERATE(-1.0, 1.0);
  glm::dvec3 const origin(0, 0, 2);
  glm::dvec3 const point(0, 0, 1);
  glm::dvec3 const direction(std::sin(angle), 0, side * std::cos(angle));
  double const expected = std::cos(angle);
  Triangle triangle(Vertex(0, 0, 0), Vertex(1, 0, 0), Vertex(0, 1, 0));
  REQUIRE(triangle.getIncidenceAngleCosine(origin, direction, point) ==
          Catch::Approx(expected).margin(1e-15));
  REQUIRE(triangle.getIncidenceAngle_rad(origin, direction, point) ==
          Catch::Approx(angle).margin(1e-15));
  Voxel voxel(glm::dvec3(0), 2.0);
  voxel.v.normal = glm::dvec3(0, 0, -1);
  REQUIRE(voxel.getIncidenceAngleCosine(origin, direction, point) ==
          Catch::Approx(expected).margin(1e-15));
  voxel.v.normal = glm::dvec3(0);
  glm::dvec3 const oblique = glm::normalize(glm::dvec3(1, 2, 3));
  for (int axis = 0; axis < 3; ++axis) {
    glm::dvec3 face(0);
    face[axis] = side;
    REQUIRE(voxel.getIncidenceAngleCosine(origin, oblique, face) ==
            Catch::Approx(oblique[axis]));
  }
  // AABB retains its existing normal-incidence behavior via base fallback.
  AABB box(glm::dvec3(-1), glm::dvec3(1));
  REQUIRE(box.getIncidenceAngleCosine(origin, direction, point) == 1.0);
}

TEST_CASE(
  "Angular reflectance follows material formulas including fractional Phong",
  "[energy]")
{
  double const angle = GENERATE(0.0, 0.2, PI / 4.0, 1.2, PI / 2.0 - 1e-8);
  int const lighting = GENERATE(0, 1, 2);
  Material material;
  material.reflectance = 0.4;
  if (lighting > 0)
    material.kd[0] = 0.75;
  if (lighting == 2)
    material.ks[0] = 0.25;
  material.setSpecularity();
  material.specularExponent = 2.5;
  double const cosine = std::cos(angle);
  auto scanner = energyScanner();
  scanner->prepareSimulation();
  auto const& device = scanner->getScanningDevice(0);
  auto const model = device.getEnergyModel();
  double const geometry =
    PI * device.getSubrays()[0].share * 0.15 * 0.15 * 0.99 / (4 * 100 * 100);
  double expected = material.reflectance;
  if (lighting == 1)
    expected *= cosine;
  if (lighting == 2)
    expected *=
      (1 - material.specularity) * cosine +
      material.specularity * std::pow(std::abs(std::cos(2 * angle)), 2.5);
  REQUIRE(ratio(*model, cosine, 100.0, material) ==
          Catch::Approx(geometry * expected).epsilon(1e-12));
  double const grazing = ratio(*model, 0.0, 100.0, material);
  REQUIRE(std::isfinite(grazing));
  double const expectedGrazing =
    (lighting == 0) ? material.reflectance
                    : material.reflectance * material.specularity;
  REQUIRE(grazing == Catch::Approx(geometry * expectedGrazing).epsilon(1e-12));
}

TEST_CASE("Rho sampling conserves elliptical Gaussian power and area",
          "[energy]")
{
  int const quality = GENERATE(1, 2, 3, 8, 32);
  double const factor = GENERATE(0.1, 1.0, 1.5, 2.0, 4.0);
  double const divergenceX = GENERATE(0.0003, 0.2);
  double const divergenceY = GENERATE(0.0003, 0.001, 0.2);
  auto scanner = energyScanner();
  scanner->setBeamDivergences(divergenceX, divergenceY);
  auto& settings = scanner->getFWFSettings();
  settings.beamSampleQuality = quality;
  settings.beamSamplingFactor = factor;
  scanner->prepareSimulation();
  auto const& device = scanner->getScanningDevice(0);
  auto const& rays = device.getSubrays();
  double const tx = std::tan(divergenceX / 2.0);
  double const ty = std::tan(divergenceY / 2.0);
  double sum = 0.0, patchArea = 0.0;
  size_t index = 0;
  double inner = 0.0;
  for (int ring = 0; ring < quality; ++ring) {
    int const count = ring == 0 ? 1 : static_cast<int>(2.0 * PI * ring);
    double const outer = factor * (ring + 0.5) / (quality - 0.5);
    double const rho = factor * ring / (quality - 0.5);
    double const expectedRingShare =
      std::exp(-2 * inner * inner) - std::exp(-2 * outer * outer);
    double ringShare = 0.0;
    for (int j = 0; j < count; ++j, ++index) {
      auto const& ray = rays.at(index);
      REQUIRE(std::isfinite(ray.share));
      REQUIRE(ray.share >= 0.0);
      auto rotation = ray.rotation;
      auto const direction = rotation.applyTo(Directions::forward);
      double const x = direction.x / direction.y;
      double const y = direction.z / direction.y;
      REQUIRE(
        x ==
        Catch::Approx(rho * tx * std::sin(2 * PI * j / count)).margin(1e-14));
      REQUIRE(
        y ==
        Catch::Approx(rho * ty * std::cos(2 * PI * j / count)).margin(1e-14));
      REQUIRE(std::hypot(x / tx, y / ty) == Catch::Approx(rho).margin(1e-14));
      REQUIRE(ray.angle_rad ==
              Catch::Approx(std::atan2(std::hypot(x, y), 1.0)).margin(1e-14));
      REQUIRE(glm::length(direction) == Catch::Approx(1.0).margin(1e-14));
      REQUIRE(ray.rho == Catch::Approx(rho).margin(1e-14));
      REQUIRE(ray.innerRho == Catch::Approx(inner).margin(1e-14));
      REQUIRE(ray.outerRho == Catch::Approx(outer).margin(1e-14));
      REQUIRE(ray.share == rays[index - j].share);
      REQUIRE(
        ray.areaFactor ==
        Catch::Approx(PI * tx * ty * (outer * outer - inner * inner) / count));
      ringShare += ray.share;
      patchArea += ray.areaFactor;
    }
    REQUIRE(ringShare == Catch::Approx(expectedRingShare).margin(2e-15));
    sum += ringShare;
    inner = outer;
  }
  REQUIRE(index == rays.size());
  REQUIRE(scanner->getNumRays() == rays.size());
  REQUIRE(sum ==
          Catch::Approx(-std::expm1(-2 * factor * factor)).epsilon(1e-12));
  REQUIRE(patchArea ==
          Catch::Approx(PI * factor * factor * tx * ty).epsilon(1e-12));
  REQUIRE(rays.back().outerRho == factor);
}

TEST_CASE("Elliptical divergence updates are atomic and axis specific",
          "[energy]")
{
  auto scanner = energyScanner();
  scanner->setBeamDivergences(0.0003, 0.001);
  auto const before = scanner->getScanningDevice(0).getSubrays();
  for (double bad : { 0.0,
                      -1.0,
                      PI,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN() }) {
    REQUIRE_THROWS_AS(scanner->setBeamDivergences(0.002, bad),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(scanner->setBeamDivergences(bad, 0.002),
                      std::invalid_argument);
    REQUIRE(scanner->getBeamDivergenceX() == 0.0003);
    REQUIRE(scanner->getBeamDivergenceY() == 0.001);
    REQUIRE(scanner->getScanningDevice(0).isSubrayPatternCurrent());
    REQUIRE(scanner->getScanningDevice(0).getSubrays().back().angle_rad ==
            before.back().angle_rad);
  }
  scanner->setBeamDivergenceX(0.0006);
  REQUIRE(scanner->getBeamDivergenceY() == 0.001);
  scanner->setBeamDivergenceY(0.002);
  REQUIRE(scanner->getBeamDivergence() == 0.0006);
  auto copy = scanner->clone();
  REQUIRE(copy->getBeamDivergenceY() == 0.002);
  scanner->setBeamDivergence(0.003);
  REQUIRE(scanner->getBeamDivergenceY() == 0.003);
  REQUIRE(copy->getBeamDivergenceY() == 0.002);
}

TEST_CASE("Far field returns use absolute shares", "[energy]")
{
  auto scanner = energyScanner();
  scanner->setBeamDivergences(0.0003, GENERATE(0.0003, 0.001));
  scanner->getFWFSettings().beamSamplingFactor = GENERATE(1.0, 2.0);
  scanner->getFWFSettings().beamSampleQuality = GENERATE(1, 3, 8);
  scanner->prepareSimulation();
  auto const& device = scanner->getScanningDevice(0);
  auto model = device.getEnergyModel();
  Material material;
  material.reflectance = 0.5;
  double const range = GENERATE(1.0, 10.0, 100.0, 1000.0);
  double sumPower = 0.0, sumReturn = 0.0;
  for (size_t j = 0; j < device.getSubrays().size(); ++j) {
    std::size_t const index = j;
    auto const& ray = device.getSubrays()[j];
    double const power = model->computeEmittedPower(index);
    REQUIRE(power == Catch::Approx(4.0 * ray.share).epsilon(1e-12));
    double const expected =
      PI * power * 0.15 * 0.15 * 0.99 * 0.5 / (4.0 * range * range);
    double const receivedPower_W = received(*model, range, material, index);
    REQUIRE(receivedPower_W == Catch::Approx(expected).epsilon(1e-12));
    REQUIRE(received(*model, 2 * range, material, index) ==
            Catch::Approx(receivedPower_W / 4).epsilon(1e-12));
    double const patchArea = area(*model, range * range, index);
    double const sigma = 4 * PI * 0.5 * patchArea;
    REQUIRE(model->computeReceivedPowerWithSigma(range, sigma, index) ==
            Catch::Approx(receivedPower_W).epsilon(1e-12));
    REQUIRE(model->computeReceivedPowerWithSigma(2 * range, sigma, index) ==
            Catch::Approx(receivedPower_W / 16).epsilon(1e-12));
    sumPower += power;
    sumReturn += receivedPower_W;
  }
  double const factor = scanner->getFWFSettings().beamSamplingFactor;
  double const captured = -std::expm1(-2 * factor * factor);
  REQUIRE(sumPower == Catch::Approx(4 * captured).epsilon(1e-12));
  REQUIRE(sumReturn == Catch::Approx(PI * 4 * captured * 0.15 * 0.15 * 0.99 *
                                     0.5 / (4 * range * range))
                         .epsilon(1e-12));
  scanner->setPulseAmplitude(8.0);
  REQUIRE(model->computeEmittedPower(0) ==
          Catch::Approx(8 * device.getSubrays()[0].share));
}

TEST_CASE("Subray settings changes and copies cannot use stale tables",
          "[energy]")
{
  auto scanner = energyScanner();
  scanner->prepareSimulation();
  auto& device = scanner->getScanningDevice(0);
  auto copy = scanner->clone();
  REQUIRE(copy->getScanningDevice(0).getEnergyModel().get() !=
          device.getEnergyModel().get());
  copy->setPulseAmplitude(8.0);
  REQUIRE(copy->getScanningDevice(0).getEnergyModel()->computeEmittedPower(0) ==
          Catch::Approx(2 * device.getEnergyModel()->computeEmittedPower(0)));

  double const previousAngle = device.getSubrays().back().angle_rad;
  scanner->setBeamDivergence(0.0006);
  REQUIRE(device.isSubrayPatternCurrent());
  REQUIRE(device.getSubrays().back().angle_rad > previousAngle);
  scanner->getFWFSettings().beamSamplingFactor = 1.0;
  REQUIRE_FALSE(device.isSubrayPatternCurrent());
  scanner->getFWFSettings().beamSampleQuality = 8;
  scanner->calcRaysNumber();
  REQUIRE(device.getSubrays().size() == 173);
  REQUIRE(scanner->getNumRays() == 173);
  REQUIRE(copy->getScanningDevice(0).getSubrays().size() == 19);
}

TEST_CASE("Subray sampling validates settings", "[energy]")
{
  auto scanner = energyScanner();
  auto& settings = scanner->getFWFSettings();
  SECTION("Invalid quality")
  {
    settings.beamSampleQuality = GENERATE(0, -1);
  }
  SECTION("Invalid factor")
  {
    settings.beamSamplingFactor =
      GENERATE(0.0,
               -1.0,
               std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::quiet_NaN());
  }
  SECTION("Invalid divergence")
  {
    double const invalid = GENERATE(0.0,
                                    -1.0,
                                    PI,
                                    std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::quiet_NaN());
    scanner->prepareSimulation();
    double const previousAngle =
      scanner->getScanningDevice(0).getSubrays().back().angle_rad;
    REQUIRE_THROWS_AS(scanner->setBeamDivergence(invalid),
                      std::invalid_argument);
    REQUIRE(scanner->getBeamDivergence() == 0.0003);
    REQUIRE(scanner->getScanningDevice(0).isSubrayPatternCurrent());
    REQUIRE(scanner->getScanningDevice(0).getSubrays().back().angle_rad ==
            previousAngle);
    return;
  }
  REQUIRE_THROWS_AS(scanner->prepareSimulation(), std::invalid_argument);
}

TEST_CASE("Channels own independent subray geometry and weights", "[energy]")
{
  std::vector<ScanningDevice> devices;
  for (int j = 0; j < 2; ++j) {
    devices.emplace_back(j,
                         "channel",
                         0.0003 * (j + 1),
                         glm::dvec3(0),
                         Rotation(),
                         std::list<int>{ 100000 },
                         5.0,
                         4.0,
                         0.99,
                         0.15,
                         23.0,
                         1064e-9);
  }
  MultiScanner scanner(std::move(devices), "multi", std::list<int>{ 100000 });
  scanner.getFWFSettings(0).beamSamplingFactor = 1.0;
  scanner.getFWFSettings(1).beamSamplingFactor = 2.0;
  scanner.getFWFSettings(1).beamSampleQuality = 8;
  scanner.getFWFSettings(1).pulseShape = "gaussian";
  scanner.setPulseLength_ns(10.0, 1);
  scanner.prepareSimulation();
  REQUIRE(scanner.getPulseModel(0).evaluate(-0.01) == 0);
  REQUIRE(scanner.getPulseModel(1).evaluate(-0.01) > 0);
  REQUIRE(scanner.getPulseModel(0).peakOffset() !=
          scanner.getPulseModel(1).peakOffset());
  double const previousAngle =
    scanner.getScanningDevice(1).getSubrays().back().angle_rad;
  scanner.setBeamDivergence(0.003, 1);
  REQUIRE(scanner.getBeamDivergence(0) == 0.0003);
  REQUIRE(scanner.getBeamDivergence(1) == 0.003);
  REQUIRE(scanner.getScanningDevice(1).getSubrays().back().angle_rad >
          previousAngle);
  for (size_t channel = 0; channel < 2; ++channel) {
    auto const& device = scanner.getScanningDevice(channel);
    double sum = 0;
    for (auto const& ray : device.getSubrays())
      sum += ray.share;
    REQUIRE(sum ==
            Catch::Approx(-std::expm1(-2.0 * (channel + 1) * (channel + 1))));
    REQUIRE(scanner.getNumRays(channel) == (channel == 0 ? 19 : 173));
  }
}

#if DATA_ANALYTICS < 2
namespace {
class CosineOnlyTriangle : public Triangle
{
public:
  using Triangle::Triangle;
  double callbackSum = 0.0;
  bool canHandleIntersections() override { return true; }
  IntersectionHandlingResult onRayIntersection(NoiseSource<double>&,
                                               glm::dvec3&,
                                               glm::dvec3 const& inside,
                                               glm::dvec3 const&,
                                               double receivedPower_nW) override
  {
    callbackSum += receivedPower_nW;
    return IntersectionHandlingResult(inside, false);
  }
  double getIncidenceAngle_rad(const glm::dvec3&,
                               const glm::dvec3&,
                               const glm::dvec3&) override
  {
    throw std::logic_error("Tracing must use the cosine accessor");
  }
};

class WaveformTestScene : public Scene
{
public:
  WaveformTestScene()
  {
    bbox = std::make_shared<AABB>(glm::dvec3(-200), glm::dvec3(200));
    bbox_crs = bbox;
  }
};

class RecordingWaveformYielder : public FullWaveformYielder
{
public:
  std::vector<FullWaveform> recorded;
  explicit RecordingWaveformYielder(helios::filems::FMSWriteFacade& writer)
    : FullWaveformYielder(writer, 1)
  {
  }
  void digest(std::vector<FullWaveform>& copy) override
  {
    recorded.insert(recorded.end(), copy.begin(), copy.end());
  }
};

class TracingTestPulse : public FullWaveformPulseRunnable
{
public:
  using FullWaveformPulseRunnable::computeSubrays;
  using FullWaveformPulseRunnable::digestIntersections;
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;
  using FullWaveformPulseRunnable::populateFullWaveform;
};
}

TEST_CASE("Tracing adds coincident returns without redistributing misses",
          "[energy]")
{
  double const xmin = GENERATE(-1.0, 0.002);
  bool const fixedIncidence = GENERATE(false, true);
  auto scanner = energyScanner();
  scanner->setBeamDivergences(0.0003, GENERATE(0.0003, 0.001));
  scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01));
  scanner->prepareSimulation();
  Scene scene;
  auto part = std::make_shared<ScenePart>();
  auto material = std::make_shared<Material>();
  material->reflectance = 0.5;
  material->kd[0] = 1.0;
  scanner->setFixedIncidenceAngle(fixedIncidence);
  scene.primitives.push_back(new CosineOnlyTriangle(
    Vertex(xmin, 100, -1), Vertex(1, 100, -1), Vertex(1, 100, 1)));
  scene.primitives.push_back(new CosineOnlyTriangle(
    Vertex(xmin, 100, -1), Vertex(1, 100, 1), Vertex(xmin, 100, 1)));
  for (auto primitive : scene.primitives) {
    primitive->part = part;
    primitive->material = material;
    part->mPrimitives.push_back(primitive);
  }
  REQUIRE(scene.finalizeLoading());
  Rotation attitude(Directions::forward, GENERATE(0.0, PI / 2));
  SimulatedPulse pulse(-scene.getShift(), attitude, 0.0, 0, 1, 0);
  TracingTestPulse runnable(scanner, scene, pulse);
  runnable.AbstractPulseRunnable::initialize();
  UniformNoiseSource<double> noise("42");
  std::map<double, double> reflections;
  std::vector<RaySceneIntersection> intersections;
  runnable.computeSubrays(noise, reflections, intersections);

  size_t expectedHits = 0;
  double expected = 0.0;
  auto const& rays = scanner->getScanningDevice(0).getSubrays();
  for (auto const& ray : rays) {
    auto rotation = ray.rotation;
    auto direction = attitude.applyTo(rotation.applyTo(Directions::forward));
    if (100.0 * direction.x / direction.y < xmin)
      continue;
    ++expectedHits;
    double const range = 100.0 / direction.y;
    expected += PI * ray.share * 0.15 * 0.15 * 0.99 * 0.5 /
                (4.0 * range * range) * (fixedIncidence ? 1.0 : direction.y);
  }
  double total = 0.0;
  for (auto const& reflection : reflections)
    total += reflection.second;
  REQUIRE(intersections.size() == expectedHits);
  REQUIRE(total == Catch::Approx(expected).epsilon(1e-10));
  double callbackSum = 0.0;
  for (auto primitive : scene.primitives)
    callbackSum += static_cast<CosineOnlyTriangle*>(primitive)->callbackSum;
  REQUIRE(callbackSum ==
          Catch::Approx(expected * scanner->getPulseAmplitude() * 1e9)
            .epsilon(1e-10));
  if (xmin < 0) {
    REQUIRE(expectedHits == rays.size());
    REQUIRE(reflections.size() < expectedHits);
  } else {
    REQUIRE(expectedHits > 0);
    REQUIRE(expectedHits < rays.size());
  }
}
#endif

TEST_CASE(
  "Whole-beam amplitude validation is shared by construction and setters",
  "[energy]")
{
  double const invalid = GENERATE(-1.0,
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN());
  REQUIRE_THROWS_AS(energyScanner(invalid), std::invalid_argument);
  auto scanner = energyScanner();
  REQUIRE_THROWS_AS(scanner->setPulseAmplitude(invalid), std::invalid_argument);
  REQUIRE(scanner->getPulseAmplitude() == 4.0);
  scanner->setPulseAmplitude(0.0);
  REQUIRE(scanner->getPulseAmplitude() == 0.0);
}

TEST_CASE("Power ratios are independent of amplitude and pulse frequency",
          "[energy]")
{
  auto scanner = energyScanner();
  scanner->prepareSimulation();
  auto model = scanner->getScanningDevice(0).getEnergyModel();
  Material material;
  material.reflectance = 0.5;
  double const sigma = 4 * PI * 0.5 * area(*model, 10000.0, 0);
  double const baseline = model->computePowerRatioWithSigma(100.0, sigma, 0);
  double const amplitude = GENERATE(0.0, 1.0, 7.5);
  scanner->setPulseAmplitude(amplitude);
  scanner->setPulseFreq_Hz(GENERATE(50000, 100000));
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> records;
  double const powerRatio =
    model->computePowerRatio(1.0, 100.0, material, 0, records);
  REQUIRE(records.size() == 1);
  REQUIRE(records.back().size() == 13);
  REQUIRE(records.back()[3] == 0.0);
  REQUIRE(records.back()[4] == 100.0);
  REQUIRE(records.back()[5] ==
          Catch::Approx(model->computeTargetArea(100.0, 0)));
  REQUIRE(records.back()[7] == material.reflectance);
  REQUIRE(records.back()[8] == Catch::Approx(sigma));
  REQUIRE(records.back()[12] == 0.0);
  // Geometry queries do not append or alter analytics records.
  auto const savedRecord = records.back();
  REQUIRE(model->computeTargetArea(200.0, 0) ==
          Catch::Approx(4 * records.back()[5]));
  REQUIRE(records.size() == 1);
  REQUIRE(records.back()[5] == savedRecord[5]);
  REQUIRE(records.back()[9] == Catch::Approx(amplitude * baseline));
  REQUIRE(records.back()[11] ==
          Catch::Approx(amplitude *
                        scanner->getScanningDevice(0).getSubrays()[0].share));
#else
  double const powerRatio = model->computePowerRatio(1.0, 100.0, material, 0);
#endif
  REQUIRE(powerRatio == Catch::Approx(baseline));
  REQUIRE(model->computePowerRatioWithSigma(100.0, sigma, 0) ==
          Catch::Approx(baseline));
  REQUIRE(received(*model, 100.0, material, 0) ==
          Catch::Approx(amplitude * baseline));
  REQUIRE(model->computeReceivedPowerWithSigma(100.0, sigma, 0) ==
          Catch::Approx(amplitude * baseline));
}

TEST_CASE("Analytic pulse normalization, FWHM, and full tails",
          "[energy][waveform]")
{
  double const length = GENERATE(5.0, 12.0);
  std::string const shape = GENERATE("gamma", "gaussian");
  auto model = PulseShape::create(shape, length, 1e-4);
  double const peak = model->peakOffset();
  REQUIRE(model->evaluate(peak) == Catch::Approx(1.0));
  auto bounds = model->recordingBounds();
  REQUIRE(model->evaluate(bounds.second) == Catch::Approx(1e-4));
  REQUIRE(model->evaluate(bounds.second + 0.01) > 0);
  double const fwhm = length * SPEEDofLIGHT_mPerNanosec / 2;
  if (shape == "gaussian") {
    REQUIRE(model->evaluate(peak - fwhm / 2) == Catch::Approx(0.5));
    REQUIRE(model->evaluate(peak + fwhm / 2) == Catch::Approx(0.5));
    REQUIRE(bounds.first < 0);
    REQUIRE(model->evaluate(bounds.first) == Catch::Approx(1e-4));
    REQUIRE(model->evaluate(-0.01) > 0);
  } else {
    REQUIRE(bounds.first == 0);
    REQUIRE(model->evaluate(-1) == 0);
    REQUIRE(model->evaluate(0) == 0);
    // Independent half-height roots of x^2 exp(-x).
    REQUIRE(model->evaluate(0.7612402293544026 * peak / 2) ==
            Catch::Approx(0.5));
    REQUIRE(model->evaluate(4.155920900200907 * peak / 2) ==
            Catch::Approx(0.5));
    REQUIRE(model->evaluate(7 * peak / 2) > 0.08);
  }
  REQUIRE(model->evaluate(1e300) == 0);
  REQUIRE_THROWS_AS(PulseShape::create(shape, 0, 1e-4), std::invalid_argument);
  REQUIRE_THROWS_AS(PulseShape::create(shape, length, 1),
                    std::invalid_argument);
}

TEST_CASE("Gaussian mean translates the pulse and recording bounds",
          "[energy][waveform]")
{
  double const mean = GENERATE(-5.0, 0.0, 3.0);
  GaussianPulse origin(4.0);
  GaussianPulse shifted(4.0, 1e-4, mean);
  double const shift = mean * SPEEDofLIGHT_mPerNanosec / 2;
  REQUIRE(shifted.peakOffset() == Catch::Approx(shift));
  REQUIRE(shifted.recordingBounds().first ==
          Catch::Approx(origin.recordingBounds().first + shift));
  REQUIRE(shifted.recordingBounds().second ==
          Catch::Approx(origin.recordingBounds().second + shift));
  for (double x : { -0.8, -0.1, 0.0, 0.3, 1.0 })
    REQUIRE(shifted.evaluate(x + shift) == Catch::Approx(origin.evaluate(x)));
  REQUIRE_THROWS_AS(
    GaussianPulse(4, 1e-4, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST_CASE("Sampled pulses interpolate independently of input density",
          "[energy][waveform]")
{
  SampledPulse coarse({ -2, 0, 3, 7 }, { 0, 4, 2, 0 });
  SampledPulse dense({ -2, -1, 0, 1.5, 3, 5, 7 }, { 0, 2, 4, 3, 2, 1, 0 });
  SampledPulse translated({ 1, 3, 6, 10 }, { 0, 4, 2, 0 });
  double const scale = SPEEDofLIGHT_mPerNanosec / 2;
  REQUIRE(coarse.peakOffset() == 0);
  REQUIRE(coarse.evaluate(-scale) == Catch::Approx(0.5));
  REQUIRE(coarse.evaluate(1.5 * scale) == Catch::Approx(0.75));
  REQUIRE(coarse.evaluate(-3 * scale) == 0);
  REQUIRE(coarse.evaluate(8 * scale) == 0);
  for (double spacing : { 0.013, 0.037, 0.1 }) {
    for (double x = -1; x < 2; x += spacing) {
      REQUIRE(coarse.evaluate(x) ==
              Catch::Approx(dense.evaluate(x)).margin(1e-14));
      REQUIRE(coarse.evaluate(x) ==
              Catch::Approx(translated.evaluate(x + 3 * scale)).margin(1e-14));
    }
  }
  SampledPulse plateau({ -2, 0, 3, 7 }, { 0, 1, 1, 0 }, 1.5);
  REQUIRE(plateau.peakOffset() == Catch::Approx(1.5 * scale));
  SampledPulse endpoints({ 0, 1 }, { 1, 0.5 });
  REQUIRE(endpoints.evaluate(0) == 1);
  REQUIRE(endpoints.evaluate(scale) == 0.5);
  REQUIRE(endpoints.evaluate(-0.001) == 0);
  REQUIRE(endpoints.evaluate(scale + 0.001) == 0);
  REQUIRE_THROWS_AS(SampledPulse({ 0 }, { 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 1 }, { 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 0 }, { 0, 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 1, 0 }, { 0, 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 1 }, { 0, 0 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 1 }, { -1, 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 1 }, { 1, 1 }), std::invalid_argument);
  REQUIRE_THROWS_AS(SampledPulse({ 0, 1 }, { 0, 1 }, 0.5),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(
    SampledPulse({ 0, std::numeric_limits<double>::infinity() }, { 0, 1 }),
    std::invalid_argument);
  REQUIRE_THROWS_AS(
    SampledPulse({ 0, 1 }, { 0, std::numeric_limits<double>::quiet_NaN() }),
    std::invalid_argument);
}

TEST_CASE(
  "Range grid preserves spacing with distant returns and clips only recording",
  "[energy][waveform]")
{
  auto model = PulseShape::create("gaussian", 5, 1e-4);
  auto near = WaveMaths::recordingGrid(10.013, 10.034, *model, 0.04, 0);
  auto far = WaveMaths::recordingGrid(10.013, 1000, *model, 0.04, 0);
  REQUIRE(far.sampleCount > near.sampleCount);
  for (int i = 0; i < near.sampleCount; ++i)
    REQUIRE(near.rangeAt(i) == far.rangeAt(i));
  auto clipped = WaveMaths::recordingGrid(10.013, 1000, *model, 0.04, 0.105);
  REQUIRE(clipped.sampleCount == 3);
  REQUIRE(clipped.rangeAt(2) - clipped.rangeAt(0) <= 0.105);
  auto shortGrid = WaveMaths::recordingGrid(10, 10, *model, 0.04, 0.001);
  REQUIRE(shortGrid.sampleCount == 1);
  REQUIRE_THROWS_AS(WaveMaths::recordingGrid(0, 1e100, *model, 0.04, 0),
                    std::length_error);
  REQUIRE_THROWS_AS(WaveMaths::recordingGrid(0, 1, *model, 0, 0),
                    std::invalid_argument);
}

#if DATA_ANALYTICS < 2
TEST_CASE("Waveform evaluates exact fractional returns including distant tails",
          "[energy][waveform]")
{
  std::string const shape = GENERATE("gamma", "gaussian");
  auto scanner = energyScanner();
  scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01));
  scanner->getFWFSettings().pulseShape = shape;
  scanner->preparePulse();
  WaveformTestScene scene;
  SimulatedPulse pulse(glm::dvec3(0), Rotation(), 0, 0, 1, 0);
  TracingTestPulse runnable(scanner, scene, pulse);
  std::map<double, double> reflections{ { 10.013, 1e-6 },
                                        { 10.037, 2e-6 },
                                        { 12.3, 1e-6 } };
  auto const& model = scanner->getPulseModel();
  auto grid = WaveMaths::recordingGrid(10.013, 12.3, model, 0.03, 0);
  std::vector<double> samples(grid.sampleCount, 0);
  runnable.populateFullWaveform(reflections, samples, grid);
  double const width = 5 * SPEEDofLIGHT_mPerNanosec / 2;
  for (int k = 0; k < grid.sampleCount; ++k) {
    double expected = 0;
    for (auto const& entry : reflections) {
      double u = grid.rangeAt(k) - entry.first;
      double value;
      if (shape == "gamma") {
        double x = u / (width / 3.394680670846504);
        value = x < 0 ? 0 : std::exp(2.0) / 4 * x * x * std::exp(-x);
      } else {
        double z = u / (width / std::sqrt(8 * std::log(2.0)));
        value = std::exp(-z * z / 2);
      }
      expected += entry.second * value;
    }
    REQUIRE(samples[k] == Catch::Approx(expected * 4e9).margin(1e-10));
  }
  // A recording cap must not remove a later Gaussian return's leading tail.
  WaveformGrid clipped{ 400, 0.03, 3 };
  std::vector<double> tail(3, 0);
  runnable.populateFullWaveform({ { 12.3, 1 } }, tail, clipped);
  if (shape == "gaussian")
    REQUIRE(tail[0] > 0);
  else
    REQUIRE(tail[0] == 0);
}

TEST_CASE("Peak correction changes only measured ranges and preserves "
          "amplitude detection",
          "[energy][waveform]")
{
  std::string const shape = GENERATE("gamma", "gaussian", "sampled");
  // Keep even uncorrected negative-mean peaks inside the detector range.
  double const target = GENERATE(1.02, 100.013);
  auto scanner = energyScanner();
  scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01));
  auto& settings = scanner->getFWFSettings();
  settings.pulseShape = shape;
  settings.gaussianMean_ns = GENERATE(-3.0, 0.0, 4.0);
  if (shape == "sampled")
    settings.sampledPulse =
      std::make_shared<SampledPulse>(std::vector<double>{ -4, 0, 2, 5, 9 },
                                     std::vector<double>{ 0, 0.4, 1, 0.5, 0 });
  settings.sampleSpacing_m = 0.005;
  settings.peakWindow_m = 0.01;
  scanner->preparePulse();
  scanner->allMeasurements = std::make_shared<std::vector<Measurement>>();
  scanner->allMeasurementsMutex = std::make_shared<std::mutex>();
  WaveformTestScene scene;
  SimulatedPulse pulse(glm::dvec3(0), Rotation(), 0, 0, 1, 0);
  TracingTestPulse runnable(scanner, scene, pulse);
  AbstractPulseRunnable& base = runnable;
  base.initialize();
  helios::filems::FMSWriteFacade writer;
  auto recorder = std::make_shared<RecordingWaveformYielder>(writer);
  scanner->getDetector()->fwfYielder = recorder;
  scanner->setWriteWaveform(true);
  auto part = std::make_shared<ScenePart>();
  part->setId("waveform-test");
  AABB primitive(glm::dvec3(-1), glm::dvec3(1));
  primitive.part = part;
  primitive.material = std::make_shared<Material>();
  RaySceneIntersection intersection;
  intersection.prim = &primitive;
  intersection.point = glm::dvec3(0, target, 0);
  intersection.hitDistance = target;
  std::vector<RaySceneIntersection> intersections{ intersection };
  std::map<double, double> reflections{ { target, 1e-6 } };
  std::vector<std::vector<double>> matrix;
  RandomnessGenerator<double> rng(42), rng2(43);
  rng.computeNormalDistribution(0.0, 0.0);
  glm::dvec3 direction(0, 1, 0);
  WaveformGrid grid;
  REQUIRE(scanner->initializeFullWaveform(target, target, grid, 0));
  std::vector<double> wave(grid.sampleCount, 0);
  runnable.populateFullWaveform(reflections, wave, grid);
  double correctedDistance = 0;
  for (bool corrected : { true, false }) {
    settings.correctPeakOffset = corrected;
    for (double amplitude : { 0.0, 1.0, 2.0, 4.0 }) {
      CAPTURE(shape, target, settings.gaussianMean_ns, corrected, amplitude);
      scanner->setPulseAmplitude(amplitude);
      scanner->setReceivedEnergyMin(amplitude == 0 ? 0 : 1500.0);
      scanner->allMeasurements->clear();
      recorder->recorded.clear();
      runnable.digestIntersections(
        matrix, rng, rng2, direction, reflections, intersections);
      auto const& points = *scanner->allMeasurements;
      if (amplitude < 1.5) {
        REQUIRE(points.empty());
        REQUIRE(recorder->recorded.empty());
      } else {
        REQUIRE(points.size() == 1);
        REQUIRE(recorder->recorded.size() == 1);
        auto const& exported = recorder->recorded.front();
        REQUIRE(exported.minTime ==
                Catch::Approx(2 * grid.rangeAt(0) / SPEEDofLIGHT_mPerNanosec));
        REQUIRE(exported.maxTime ==
                Catch::Approx(2 * grid.rangeAt(grid.sampleCount - 1) /
                              SPEEDofLIGHT_mPerNanosec));
        for (size_t k = 0; k < wave.size(); ++k)
          REQUIRE(exported.fullwave[k] ==
                  Catch::Approx(wave[k] * amplitude / 4));
        double expected =
          target + (corrected ? 0 : scanner->getPulseModel().peakOffset());
        REQUIRE(std::abs(points[0].distance - expected) <=
                settings.sampleSpacing_m);
        REQUIRE(points[0].intensity ==
                Catch::Approx(amplitude *
                              *std::max_element(wave.begin(), wave.end()) / 4));
        if (corrected)
          correctedDistance = points[0].distance;
        else
          REQUIRE(points[0].distance - correctedDistance ==
                  Catch::Approx(scanner->getPulseModel().peakOffset()));
      }
    }
  }
  std::vector<double> uncorrectedWave(grid.sampleCount, 0);
  runnable.populateFullWaveform(reflections, uncorrectedWave, grid);
  REQUIRE(uncorrectedWave == wave);
  if (shape == "gaussian") {
    scanner->setCalcEchowidth(true);
    settings.correctPeakOffset = true;
    scanner->allMeasurements->clear();
    runnable.digestIntersections(
      matrix, rng, rng2, direction, reflections, intersections);
    REQUIRE(scanner->allMeasurements->size() == 1);
    // The existing fitter uses exp(-((x-centre)/width)^2), so
    // width=sqrt(2)*sigma.
    REQUIRE(scanner->allMeasurements->front().echo_width ==
            Catch::Approx(5.0 / (2 * std::sqrt(std::log(2.0)))).epsilon(0.001));
  }
  if (shape == "gaussian") {
    // The unresolved pair must remain at the physical combined maximum,
    // rather than being corrected back to either surface.
    scanner->setCalcEchowidth(false);
    scanner->setReceivedEnergyMin(0);
    reflections[target + 0.2] = 1e-6;
    scanner->allMeasurements->clear();
    runnable.digestIntersections(
      matrix, rng, rng2, direction, reflections, intersections);
    REQUIRE(scanner->allMeasurements->size() == 1);
    REQUIRE(scanner->allMeasurements->front().distance ==
            Catch::Approx(target + 0.1).margin(settings.sampleSpacing_m));
  }
  reflections.clear();
  scanner->allMeasurements->clear();
  runnable.digestIntersections(
    matrix, rng, rng2, direction, reflections, intersections);
  REQUIRE(scanner->allMeasurements->empty());
}
#endif

#if DATA_ANALYTICS >= 2
namespace {
class AnalyticsTriangle : public Triangle
{
public:
  using Triangle::Triangle;
  bool useSigma = false;
  bool canComputeSigmaWithLadLut() override { return useSigma; }
  double computeSigmaWithLadLut(glm::dvec3 const&) override { return 0.01; }
};
class AnalyticsTestPulse : public FullWaveformPulseRunnable
{
public:
  using FullWaveformPulseRunnable::computeSubrays;
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;
};
}
TEST_CASE("Both analytics paths record transverse hit radii", "[energy]")
{
  bool const useSigma = GENERATE(false, true);
  auto scanner = energyScanner();
  scanner->setBeamDivergences(0.0003, 0.001);
  scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01));
  scanner->prepareSimulation();
  Scene scene;
  auto part = std::make_shared<ScenePart>();
  auto material = std::make_shared<Material>();
  material->reflectance = 0.5;
  material->kd[0] = 1.0;
  scene.primitives.push_back(new AnalyticsTriangle(
    Vertex(-1, 100, -1), Vertex(1, 100, -1), Vertex(1, 100, 1)));
  scene.primitives.push_back(new AnalyticsTriangle(
    Vertex(-1, 100, -1), Vertex(1, 100, 1), Vertex(-1, 100, 1)));
  for (auto primitive : scene.primitives) {
    primitive->part = part;
    primitive->material = material;
    static_cast<AnalyticsTriangle*>(primitive)->useSigma = useSigma;
    part->mPrimitives.push_back(primitive);
  }
  REQUIRE(scene.finalizeLoading());
  Rotation attitude(Directions::forward, GENERATE(0.0, PI / 2));
  SimulatedPulse pulse(-scene.getShift(), attitude, 0.0, 0, 1, 0);
  AnalyticsTestPulse runnable(scanner, scene, pulse);
  runnable.AbstractPulseRunnable::initialize();
  UniformNoiseSource<double> noise("42");
  auto const& rays = scanner->getScanningDevice(0).getSubrays();
  auto const nonce =
    std::chrono::steady_clock::now().time_since_epoch().count();
  struct RecordDirectory
  {
    std::filesystem::path path;
    ~RecordDirectory()
    {
      std::error_code ec;
      std::filesystem::remove_all(path, ec);
    }
  } directory{ std::filesystem::temp_directory_path() /
               ("helios_elliptical_records_" + std::to_string(nonce)) };
  std::filesystem::create_directory(directory.path);
  auto recorder = std::make_shared<helios::analytics::HDA_PulseRecorder>(
    directory.path.string());
  std::map<double, double> reflections;
  std::vector<RaySceneIntersection> intersections;
  std::vector<std::vector<double>> records;
  runnable.computeSubrays(noise, reflections, intersections, records, recorder);
  REQUIRE(records.size() == rays.size());
  REQUIRE(intersections.size() == rays.size());
  for (size_t i = 0; i < rays.size(); ++i) {
    auto const point = intersections.at(i).point + scene.getShift();
    REQUIRE(records[i][6] ==
            Catch::Approx(std::hypot(point.x, point.z)).margin(1e-12));
    if (i == 0)
      REQUIRE(records[i][6] == 0.0);
    REQUIRE(records[i][5] > 0.0);
    REQUIRE(records[i][9] > 0.0);
  }
}
#endif
