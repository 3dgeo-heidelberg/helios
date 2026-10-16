#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <algorithm>
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
                                                 1.0,
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

TEST_CASE("Subray table conserves captured Gaussian power", "[energy]")
{
  int const quality = GENERATE(1, 2, 3, 8, 32);
  double const factor = GENERATE(0.1, 1.0, 1.5, 2.0, 4.0);
  double const divergence = GENERATE(0.0003, 0.2);
  auto scanner = energyScanner();
  scanner->setBeamDivergence(divergence);
  auto& settings = scanner->getFWFSettings();
  settings.beamSampleQuality = quality;
  settings.beamSamplingFactor = factor;
  scanner->prepareSimulation();
  auto const& device = scanner->getScanningDevice(0);
  auto const& rays = device.getSubrays();
  double const tangent0 = std::tan(divergence / 2.0);
  double const cutoff = std::atan(factor * tangent0);
  double sum = 0.0, patchArea = 0.0;
  size_t index = 0;
  double inner = 0.0;
  for (int ring = 0; ring < quality; ++ring) {
    int const count = ring == 0 ? 1 : static_cast<int>(2.0 * PI * ring);
    double const outer = cutoff * (ring + 0.5) / (quality - 0.5);
    double const expectedRingShare =
      std::exp(-2 * std::pow(std::tan(inner) / tangent0, 2)) -
      std::exp(-2 * std::pow(std::tan(outer) / tangent0, 2));
    double ringShare = 0.0;
    for (int j = 0; j < count; ++j, ++index) {
      auto const& ray = rays.at(index);
      REQUIRE(std::isfinite(ray.share));
      REQUIRE(ray.share >= 0.0);
      auto rotation = ray.rotation;
      auto const direction = rotation.applyTo(Directions::forward);
      double const actualAngle =
        std::atan2(glm::length(glm::cross(direction, Directions::forward)),
                   glm::dot(direction, Directions::forward));
      REQUIRE(actualAngle ==
              Catch::Approx(cutoff * ring / (quality - 0.5)).margin(1e-14));
      REQUIRE(glm::length(direction) == Catch::Approx(1.0).margin(1e-14));
      REQUIRE(ray.outerAngle_rad == Catch::Approx(outer).margin(1e-14));
      REQUIRE(ray.areaFactor == Catch::Approx(PI *
                                              (std::pow(std::tan(outer), 2) -
                                               std::pow(std::tan(inner), 2)) /
                                              count)
                                  .epsilon(1e-12));
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
          Catch::Approx(PI * std::pow(factor * tangent0, 2)).epsilon(1e-12));
  REQUIRE(rays.back().outerAngle_rad == cutoff);
  scanner->prepareSimulation();
  REQUIRE(device.getSubrays().size() == index);
}

TEST_CASE("Far field returns use absolute shares", "[energy]")
{
  auto scanner = energyScanner();
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
  REQUIRE(device.cached_Bt2 == Catch::Approx(0.0006 * 0.0006));
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
                         1.0,
                         0.99,
                         0.15,
                         23.0,
                         1064e-9);
  }
  MultiScanner scanner(std::move(devices), "multi", std::list<int>{ 100000 });
  scanner.getFWFSettings(0).beamSamplingFactor = 1.0;
  scanner.getFWFSettings(1).beamSamplingFactor = 2.0;
  scanner.getFWFSettings(1).beamSampleQuality = 8;
  scanner.prepareSimulation();
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

class TracingTestPulse : public FullWaveformPulseRunnable
{
public:
  using FullWaveformPulseRunnable::computeSubrays;
  using FullWaveformPulseRunnable::digestIntersections;
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;
};
}

TEST_CASE("Tracing adds coincident returns without redistributing misses",
          "[energy]")
{
  double const xmin = GENERATE(-1.0, 0.002);
  bool const fixedIncidence = GENERATE(false, true);
  auto scanner = energyScanner();
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
  SimulatedPulse pulse(-scene.getShift(), Rotation(), 0.0, 0, 1, 0);
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
    auto direction = rotation.applyTo(Directions::forward);
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

TEST_CASE("Temporal peak normalization preserves migrated samples and "
          "whole-beam energy",
          "[energy]")
{
  double const length = GENERATE(5.0, 12.0);
  double const bin = GENERATE(0.01, 0.1, 0.7);
  int const count = static_cast<int>(length / bin);
  std::vector<double> wave(count);
  int const peak =
    WaveMaths::calcPropagationTimeLegacy(wave, count, bin, length, 7.0);
  REQUIRE(wave[peak] <= 1.0 + 1e-14);
  double integral = 0;
  for (int i = 0; i < count; ++i) {
    double const x = i * bin / (length / 7.0);
    double const legacy = x * x * std::exp(-x);
    REQUIRE(wave[i] * (16.0 / std::exp(2.0)) ==
            Catch::Approx(4.0 * legacy).margin(1e-14));
    integral += wave[i] * bin * 1e-9;
  }
  // Use a grid that includes the analytic peak explicitly.
  std::vector<double> exact(3);
  REQUIRE(WaveMaths::calcPropagationTimeLegacy(
            exact, 3, length / 7.0, length, 7.0) == 2);
  REQUIRE(exact[2] == Catch::Approx(1.0).epsilon(1e-14));
  auto scanner = energyScanner();
  double const factor = GENERATE(1.0, 2.0);
  scanner->getFWFSettings().beamSamplingFactor = factor;
  scanner->getFWFSettings().beamSampleQuality = GENERATE(1, 3, 8);
  scanner->prepareSimulation();
  double power = 0;
  auto model = scanner->getScanningDevice(0).getEnergyModel();
  for (size_t i = 0; i < scanner->getNumRays(); ++i)
    power += model->computeEmittedPower(i);
  double const captured = -std::expm1(-2 * factor * factor);
  REQUIRE(power == Catch::Approx(4.0 * captured).epsilon(1e-12));
  REQUIRE(power * integral ==
          Catch::Approx(4.0 * captured * integral).epsilon(1e-12));
}

#if DATA_ANALYTICS < 2
TEST_CASE("Waveform detection uses scaled whole-beam amplitude", "[energy]")
{
  auto scanner = energyScanner();
  scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.0, 0.01));
  scanner->getFWFSettings().winSize_ns =
    2 * scanner->getFWFSettings().binSize_ns;
  scanner->prepareDiscretization(0);
  scanner->allMeasurements = std::make_shared<std::vector<Measurement>>();
  scanner->allMeasurementsMutex = std::make_shared<std::mutex>();
  WaveformTestScene scene;
  SimulatedPulse pulse(glm::dvec3(0), Rotation(), 0.0, 0, 1, 0);
  TracingTestPulse runnable(scanner, scene, pulse);
  runnable.AbstractPulseRunnable::initialize();
  auto part = std::make_shared<ScenePart>();
  part->setId("amplitude-test");
  AABB primitive(glm::dvec3(-1), glm::dvec3(1));
  primitive.part = part;
  primitive.material = std::make_shared<Material>();
  RaySceneIntersection intersection;
  intersection.prim = &primitive;
  intersection.point = glm::dvec3(0, 100, 0);
  intersection.hitDistance = 100;
  std::vector<RaySceneIntersection> intersections{ intersection };
  std::map<double, double> reflections{ { 100.0, 1e-6 } };
  std::vector<std::vector<double>> matrix;
  RandomnessGenerator<double> rng(42), rng2(43);
  glm::dvec3 direction(0, 1, 0);
  double const peak = *std::max_element(scanner->getTimeWave(0).begin(),
                                        scanner->getTimeWave(0).end());
  scanner->setReceivedEnergyMin(1.5 * peak * 1e3);
  for (double amplitude : { 0.0, 1.0, 2.0, 16.0 / std::exp(2.0) }) {
    scanner->setPulseAmplitude(amplitude);
    scanner->setReceivedEnergyMin(amplitude == 0.0 ? 0.0 : 1.5 * peak * 1e3);
    scanner->allMeasurements->clear();
    runnable.digestIntersections(
      matrix, rng, rng2, direction, reflections, intersections);
    if (amplitude < 1.5) {
      REQUIRE(scanner->allMeasurements->empty());
    } else {
      REQUIRE(scanner->allMeasurements->size() == 1);
      REQUIRE(scanner->allMeasurements->front().intensity ==
              Catch::Approx(amplitude * peak * 1e3));
      if (amplitude == 16.0 / std::exp(2.0)) {
        double const x = scanner->getPeakIntensityIndex(0) *
                         scanner->getFWFSettings().binSize_ns /
                         (scanner->getPulseLength_ns() / 7.0);
        REQUIRE(scanner->allMeasurements->front().intensity ==
                Catch::Approx(4.0 * x * x * std::exp(-x) * 1e3));
      }
    }
  }
}
#endif
