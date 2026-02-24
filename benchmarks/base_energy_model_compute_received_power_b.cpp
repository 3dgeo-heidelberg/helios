/*
This benchmark tests the performance of BaseEnergyModel::computeReceivedPower.

It sets up a minimal SingleScanner/ScanningDevice context (so all cached device
constants and detector settings are realistic) and repeatedly evaluates the
received power for fixed, synthetic arguments.
*/

#include <benchmark/benchmark.h>
#include <logging.hpp>

#include <maths/model/BaseEnergyModel.h>

#include <AABB.h>
#include <platform/Platform.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseDetector.h>

#include <memory>

bool logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
  logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_TIME,
  logging::LOGGING_SHOW_WARN, logging::LOGGING_SHOW_ERR;

namespace {

class BenchmarkScene : public Scene
{
public:
  void setBenchmarkAABB(glm::dvec3 const& min, glm::dvec3 const& max)
  {
    bbox = std::make_shared<AABB>(min, max);
    bbox_crs = bbox;
  }
};

struct BenchmarkContext
{
  std::shared_ptr<SingleScanner> scanner;
  ScanningDevice const* scanningDevice;
  BaseEnergyModel energyModel;
  Material material;

  BaseReceivedPowerArgs args;

  BenchmarkContext(double const incidenceAngle_rad,
                   double const targetRange_m,
                   double const subrayRadius_m)
    : scanner(makeScanner())
    , scanningDevice(&scanner->getScanningDevice(0))
    , energyModel(*scanningDevice)
    , material(makeLambertMaterial())
    , args(incidenceAngle_rad, targetRange_m, material, subrayRadius_m)
  {
  }

private:
  static Material makeLambertMaterial()
  {
    Material mat;
    mat.name = "benchmark_lambert";
    mat.reflectance = 0.25;

    // Lambert: kd != 0, ks == 0
    mat.kd[0] = 1.0f;
    mat.kd[1] = 1.0f;
    mat.kd[2] = 1.0f;
    mat.ks[0] = 0.0f;
    mat.ks[1] = 0.0f;
    mat.ks[2] = 0.0f;

    // Not used by lambert branch, but keep deterministic values.
    mat.specularity = 0.0;
    mat.specularExponent = 10.0;
    mat.classification = 1;

    return mat;
  }

  static std::shared_ptr<SingleScanner> makeScanner()
  {
    auto scanner =
      std::make_shared<SingleScanner>(0.0003,              // beamDiv_rad
                                      glm::dvec3(0, 0, 0), // beamOrigin
                                      Rotation(),          // beamOrientation
                                      std::list<int>({ 100000 }), // pulse freqs
                                      5.0,                // pulseLength_ns
                                      "benchmarkScanner", // id
                                      4.0,                // averagePower
                                      1.0,                // beamQuality
                                      0.99,               // efficiency
                                      0.15,               // receiverDiameter
                                      23.0,  // atmosphericVisibility
                                      1064,  // wavelength (nm)
                                      false, // writeWaveform
                                      false, // writePulse
                                      false, // calcEchowidth
                                      false, // fullWaveNoise
                                      false  // platformNoiseDisabled
      );

    auto platform = std::make_shared<Platform>();
    auto scene = std::make_shared<BenchmarkScene>();
    scene->setBenchmarkAABB(glm::dvec3(-50.0, -50.0, -50.0),
                            glm::dvec3(50.0, 50.0, 50.0));
    platform->scene = scene;
    scanner->platform = platform;

    // Detector is required as computeReceivedPower reads detector rangeMin.
    scanner->setDetector(
      std::make_shared<FullWaveformPulseDetector>(scanner,
                                                  0.005, // accuracy_m
                                                  0.01   // rangeMin_m
                                                  ),
      0);

    scanner->prepareSimulation(false);
    return scanner;
  }
};

static void
base_energy_model_compute_received_power_benchmark(benchmark::State& state)
{
  // Parameters
  double const incidenceAngle_rad = 0.3;
  double const targetRange_m = static_cast<double>(state.range(0));

  // Subray radius is normally derived from subray divergence; here we pick a
  // stable representative value.
  double const subrayRadius_m = 0.05;

  BenchmarkContext ctx(incidenceAngle_rad, targetRange_m, subrayRadius_m);

  for (auto _ : state) {
    double receivedPower = ctx.energyModel.computeReceivedPower(ctx.args);
    benchmark::DoNotOptimize(receivedPower);
    benchmark::ClobberMemory();
  }

  state.counters["range_m"] = benchmark::Counter(targetRange_m);
}

} // namespace

BENCHMARK(base_energy_model_compute_received_power_benchmark)
  ->Arg(5)
  ->Arg(10)
  ->Arg(25)
  ->Arg(50)
  ->Arg(100);

BENCHMARK_MAIN();
