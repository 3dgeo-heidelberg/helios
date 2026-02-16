/*
This benchmark tests the performance of the computeSubrays method of
FullWaveformPulseRunnable. It sets up a mock Scene, SingleScanner and
FullWaveformPulseRunnable. The benchmark does not test the performance of
testIntersection, and instead returns nullptr for all subray intersections.
Scanner settings are largely taken from example RIEGL VZ-400 specifications.
*/

#include <benchmark/benchmark.h>
#include <logging.hpp>

#include <AABB.h>
#include <noise/UniformNoiseSource.h>
#include <platform/Platform.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseDetector.h>

#define private public
#include <scanner/detector/FullWaveformPulseRunnable.h>
#undef private

#include <map>
#include <memory>
#include <vector>

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

class BenchmarkFullWaveformPulseRunnable : public FullWaveformPulseRunnable
{
public:
  using FullWaveformPulseRunnable::computeSubrays;
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;

  std::shared_ptr<RaySceneIntersection> findIntersection(
    std::vector<double> const& tMinMax,
    glm::dvec3 const& o,
    glm::dvec3 const& v) const override
  {
    return nullptr;
  }
};

struct BenchmarkContext
{
  std::shared_ptr<SingleScanner> scanner;
  BenchmarkFullWaveformPulseRunnable runnable;
  UniformNoiseSource<double> intersectionNoise;
  std::map<double, double> reflections;
  std::vector<RaySceneIntersection> intersections;

  explicit BenchmarkContext(int const beamSampleQuality)
    : scanner(makeScanner(beamSampleQuality))
    , runnable(scanner, makePulse())
    , intersectionNoise(0.0, 1.0)
  {
    runnable.detector = scanner->getDetector(0);
    intersections.reserve(
      scanner->getScanningDevice(0).cached_subrayRotation.size());
  }

  std::size_t numSubrays() const
  {
    return scanner->getScanningDevice(0).cached_subrayRotation.size();
  }

private:
  static SimulatedPulse makePulse()
  {
    return SimulatedPulse(glm::dvec3(0, 0, 0),   // origin
                          Rotation(),            // attitude
                          0.0,                   // time_ns
                          0,                     // legIndex
                          1,                     // pulseNumber
                          static_cast<size_t>(0) // deviceIndex
    );
  }

  static std::shared_ptr<SingleScanner> makeScanner(int const beamSampleQuality)
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

    scanner->getFWFSettings(0).beamSampleQuality = beamSampleQuality;

    auto platform = std::make_shared<Platform>();
    auto scene = std::make_shared<BenchmarkScene>();
    scene->setBenchmarkAABB(glm::dvec3(-25.0, -25.0, -25.0),
                            glm::dvec3(-5.0, -5.0, -5.0));
    platform->scene = scene;
    scanner->platform = platform;

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

void
fullwaveform_compute_subrays_benchmark(benchmark::State& state)
{
  BenchmarkContext ctx(3);

  for (auto _ : state) {
    ctx.reflections.clear();
    ctx.intersections.clear();

    ctx.runnable.computeSubrays(
      ctx.intersectionNoise, ctx.reflections, ctx.intersections);

    benchmark::DoNotOptimize(ctx.reflections);
    benchmark::DoNotOptimize(ctx.intersections);
    benchmark::ClobberMemory();
  }

  state.counters["subrays"] =
    benchmark::Counter(static_cast<double>(ctx.numSubrays()));
}

} // namespace

BENCHMARK(fullwaveform_compute_subrays_benchmark);
BENCHMARK_MAIN();
