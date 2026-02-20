/*
This benchmark tests the performance of
FullWaveformPulseRunnable::digestIntersections.

It builds a small mock scene + scanner setup and feeds a fixed set of synthetic
reflections/intersections into digestIntersections.

The goal is to measure the full-waveform digestion path, without including ray
casting costs, which are tested via the computeSubrays benchmark.
*/

#include <benchmark/benchmark.h>
#include <logging.hpp>

#include <AABB.h>
#include <noise/RandomnessGenerator.h>
#include <platform/Platform.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseDetector.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>

#include <RaySceneIntersection.h>
#include <ScenePart.h>

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
  using FullWaveformPulseRunnable::digestIntersections;
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;
};

struct BenchmarkContext
{
  std::shared_ptr<SingleScanner> scanner;
  BenchmarkFullWaveformPulseRunnable runnable;

  // Inputs to digestIntersections
  std::vector<std::vector<double>> apMatrix;
  RandomnessGenerator<double> randGen1;
  RandomnessGenerator<double> randGen2;
  glm::dvec3 beamDir;
  std::map<double, double> reflections; // distance_m -> intensity
  std::vector<RaySceneIntersection> intersects;

  // Keep a valid Primitive graph for hitObject/classification logic.
  std::shared_ptr<ScenePart> part;
  std::shared_ptr<Material> material;
  std::unique_ptr<AABB> prim;

  explicit BenchmarkContext(int const beamSampleQuality,
                            std::size_t const numReflections)
    : scanner(makeScanner(beamSampleQuality))
    , runnable(scanner, makePulse())
    , randGen1(1337)
    , randGen2(7331)
    , beamDir(0, 0, 0)
  {
    runnable.detector = scanner->getDetector(0);

    // Force lazy init path to run once outside timing loop.
    AbstractPulseRunnable& base = runnable;
    base.initialize();

    beamDir = runnable.pulse.computeDirection();

    // Warm up RNG distributions to avoid measuring first-use init.
    (void)randGen1.normalDistributionNext();
    (void)randGen2.normalDistributionNext();

    // Create a minimal, valid primitive/material/part so that
    // digestFullWaveform can safely dereference prim->part/material.
    part = std::make_shared<ScenePart>();
    part->setId("benchmark_part");

    material = std::make_shared<Material>();
    material->classification = 1;

    prim = std::make_unique<AABB>(glm::dvec3(-1.0, -1.0, -1.0),
                                  glm::dvec3(1.0, 1.0, 1.0));
    prim->part = part;
    prim->material = material;

    // Synthesize reflections / intersections.
    reflections.clear();
    intersects.clear();
    intersects.reserve(numReflections);

    // Distances are chosen to produce separated waveform peaks.
    // Use strictly increasing distances to avoid map key collisions.
    double const startDist = 10.0;
    double const stepDist = 5.0;
    for (std::size_t i = 0; i < numReflections; ++i) {
      double const d = startDist + stepDist * static_cast<double>(i);
      double const intensity = 5000.0 / (1.0 + static_cast<double>(i));
      reflections.emplace(d, intensity);

      RaySceneIntersection rsi;
      rsi.prim = prim.get();
      rsi.point = glm::dvec3(d, 0.0, 0.0);
      rsi.incidenceAngle = 0.0;
      rsi.hitDistance = d;
      intersects.push_back(rsi);
    }

    // Avoid vector growth affecting timings.
    if (scanner->allMeasurements)
      scanner->allMeasurements->reserve(1024);
    if (scanner->cycleMeasurements)
      scanner->cycleMeasurements->reserve(1024);
  }

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
    scene->setBenchmarkAABB(glm::dvec3(-50.0, -50.0, -50.0),
                            glm::dvec3(50.0, 50.0, 50.0));
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
fullwaveform_digest_intersections_benchmark(benchmark::State& state)
{
  // Parameterize by number of reflections/intersections.
  std::size_t const numReflections = static_cast<std::size_t>(state.range(0));

  BenchmarkContext ctx(/*beamSampleQuality=*/3, numReflections);

  for (auto _ : state) {
    // Keep capture buffers stable.
    if (ctx.scanner->allMeasurements)
      ctx.scanner->allMeasurements->clear();
    if (ctx.scanner->cycleMeasurements)
      ctx.scanner->cycleMeasurements->clear();

    ctx.runnable.digestIntersections(ctx.apMatrix,
                                     ctx.randGen1,
                                     ctx.randGen2,
                                     ctx.beamDir,
                                     ctx.reflections,
                                     ctx.intersects);

    benchmark::DoNotOptimize(ctx.apMatrix);
    benchmark::DoNotOptimize(ctx.reflections);
    benchmark::DoNotOptimize(ctx.intersects);
    benchmark::ClobberMemory();
  }

  state.counters["reflections"] =
    benchmark::Counter(static_cast<double>(ctx.reflections.size()));
}

} // namespace

BENCHMARK(fullwaveform_digest_intersections_benchmark)
  ->Arg(1)
  ->Arg(3)
  ->Arg(8)
  ->Arg(16)
  ->Arg(32);

BENCHMARK_MAIN();
