/*
This benchmark tests the performance of the findIntersection method of
FullWaveformPulseRunnable.

Unlike the computeSubrays benchmark, this focuses on the ray-scene
intersection path by repeatedly calling findIntersection against a small,
in-memory triangle-grid scene that is finalized (KD-Grove built) once during
setup.

This file also contains a "miss" benchmark variant that exercises the
intersection traversal work when the ray intersects the scene AABB but does
not intersect any primitive. It uses a sparse scene with a large AABB and a
few tiny triangle patches placed in the corners, then shoots a ray through
the empty center.
*/

#include <benchmark/benchmark.h>
#include <logging.hpp>

#include <Scene.h>
#include <Triangle.h>
#include <Vertex.h>

#include <platform/Platform.h>
#include <scanner/SingleScanner.h>
#include <scanner/detector/FullWaveformPulseDetector.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>

#include <cmath>
#include <list>
#include <memory>
#include <stdexcept>
#include <vector>

bool logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
  logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_TIME,
  logging::LOGGING_SHOW_WARN, logging::LOGGING_SHOW_ERR;

namespace {

class BenchmarkFullWaveformPulseRunnable : public FullWaveformPulseRunnable
{
public:
  using FullWaveformPulseRunnable::FullWaveformPulseRunnable;

  std::shared_ptr<RaySceneIntersection> callFindIntersection(
    std::vector<double> const& tMinMax,
    glm::dvec3 const& o,
    glm::dvec3 const& v) const
  {
    return FullWaveformPulseRunnable::findIntersection(tMinMax, o, v);
  }
};

static SimulatedPulse
makePulse(glm::dvec3 const& origin)
{
  return SimulatedPulse(origin,                // origin
                        Rotation(),            // attitude
                        0.0,                   // time_ns
                        0,                     // legIndex
                        1,                     // pulseNumber
                        static_cast<size_t>(0) // deviceIndex
  );
}

static std::shared_ptr<Scene>
makeTriangleGridScene(std::size_t const gridResolution)
{
  auto scene = std::make_shared<Scene>();
  auto part = std::make_shared<ScenePart>();
  part->mId = "benchmarkTriangleGrid";

  // Create a square grid on a plane (Z = -10).
  // Two triangles per cell.
  // The scene is centered around (0,0) in XY to keep the KD build stable.
  double const halfExtent = 50.0;
  double const z = -10.0;

  std::size_t const cells = (gridResolution < 2) ? 1 : (gridResolution - 1);
  double const step = (2.0 * halfExtent) / static_cast<double>(cells);

  part->mPrimitives.reserve(2 * cells * cells);
  scene->primitives.reserve(2 * cells * cells);

  for (std::size_t iy = 0; iy < cells; ++iy) {
    double const y0 = -halfExtent + static_cast<double>(iy) * step;
    double const y1 = y0 + step;
    for (std::size_t ix = 0; ix < cells; ++ix) {
      double const x0 = -halfExtent + static_cast<double>(ix) * step;
      double const x1 = x0 + step;

      Vertex const v00(x0, y0, z);
      Vertex const v10(x1, y0, z);
      Vertex const v01(x0, y1, z);
      Vertex const v11(x1, y1, z);

      auto* t0 = new Triangle(v00, v10, v11);
      t0->part = part;
      part->mPrimitives.push_back(t0);
      scene->primitives.push_back(t0);

      auto* t1 = new Triangle(v00, v11, v01);
      t1->part = part;
      part->mPrimitives.push_back(t1);
      scene->primitives.push_back(t1);
    }
  }

  // Finalize scene (computes bbox, centers vertices, builds KD-Grove +
  // raycaster). This is done once as part of benchmark setup.
  if (!scene->finalizeLoading(false)) {
    throw std::runtime_error("Failed to finalize benchmark scene");
  }

  return scene;
}

static std::shared_ptr<Scene>
makeSparseCornerScene()
{
  auto scene = std::make_shared<Scene>();
  auto part = std::make_shared<ScenePart>();
  part->mId = "benchmarkSparseCorners";

  // Create a few tiny triangle patches in the four corners of a large square.
  // This makes the scene AABB large while leaving empty space in the center.
  // A ray through the center will intersect the AABB but miss all primitives.
  double const halfExtent = 50.0;
  double const patchSize = 1.0;
  double const z = -10.0;

  auto addPatch = [&](double const cx, double const cy) {
    double const x0 = cx;
    double const x1 = cx + patchSize;
    double const y0 = cy;
    double const y1 = cy + patchSize;

    Vertex const v00(x0, y0, z);
    Vertex const v10(x1, y0, z);
    Vertex const v01(x0, y1, z);
    Vertex const v11(x1, y1, z);

    auto* t0 = new Triangle(v00, v10, v11);
    t0->part = part;
    part->mPrimitives.push_back(t0);
    scene->primitives.push_back(t0);

    auto* t1 = new Triangle(v00, v11, v01);
    t1->part = part;
    part->mPrimitives.push_back(t1);
    scene->primitives.push_back(t1);
  };

  part->mPrimitives.reserve(8);
  scene->primitives.reserve(8);

  // Four corners, with a tiny patch in each.
  addPatch(-halfExtent, -halfExtent);
  addPatch(-halfExtent, halfExtent - patchSize);
  addPatch(halfExtent - patchSize, -halfExtent);
  addPatch(halfExtent - patchSize, halfExtent - patchSize);

  if (!scene->finalizeLoading(false)) {
    throw std::runtime_error("Failed to finalize sparse benchmark scene");
  }

  return scene;
}

static std::shared_ptr<SingleScanner>
makeScannerWithScene(std::shared_ptr<Scene> scene)
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
                                    23.0,               // atmosphericVisibility
                                    1064,               // wavelength (nm)
                                    false,              // writeWaveform
                                    false,              // writePulse
                                    false,              // calcEchowidth
                                    false,              // fullWaveNoise
                                    false               // platformNoiseDisabled
    );

  auto platform = std::make_shared<Platform>();
  platform->scene = std::move(scene);
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

struct BenchmarkContext
{
  std::shared_ptr<SingleScanner> scanner;
  BenchmarkFullWaveformPulseRunnable runnable;
  glm::dvec3 rayOrigin;
  glm::dvec3 rayDir;
  std::vector<double> tMinMax;

  explicit BenchmarkContext(std::size_t const gridResolution)
    : scanner(makeScannerWithScene(makeTriangleGridScene(gridResolution)))
    , runnable(scanner, makePulse(glm::dvec3(0.0, 0.0, 20.0)))
    , rayOrigin(0.0, 0.0, 20.0)
    , rayDir(0.0, 0.0, -1.0)
  {
    // Ensure the runnable is wired the same way production code expects.
    runnable.detector = scanner->getDetector(0);

    // Precompute bbox intersection times so the benchmark isolates the
    // raycasting work in Scene::getIntersection.
    tMinMax = scanner->platform->scene->getAABB()->getRayIntersection(rayOrigin,
                                                                      rayDir);

    // If the ray misses the scene bbox, the benchmark would measure the fast
    // early-out path. That is not intended here.
    if (tMinMax.empty()) {
      throw std::runtime_error(
        "Benchmark ray does not intersect the scene AABB");
    }
  }
};

struct MissBenchmarkContext
{
  std::shared_ptr<SingleScanner> scanner;
  BenchmarkFullWaveformPulseRunnable runnable;
  glm::dvec3 rayOrigin;
  glm::dvec3 rayDir;
  std::vector<double> tMinMax;

  MissBenchmarkContext()
    : scanner(makeScannerWithScene(makeSparseCornerScene()))
    , runnable(scanner, makePulse(glm::dvec3(0.0, 0.0, 20.0)))
    , rayOrigin(0.0, 0.0, 20.0)
    , rayDir(0.0, 0.0, -1.0)
  {
    runnable.detector = scanner->getDetector(0);
    tMinMax = scanner->platform->scene->getAABB()->getRayIntersection(rayOrigin,
                                                                      rayDir);
    if (tMinMax.empty()) {
      throw std::runtime_error(
        "Miss benchmark ray does not intersect the scene AABB");
    }
  }
};

static void
fullwaveform_find_intersection_benchmark(benchmark::State& state)
{
  std::size_t const gridResolution = static_cast<std::size_t>(state.range(0));
  BenchmarkContext ctx(gridResolution);

  std::size_t hitCount = 0;

  for (auto _ : state) {
    std::shared_ptr<RaySceneIntersection> itst =
      ctx.runnable.callFindIntersection(ctx.tMinMax, ctx.rayOrigin, ctx.rayDir);

    benchmark::DoNotOptimize(itst);

    if (itst != nullptr) {
      ++hitCount;
      benchmark::DoNotOptimize(itst->hitDistance);
    }
  }

  state.counters["hits"] = benchmark::Counter(static_cast<double>(hitCount));
  state.counters["triangles"] = benchmark::Counter(
    static_cast<double>(2 * (gridResolution < 2 ? 1 : (gridResolution - 1)) *
                        (gridResolution < 2 ? 1 : (gridResolution - 1))));
}

static void
fullwaveform_find_intersection_miss_benchmark(benchmark::State& state)
{
  MissBenchmarkContext ctx;

  std::size_t missCount = 0;

  for (auto _ : state) {
    std::shared_ptr<RaySceneIntersection> itst =
      ctx.runnable.callFindIntersection(ctx.tMinMax, ctx.rayOrigin, ctx.rayDir);

    benchmark::DoNotOptimize(itst);
    if (itst == nullptr) {
      ++missCount;
    } else {
      // In case of unexpected hits, make them visible.
      benchmark::DoNotOptimize(itst->hitDistance);
    }
  }

  state.counters["misses"] = benchmark::Counter(static_cast<double>(missCount));
  state.counters["triangles"] = benchmark::Counter(8.0);
}

} // namespace

BENCHMARK(fullwaveform_find_intersection_benchmark)->Arg(33);
BENCHMARK(fullwaveform_find_intersection_miss_benchmark);
BENCHMARK_MAIN();
