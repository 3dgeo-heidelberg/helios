#include <benchmark/benchmark.h>

#include <noise/RandomnessGenerator.h>

static void
testing_benchmark(benchmark::State& state)
{
  for (auto _ : state) {
    RandomnessGenerator<double> rg1(1L);
    rg1.computeUniformRealDistribution(0.0, 1.0);
  }
}
BENCHMARK(testing_benchmark);
BENCHMARK_MAIN();
