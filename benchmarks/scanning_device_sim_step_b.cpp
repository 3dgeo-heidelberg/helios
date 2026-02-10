#include <benchmark/benchmark.h>
#include <memory>
#include <scanner/ScanningDevice.h>
#include <scanner/ScanningPulseProcess.h>

namespace {
class DummyBeamDeflector : public AbstractBeamDeflector
{
public:
  DummyBeamDeflector()
    : AbstractBeamDeflector(/*scanAngleMax_rad=*/0.0,
                            /*scanFreqMax_Hz=*/0.0,
                            /*scanFreqMin_Hz=*/0.0)
  {
  }

  std::shared_ptr<AbstractBeamDeflector> clone() override
  {
    return std::make_shared<DummyBeamDeflector>(*this);
  }

  void doSimStep() override {}

  std::string getOpticsType() const override { return "dummy"; }

  bool lastPulseLeftDevice() override { return true; }
};

class BenchmarkScanningDevice : public ScanningDevice
{
public:
  using ScanningDevice::ScanningDevice;

  void setScannerHeadForBenchmark(std::shared_ptr<ScannerHead> sh)
  {
    scannerHead = std::move(sh);
  }

  void setBeamDeflectorForBenchmark(std::shared_ptr<AbstractBeamDeflector> bd)
  {
    beamDeflector = std::move(bd);
  }
};
} // namespace

bool logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
  logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_TIME,
  logging::LOGGING_SHOW_WARN, logging::LOGGING_SHOW_ERR;

static void
scanning_device_sim_step_benchmark(benchmark::State& state)
{
  // Set up a ScanningDevice from example RIEGL VZ-400 specs
  // Some are default values from XmlAssetsLoader
  std::string id = "TestScanningDevice";
  const double beamDiv_rad = 0.0003;
  glm::dvec3 beamOrigin(0, 0, 0);
  Rotation beamOrientation(Directions::right, 0);
  std::list<int> pulseFreqs = { 100000, 300000 };
  const double pulseLength_ns = 5;
  const double averagePower = 4.0;
  const double beamQuality = 1.0;
  const double efficiency = 0.99;
  const double receiverDiameter = 0.15;
  const double atmosphericVisibility = 23.0;
  const int wavelength = 1064;
  // create a dummy range error expression tree (constant 0) for the sake of the
  // benchmark
  auto rangeErrExpr = std::make_shared<UnivarExprTreeNode<double>>();
  rangeErrExpr->symbolType = UnivarExprTreeNode<double>::NUMBER;
  rangeErrExpr->num = 0.0;

  auto scanDevPtr =
    std::make_unique<BenchmarkScanningDevice>(0,
                                              id,
                                              beamDiv_rad,
                                              beamOrigin,
                                              beamOrientation,
                                              pulseFreqs,
                                              pulseLength_ns,
                                              averagePower,
                                              beamQuality,
                                              efficiency,
                                              receiverDiameter,
                                              atmosphericVisibility,
                                              wavelength / 1000000000.0,
                                              rangeErrExpr);
  scanDevPtr->setScannerHeadForBenchmark(
    std::make_shared<ScannerHead>(glm::dvec3(0, 1, 0), 0.0));
  scanDevPtr->setBeamDeflectorForBenchmark(
    std::make_shared<DummyBeamDeflector>());

  ScanningDevice* scanDev = scanDevPtr.get();
  for (auto _ : state) {
    // Simulate a single step of a scanning device
    scanDev->doSimStep(
      0,                              // leg index
      0,                              // current GPS time
      pulseFreqs.front(),             // pulse frequency
      true,                           // is active
      glm::dvec3(0, 0, 0),            // platform absolute mount position
      Rotation(Directions::right, 0), // platform absolute mount attitude
      [&](glm::dvec3& origin, Rotation& attitude) -> void {
        // No noise for this benchmark
      },
      [&](SimulatedPulse const& sp) -> void {
        // No pulse computation for this benchmark
      });
  }
}
BENCHMARK(scanning_device_sim_step_benchmark);
BENCHMARK_MAIN();
