#include <functional>
#include <maths/WaveMaths.h>
#include <memory>
#include <pybind11/pybind11.h>
#include <scanner/Scanner.h>

#if DATA_ANALYTICS >= 2
#define SUBRAY_ADDITIONAL_ARGS , bool&, std::vector<double>&
#define PULSE_RECORDER_ARG , std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#define INTENSITY_RECORDS_ARG                                                  \
  , std::vector<std::vector<double>>& radiometryRecords
#define PULSE_RECORDER_MACRO_ARG , pulseRecorder
#define INTENSITY_RECORDS_MACRO_ARG , radiometryRecords
#else
#define SUBRAY_ADDITIONAL_ARGS
#define PULSE_RECORDER_ARG
#define INTENSITY_RECORDS_ARG
#define PULSE_RECORDER_MACRO_ARG
#define INTENSITY_RECORDS_MACRO_ARG
#endif

class ScannerWrap : public Scanner
{
public:
  ScannerWrap()
    : Scanner("", std::list<int>())
  {
  }

  ScannerWrap(std::string const& id,
              std::list<int> const& pulseFreqs,
              bool writeWaveform = false,
              bool writePulse = false,
              bool calcEchowidth = false,
              bool fullWaveNoise = false,
              bool platformNoiseDisabled = false)
    : Scanner(id,
              pulseFreqs,
              writeWaveform,
              writePulse,
              calcEchowidth,
              fullWaveNoise,
              platformNoiseDisabled)
  {
  }

  ScannerWrap(Scanner& scanner)
    : Scanner(scanner)
  {
  }

  std::shared_ptr<std::mutex> get_all_measurements_mutex()
  {
    return Scanner::allMeasurementsMutex;
  }

  void set_all_measurements_mutex(std::shared_ptr<std::mutex> mutex = nullptr)
  {
    if (!mutex) {
      Scanner::allMeasurementsMutex = std::make_shared<std::mutex>();
    } else {
      Scanner::allMeasurementsMutex = std::move(mutex);
    }
  }

  std::shared_ptr<Scanner> clone() override
  {
    // Create a copy of the current ScannerWrap object
    // This can be a shallow or deep copy depending on your needs
    return std::make_shared<ScannerWrap>(*this);
  }
  void prepareSimulation() override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, prepareSimulation, );
  }

  std::shared_ptr<ScannerSettings> retrieveCurrentSettings(
    size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<ScannerSettings>, Scanner, retrieveCurrentSettings, idx);
  }

  void applySettings(std::shared_ptr<ScannerSettings> settings,
                     size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, applySettings, settings, idx);
  }

  void applySettingsFWF(FWFSettings fwfSettings, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, applySettingsFWF, fwfSettings, idx);
  }

  void doSimStep(unsigned int legIndex,
                 double const currentGpsTime,
                 Scene& scene) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, doSimStep, legIndex, currentGpsTime, scene);
  }

  void calcRaysNumber(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, calcRaysNumber, idx);
  }

  void preparePulse(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, preparePulse, idx);
  }

  double calcAtmosphericAttenuation(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, calcAtmosphericAttenuation, idx);
  }

  Rotation calcAbsoluteBeamAttitude(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(Rotation, Scanner, calcAbsoluteBeamAttitude, idx);
  }

  bool checkMaxNOR(int const nor, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(bool, Scanner, checkMaxNOR, nor, idx);
  }

  void computeSubrays(
    std::function<void(
      Rotation const&,
      std::size_t,
      NoiseSource<double>&,
      std::map<double, double>&,
      std::vector<RaySceneIntersection>& SUBRAY_ADDITIONAL_ARGS)> handleSubray,
    NoiseSource<double>& intersectionHandlingNoiseSource,
    std::map<double, double>& reflections,
    std::vector<RaySceneIntersection>& intersects,
    size_t const idx PULSE_RECORDER_ARG) override
  {
    PYBIND11_OVERLOAD_PURE(void,
                           Scanner,
                           computeSubrays,
                           handleSubray,
                           intersectionHandlingNoiseSource,
                           reflections,
                           intersects,
                           idx PULSE_RECORDER_MACRO_ARG);
  }

  bool initializeFullWaveform(double const minHitDist_m,
                              double const maxHitDist_m,
                              WaveformGrid& grid,
                              size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(bool,
                           Scanner,
                           initializeFullWaveform,
                           minHitDist_m,
                           maxHitDist_m,
                           grid,
                           idx);
  }

  double computeReceivedPower(double const incidenceCosine,
                              double const targetRange,
                              Material const& mat,
                              std::size_t subrayIndex,
                              size_t const idx
                                INTENSITY_RECORDS_ARG) const override
  {
    PYBIND11_OVERLOAD_PURE(double,
                           Scanner,
                           computeReceivedPower,
                           incidenceCosine,
                           targetRange,
                           mat,
                           subrayIndex,
                           idx INTENSITY_RECORDS_MACRO_ARG);
  }

  double computeReceivedPowerWithSigma(double const targetRange,
                                       double const sigma,
                                       std::size_t subrayIndex,
                                       size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double,
                           Scanner,
                           computeReceivedPowerWithSigma,
                           targetRange,
                           sigma,
                           subrayIndex,
                           idx);
  }

  double computePowerRatio(double const incidenceCosine,
                           double const targetRange,
                           Material const& mat,
                           std::size_t subrayIndex,
                           size_t const idx
                             INTENSITY_RECORDS_ARG) const override
  {
    PYBIND11_OVERLOAD_PURE(double,
                           Scanner,
                           computePowerRatio,
                           incidenceCosine,
                           targetRange,
                           mat,
                           subrayIndex,
                           idx INTENSITY_RECORDS_MACRO_ARG);
  }

  double computePowerRatioWithSigma(double const targetRange,
                                    double const sigma,
                                    std::size_t subrayIndex,
                                    size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double,
                           Scanner,
                           computePowerRatioWithSigma,
                           targetRange,
                           sigma,
                           subrayIndex,
                           idx);
  }

  void onLegComplete() override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, onLegComplete, );
  }

  ScanningDevice& getScanningDevice(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(ScanningDevice&, Scanner, getScanningDevice, idx);
  }

  int getCurrentPulseNumber(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(int, Scanner, getCurrentPulseNumber, idx);
  }

  std::size_t getNumRays(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(std::size_t, Scanner, getNumRays, idx);
  }

  void setNumRays(std::size_t numRays, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setNumRays, numRays, idx);
  }

  double getPulseLength_ns(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getPulseLength_ns, idx);
  }

  void setPulseLength_ns(double const pulseLength_ns, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setPulseLength_ns, pulseLength_ns, idx);
  }

  bool lastPulseWasHit(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(bool, Scanner, lastPulseWasHit, idx);
  }

  void setLastPulseWasHit(bool const lastPulseWasHit, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setLastPulseWasHit, lastPulseWasHit, idx);
  }

  double getBeamDivergence(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getBeamDivergence, idx);
  }

  void setBeamDivergence(double const beamDivergence, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setBeamDivergence, beamDivergence, idx);
  }

  double getPulseAmplitude(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getPulseAmplitude, idx);
  }

  void setPulseAmplitude(double const pulseAmplitude, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setPulseAmplitude, pulseAmplitude, idx);
  }

  double getBeamQuality(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getBeamQuality, idx);
  }

  void setBeamQuality(double const beamQuality, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setBeamQuality, beamQuality, idx);
  }

  double getEfficiency(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getEfficiency, idx);
  }

  void setEfficiency(double const efficiency, size_t const idx = 0) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setEfficiency, efficiency, idx);
  }

  double getReceiverDiameter(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getReceiverDiameter, idx);
  }

  void setReceiverDiameter(double const receiverDiameter,
                           size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setReceiverDiameter, receiverDiameter, idx);
  }

  double getVisibility(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getVisibility, idx);
  }

  void setVisibility(double const visibility, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setVisibility, visibility, idx);
  }

  double getWavelength(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getWavelength, idx);
  }

  void setWavelength(double const wavelength, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setWavelength, wavelength, idx);
  }

  double getAtmosphericExtinction(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getAtmosphericExtinction, idx);
  }

  void setAtmosphericExtinction(double const atmosphericExtinction,
                                size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setAtmosphericExtinction, atmosphericExtinction, idx);
  }

  double getBeamWaistRadius(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getBeamWaistRadius, idx);
  }

  void setBeamWaistRadius(double const beamWaistRadius,
                          size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setBeamWaistRadius, beamWaistRadius, idx);
  }

  glm::dvec3 getHeadRelativeEmitterPosition(size_t const idx = 0) const override
  {
    PYBIND11_OVERLOAD_PURE(
      glm::dvec3, Scanner, getHeadRelativeEmitterPosition, idx);
  }

  void setHeadRelativeEmitterPosition(glm::dvec3 const& pos,
                                      size_t const idx = 0) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setHeadRelativeEmitterPosition, pos, idx);
  }

  glm::dvec3& getHeadRelativeEmitterPositionByRef(size_t const idx = 0) override
  {
    PYBIND11_OVERLOAD_PURE(
      glm::dvec3&, Scanner, getHeadRelativeEmitterPositionByRef, idx);
  }

  Rotation getHeadRelativeEmitterAttitude(size_t const idx = 0) const override
  {
    PYBIND11_OVERLOAD_PURE(
      Rotation, Scanner, getHeadRelativeEmitterAttitude, idx);
  }

  void setHeadRelativeEmitterAttitude(Rotation const& attitude,
                                      size_t const idx = 0) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setHeadRelativeEmitterAttitude, attitude, idx);
  }

  Rotation& getHeadRelativeEmitterAttitudeByRef(size_t const idx = 0) override
  {
    PYBIND11_OVERLOAD_PURE(
      Rotation&, Scanner, getHeadRelativeEmitterAttitudeByRef, idx);
  }

  double getBt2(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getBt2, idx);
  }

  void setBt2(double const bt2, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setBt2, bt2, idx);
  }

  double getDr2(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getDr2, idx);
  }

  void setDr2(double const dr2, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setDr2, dr2, idx);
  }

  void setDeviceIndex(size_t const newIdx, size_t const oldIdx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setDeviceIndex, newIdx, oldIdx);
  }

  std::string getDeviceId(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(std::string, Scanner, getDeviceId, idx);
  }

  void setDeviceId(std::string const deviceId, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setDeviceId, deviceId, idx);
  }

  size_t getNumDevices() const override
  {
    PYBIND11_OVERLOAD_PURE(size_t, Scanner, getNumDevices, );
  }

  std::shared_ptr<ScannerHead> getScannerHead(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<ScannerHead>, Scanner, getScannerHead, idx);
  }

  void setScannerHead(std::shared_ptr<ScannerHead> scannerHead,
                      size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setScannerHead, scannerHead, idx);
  }

  std::shared_ptr<AbstractBeamDeflector> getBeamDeflector(
    size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<AbstractBeamDeflector>, Scanner, getBeamDeflector, idx);
  }

  void setBeamDeflector(std::shared_ptr<AbstractBeamDeflector> beamDeflector,
                        size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setBeamDeflector, beamDeflector, idx);
  }

  std::shared_ptr<AbstractDetector> getDetector(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<AbstractDetector>, Scanner, getDetector, idx);
  }

  void setDetector(std::shared_ptr<AbstractDetector> detector,
                   size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setDetector, detector, idx);
  }

  FWFSettings& getFWFSettings(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(FWFSettings&, Scanner, getFWFSettings, idx);
  }

  void setFWFSettings(FWFSettings const& fwfSettings, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setFWFSettings, fwfSettings, idx);
  }

  std::list<int>& getSupportedPulseFreqs_Hz(size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      std::list<int>&, Scanner, getSupportedPulseFreqs_Hz, idx);
  }

  void setSupportedPulseFreqs_Hz(std::list<int>& supportedPulseFreqs_Hz,
                                 size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setSupportedPulseFreqs_Hz, supportedPulseFreqs_Hz, idx);
  }

  int getMaxNOR(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(int, Scanner, getMaxNOR, idx);
  }

  void setMaxNOR(int const maxNOR, size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(void, Scanner, setMaxNOR, maxNOR, idx);
  }

  PulseShape const& getPulseModel(size_t idx) const override
  {
    PYBIND11_OVERLOAD_PURE(PulseShape const&, Scanner, getPulseModel, idx);
  }

  double getReceivedEnergyMin(size_t const idx) const override
  {
    PYBIND11_OVERLOAD_PURE(double, Scanner, getReceivedEnetgyMin, idx);
  }

  void setReceivedEnergyMin(double receivedEnergyMin_W,
                            size_t const idx) override
  {
    PYBIND11_OVERLOAD_PURE(
      void, Scanner, setReceivedEnetgyMin, receivedEnergyMin_W, idx);
  }
};
