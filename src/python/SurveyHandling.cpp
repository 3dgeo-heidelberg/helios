#include <SurveyHandling.h>

void
checkIntegrateSurveyAndLegs(std::shared_ptr<Survey> survey)
{
  std::shared_ptr<AbstractBeamDeflector> beamDeflector =
    survey->scanner->getBeamDeflector();
  if (!beamDeflector) {
    logging::ERR(
      "checkIntegrateSurveyAndLegs: scanner->getBeamDeflector() returned null");
    throw std::runtime_error("Beam deflector was not properly set in scanner.");
  }

  std::shared_ptr<PolygonMirrorBeamDeflector> polygonDeflector =
    std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(beamDeflector);

  for (auto& leg : survey->legs) {
    int const legId = leg->getSerialId();

    if (polygonDeflector && !leg->mScannerSettings->hasDefaultResolution()) {
      std::stringstream ss;
      ss << "Scanner settings of leg " << legId << " "
         << "have been updated to consider the ratio between max "
         << "effective scan angle and max scan angle.\n"
         << "\tConsequently, the old scanFreq_Hz = "
         << leg->mScannerSettings->scanFreq_Hz << " and "
         << "headRotatePerSec_rad = "
         << leg->mScannerSettings->headRotatePerSec_rad << " have "
         << "been updated.\n";

      leg->mScannerSettings->fitToResolution(
        polygonDeflector->cfg_device_scanAngleMax_rad);

      ss << "\tThe new values are scanFreq_Hz = "
         << leg->mScannerSettings->scanFreq_Hz << " and "
         << "headRotatePerSec_rad = "
         << leg->mScannerSettings->headRotatePerSec_rad << ".";
      logging::INFO(ss.str());
    }
    ScannerSettings const& ss = leg->getScannerSettings();
    if (ss.scanFreq_Hz < beamDeflector->cfg_device_scanFreqMin_Hz) {
      std::stringstream s;
      s << "Scanning frequency for leg " << legId << " is " << ss.scanFreq_Hz
        << "Hz but "
        << "min scanning frequency is set to "
        << beamDeflector->cfg_device_scanFreqMin_Hz << "Hz\n"
        << "The requested scanning frequency cannot be achieved by "
        << "this scanner.\n"
        << "Please update either the requested scanning "
        << "frequency (potentially via the requested scan resolution) "
        << "or the scanner specification.";
      logging::ERR(s.str());
      throw std::runtime_error(s.str());
    }

    if (beamDeflector->cfg_device_scanFreqMax_Hz != 0 &&
        ss.scanFreq_Hz > beamDeflector->cfg_device_scanFreqMax_Hz) {
      std::stringstream s;
      s << "Scanning frequency for leg " << legId << " is " << ss.scanFreq_Hz
        << "Hz but "
        << "max scanning frequency is set to "
        << beamDeflector->cfg_device_scanFreqMax_Hz << "Hz\n"
        << "The requested scanning frequency cannot be achieved by "
        << "this scanner.\n"
        << "Please update either the requested scanning "
        << "frequency (potentially via the requested scan resolution) "
        << "or the scanner specification.";
      logging::ERR(s.str());

      throw std::runtime_error(s.str());
    }
  }
}
