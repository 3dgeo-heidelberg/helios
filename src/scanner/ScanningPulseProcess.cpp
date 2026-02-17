#include <helios/scanner/Scanner.h>
#include <helios/scanner/ScanningPulseProcess.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningPulseProcess::ScanningPulseProcess(std::shared_ptr<Scanner> scanner)
  : ptf(*(scanner->platform->scene))
  , scanner(scanner)
{
}

// *** GETTERs and SETTERs  *** //
// **************************** //
std::shared_ptr<Scanner>
ScanningPulseProcess::getScanner() const
{
  return scanner;
}
bool
ScanningPulseProcess::isWriteWaveform() const
{
  return scanner->isWriteWaveform();
}

bool
ScanningPulseProcess::isCalcEchowidth() const
{
  return scanner->isCalcEchowidth();
}

std::shared_ptr<std::vector<Measurement>>&
ScanningPulseProcess::getAllMeasurements() const
{
  return scanner->allMeasurements;
}

std::shared_ptr<std::mutex>&
ScanningPulseProcess::getAllMeasurementsMutex() const
{
  return scanner->allMeasurementsMutex;
}

std::shared_ptr<std::vector<Measurement>>&
ScanningPulseProcess::getCycleMeasurements() const
{
  return scanner->cycleMeasurements;
}

std::shared_ptr<std::mutex>&
ScanningPulseProcess::getCycleMeasurementsMutex() const
{
  return scanner->cycleMeasurementsMutex;
}
