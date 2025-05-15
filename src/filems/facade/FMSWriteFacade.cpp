#include <FMSWriteFacade.h>

#include <boost/filesystem.hpp>

#include <sstream>

using namespace helios::filems;

namespace fs = boost::filesystem;

using std::list;
using std::stringstream;

// ***  FACADE WRITE METHODS  *** //
// ****************************** //
void
FMSWriteFacade::disconnect()
{
  mw->setScanner(nullptr); // Disconnect from scanner
}
void
FMSWriteFacade::configure(string const& prefix,
                          bool const computeWaveform,
                          bool const writePulse,
                          bool const lastLegInStrip)
{
  // Configure measurement output path
  logging::DEBUG("Configuring measurement writer ...");
  getMeasurementWriter()->configure(rootDir, prefix, lastLegInStrip);

  // Configure trajectory output path
  logging::DEBUG("Configuring trajectory writer ...");
  getTrajectoryWriter()->configure(rootDir, prefix);

  // Configure full waveform output path
  logging::DEBUG("Configuring full waveform writer writer ...");
  getFullWaveformWriter()->configure(rootDir, prefix, computeWaveform);

  // Configure pulse output path
  logging::DEBUG("Configuring pulse writer ...");
  getPulseWriter()->configure(rootDir, prefix, writePulse);

  logging::DEBUG("All writers are now configured!");
}

// ***  FACADE MEASUREMENT WRITE METHODS  *** //
// ****************************************** //
void
FMSWriteFacade::validateMeasurementWriter(string const& callerName,
                                          string const& errorMsg) const
{
  // Check measurement writer does exist
  if (mw == nullptr) {
    stringstream ss;
    ss << callerName << " " << errorMsg << " because it does not exist";
    throw HeliosException(ss.str());
  }
}

void
FMSWriteFacade::writeMeasurements(vector<Measurement> const& measurements)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::writeMeasurements",
                            "could not write measurements");
  // Write the measurements
  mw->writeMeasurementsUnsafe(measurements);
}

void
FMSWriteFacade::clearPointcloudFile()
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::clearPointcloudFile",
                            "could not clear pointcloud file");
  // Clear the pointcloud file
  mw->clearPointcloudFile();
}

void
FMSWriteFacade::finishMeasurementWriter()
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::finishMeasurementWriter",
                            "could not finish MeasurementWriter");
  // Check there is a measurement writer to finish
  mw->finish();
}

fs::path
FMSWriteFacade::getMeasurementWriterOutputPath()
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::getMeasurementWriterOutputPath",
                            "could not get MeasurementWriter output file path");
  // Get the output file path
  return mw->getOutputFilePath();
}
void
FMSWriteFacade::setMeasurementWriterOutputPath(std::string path,
                                               const bool lastLegInStrip)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::setMeasurementWriterOutputPath",
                            "could not set MeasurementWriter output file path");
  // Set the output file path
  mw->setOutputFilePath(path, lastLegInStrip);
}

bool
FMSWriteFacade::isMeasurementWriterLasOutput() const
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::isMeasurementWriterLasOutput",
                            "could not get MeasurementWriter LAS output flag");
  // Get the LAS output flag
  return mw->isLasOutput();
}

void
FMSWriteFacade::setMeasurementWriterLasOutput(bool const lasOutput)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::isMeasurementWriterLasOutput",
                            "could not set MeasurementWriter LAS output flag");
  // Set the LAS output flag
  return mw->setLasOutput(lasOutput);
}

bool
FMSWriteFacade::isMeasurementWriterLas10() const
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::isMeasurementWriterLas10",
                            "could not get MeasurementWriter LAS 10 flag");
  // Get the LAS 10 flag
  return mw->isLas10();
}

void
FMSWriteFacade::setMeasurementWriterLas10(bool const las10)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::setMeasurementWriterLas10",
                            "could not set MeasurementWriter LAS 10 flag");
  // Set the LAS 10 flag
  mw->setLas10(las10);
}

bool
FMSWriteFacade::isMeasurementWriterZipOutput() const
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::isMeasurementWriterZipOutput",
                            "could not get MeasurementWriter zip output flag");
  // Get the zip output flag
  return mw->isZipOutput();
}

void
FMSWriteFacade::setMeasurementWriterZipOutput(bool const zipOutput)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::setMeasurementWriterZipOutput",
                            "could not set MeasurementWriter zip output flag");
  // Set the zip output flag
  mw->setZipOutput(zipOutput);
}

double
FMSWriteFacade::getMeasurementWriterLasScale() const
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::getMeasurementWriterLasScale",
                            "could not get MeasurementWriter LAS scale");
  // Obtain LAS scale
  return mw->getLasScale();
}

void
FMSWriteFacade::setMeasurementWriterLasScale(double const lasScale)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::setMeasurementWriterLasScale",
                            "could not set MeasurementWriter LAS scale");
  // Set LAS scale
  return mw->setLasScale(lasScale);
}

// ***  FACADE TRAJECTORY WRITE METHODS  *** //
// ***************************************** //
void
FMSWriteFacade::validateTrajectoryWriter(string const& callerName,
                                         string const& errorMsg) const
{
  // Check trajectory writer does exist
  if (tw == nullptr) {
    stringstream ss;
    ss << callerName << " " << errorMsg << " because it does not exist";
    throw HeliosException(ss.str());
  }
}
void
FMSWriteFacade::writeTrajectory(Trajectory const& t)
{
  // Check it is possible to do the operation
  validateTrajectoryWriter("FMSWriteFacade::writeTrajectory",
                           "could not write trajectory");
  // Write the trajectory
  tw->writeTrajectory(t);
}

void
FMSWriteFacade::finishTrajectoryWriter()
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::finishTrajectoryWriter",
                            "could not finish TrajectoryWriter");
  // Check there is a trajectory writer to finish
  tw->finish();
}

fs::path
FMSWriteFacade::getTrajectoryWriterOutputPath()
{
  // Check it is possible to do the operation
  validateTrajectoryWriter("FMSWriteFacade::getTrajectoryWriterOutputPath",
                           "could not get TrajectoryWriter output file path");
  // Get the output file path
  return tw->getOutputFilePath();
}

void
FMSWriteFacade::setTrajectoryWriterOutputPath(string const& path)
{
  // Check it is possible to do the operation
  validateMeasurementWriter("FMSWriteFacade::setTrajectoryWriterOutputPath",
                            "could not set TrajectoryWriter output file path");
  // Set the output file path
  tw->setOutputFilePath(path);
}

bool
FMSWriteFacade::isTrajectoryWriterZipOutput() const
{
  // Check if it possible to do the operation
  validateTrajectoryWriter("FMSWriteFacade::isTrajectoryWriterZipOutput",
                           "could not get TrajectoryWriter zip output flag");
  // Get the zip output flag
  return tw->isZipOutput();
}

void
FMSWriteFacade::setTrajectoryWriterZipOutput(bool const zipOutput)
{
  // Check it is possible to do the operation
  validateTrajectoryWriter("FMSWriteFacade::setTrajectoryWriterZipOutput",
                           "could not set TrajectoryWriter zip output flag");
  // Set the zip output flag
  tw->setZipOutput(zipOutput);
}

// ***  FACADE FULL WAVEFORM WRITE METHODS  *** //
// ******************************************** //
void
FMSWriteFacade::validateFullWaveformWriter(string const& callerName,
                                           string const& errorMsg) const
{
  // Check full waveform writer does exist
  if (fww == nullptr) {
    stringstream ss;
    ss << callerName << " " << errorMsg << " because it does not exist";
    throw HeliosException(ss.str());
  }
}
void
FMSWriteFacade::writeFullWaveforms(vector<FullWaveform> const& fullWaveforms)
{
  // Check it is possible to do the operation
  validateFullWaveformWriter("FMSWriteFacade::writeFullWaveform",
                             "could not write full waveform");
  // Write the full waveform data
  fww->writeFullWaveformsUnsafe(fullWaveforms);
}

void
FMSWriteFacade::finishFullWaveformWriter()
{
  // Check it is possible to do the operation
  validateFullWaveformWriter("FMSWriteFacade::finishFullWaveformWriter",
                             "could not finish FullWaveformWriter");
  // Check there is a full waveform writer to finish
  fww->finish();
}

bool
FMSWriteFacade::isFullWaveformWriterZipOutput() const
{
  // Check it is possible to do the operation
  validateFullWaveformWriter(
    "FMSWriteFacade::isFullWaveformWriterZipOutput",
    "could not get FullWaveformWriter zip output flag");
  // Get the zip output flag
  return fww->isZipOutput();
}

void
FMSWriteFacade::setFullWaveformWriterZipOutput(bool const zipOutput)
{
  // Check it is possible to do the operation
  validateFullWaveformWriter(
    "FMSWriteFacade::setFullWaveformWriterZipOutput",
    "could not set FullWaveformWriter zip output flag");
  // Set the zip output flag
  fww->setZipOutput(zipOutput);
}

// ***  FACADE PULSE WRITE METHODS  *** //
// ************************************ //
void
FMSWriteFacade::validatePulseWriter(string const& callerName,
                                    string const& errorMsg) const
{
  // Check pulse writer does exist
  if (pw == nullptr) {
    stringstream ss;
    ss << callerName << " " << errorMsg << " because it does not exist";
    throw HeliosException(ss.str());
  }
}

void
FMSWriteFacade::writePulses(vector<PulseRecord> const& pulses)
{
  // Check it is possible to do the operation
  validatePulseWriter("FMSWriteFacade::writePulses", "could not write pulse");
  // Write the pulse data
  pw->writePulsesUnsafe(pulses);
}

void
FMSWriteFacade::finishPulseWriter()
{
  // Check it is possible to do the operation
  validatePulseWriter("FMSWriteFacade::finishPulseWriter",
                      "could not finish PulseWriter");
  // Check there is a pulse writer to finish
  pw->finish();
}

bool
FMSWriteFacade::isPulseWriterZipOutput() const
{
  // Check it is possible to do the operation
  validatePulseWriter("FMSWriteFacade::isPulseWriterZipOutput",
                      "could not get PulseWriter zip output flag");
  // Get the zip output flag
  return pw->isZipOutput();
}

void
FMSWriteFacade::setPulseWriterZipOutput(bool const zipOutput)
{
  // Check it is possible to do the operation
  validatePulseWriter("FMSWriteFacade::setPulseWriterZipOutput",
                      "could not set PulseWriter zip output flag");
  // Set the zip output flag
  pw->setZipOutput(zipOutput);
}
