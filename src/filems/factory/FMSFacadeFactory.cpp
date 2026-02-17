#include <helios/filems/factory/FMSFacadeFactory.h>
#include <helios/filems/write/core/MultiVectorialMeasurementWriter.h>
#include <helios/util/logger/logging.hpp>

#include <boost/filesystem.hpp>

#include <sstream>
#include <string>

namespace fs = boost::filesystem;

// ***  BUILD METHODS  *** //
// *********************** //
std::shared_ptr<helios::filems::FMSFacade>
helios::filems::FMSFacadeFactory::buildFacade(std::string const& outdir,
                                              double const lasScale,
                                              bool const lasOutput,
                                              bool const las10,
                                              bool const zipOutput,
                                              bool const splitByChannel,
                                              Survey& survey,
                                              bool const updateSurvey)
{
  // Try to find a non-existent root directory
  time_t t = std::time(nullptr);
  struct tm* tm = std::localtime(&t);
  char const pathsep = (char)fs::path::preferred_separator;
  std::stringstream ss;
  ss << outdir;
  if (!outdir.empty() && outdir.back() != pathsep)
    ss << pathsep;
  if (!survey.name.empty())
    ss << survey.name << pathsep;
  ss << std::put_time(tm, "%Y-%m-%d_%H-%M-%S") << pathsep;
  std::string rootDir = ss.str();
  bool rootDirExists = fs::exists(rootDir);
  for (size_t i = 0; i < 98 && rootDirExists; ++i) {
    ss.str("");
    ss << outdir << pathsep << survey.name << pathsep
       << std::put_time(tm, "%Y-%m-%d_%H-%M-%S") << "_" << (i + 2) << pathsep;
    rootDir = ss.str();
    rootDirExists = fs::exists(rootDir);
  }
  if (rootDirExists) { // Exception if cannot create rootDir
    ss.str("");
    ss << "The rootDir: \"" << rootDir << "\" does already exist.\n"
       << "Please use a different output directory";
    throw HeliosException(ss.str());
  }
  // Create the root directory
  fs::create_directories(rootDir);
  logging::INFO("Output directory: \"" + rootDir + "\"");

  // Create FMS facade
  std::shared_ptr<FMSFacade> fms = std::make_shared<FMSFacade>();
  // Assign facade to interested survey components, if requested
  if (updateSurvey) {
    survey.scanner->fms = fms;
    for (size_t i = 0; i < survey.scanner->getNumDevices(); ++i) {
      survey.scanner->getDetector(i)->setFMS(fms);
    }
  }

  // Configure write facade
  helios::filems::FMSWriteFacade& fmsWrite = fms->write;
  fmsWrite.splitByChannel = splitByChannel;
  fmsWrite.outDir = outdir;
  fmsWrite.setRootDir(rootDir);

  // Configure measurement writer
  if (survey.scanner->getNumDevices() > 1 && splitByChannel) {
    fmsWrite.setMeasurementWriter(
      std::make_shared<MultiVectorialMeasurementWriter>());
  } else {
    fmsWrite.setMeasurementWriter(
      std::make_shared<VectorialMeasurementWriter>());
  }
  fmsWrite.getMeasurementWriter()->setScanner(survey.scanner);
  fmsWrite.setMeasurementWriterLasOutput(lasOutput);
  fmsWrite.setMeasurementWriterLas10(las10);
  fmsWrite.setMeasurementWriterZipOutput(zipOutput);
  fmsWrite.setMeasurementWriterLasScale(lasScale);

  // Configure trajectory writer
  fmsWrite.setTrajectoryWriter(std::make_shared<TrajectoryWriter>());
  // fmsWrite.setTrajectoryWriterZipOutput(zipOutput); // Zip if requested
  fmsWrite.setTrajectoryWriterZipOutput(false); // Never zip

  // Configure full waveform writer
  fmsWrite.setFullWaveformWriter(
    std::make_shared<VectorialFullWaveformWriter>());
  fmsWrite.setFullWaveformWriterZipOutput(zipOutput);

  // Configure pulse writer
  fmsWrite.setPulseWriter(std::make_shared<VectorialPulseWriter>());
  fmsWrite.setPulseWriterZipOutput(zipOutput);

  // Return
  return fms;
}
