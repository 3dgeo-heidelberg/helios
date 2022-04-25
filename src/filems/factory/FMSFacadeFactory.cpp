#include <FMSFacadeFactory.h>
#include <logging.hpp>

#include <boost/filesystem.hpp>

#include <string>
#include <sstream>

using namespace helios::filems;

namespace fs = boost::filesystem;

using std::shared_ptr;
using std::make_shared;
using std::string;
using std::stringstream;

// ***  BUILD METHODS  *** //
// *********************** //
shared_ptr<FMSFacade> FMSFacadeFactory::buildFacade(
    string const &outdir,
    double const lasScale,
    bool const lasOutput,
    bool const las10,
    bool const zipOutput,
    Survey &survey
){
    // Determine root directory for output files, create it if necessary
    time_t t = std::time(nullptr);
    struct tm * tm = std::localtime(&t);
    char const pathsep = (char) fs::path::preferred_separator;
    stringstream ss;
    ss  << outdir << pathsep
        << survey.name << pathsep
        << std::put_time(tm, "%Y-%m-%d_%H-%M-%S")
        << pathsep;
    string rootDir = ss.str();
    fs::create_directories(rootDir);
    logging::INFO("Output directory: \""+rootDir+"\"");

    // Create FMS facade
    shared_ptr<FMSFacade> fms = make_shared<FMSFacade>();
    // Assign facade to interested survey components
    survey.scanner->fms = fms;
    survey.scanner->detector->setFMS(fms);

    // Configure write facade
    FMSWriteFacade &fmsWrite = fms->write;
    fmsWrite.setRootDir(rootDir);

    // Configure measurement writer
    fmsWrite.setMeasurementWriter(make_shared<VectorialMeasurementWriter>());
    fmsWrite.getMeasurementWriter()->setScanner(survey.scanner);
    fmsWrite.setMeasurementWriterLasOutput(lasOutput);
    fmsWrite.setMeasurementWriterLas10(las10);
    fmsWrite.setMeasurementWriterZipOutput(zipOutput);
    fmsWrite.setMeasurementWriterLasScale(lasScale);

    // Configure trajectory writer
    fmsWrite.setTrajectoryWriter(make_shared<TrajectoryWriter>());
    //fmsWrite.setTrajectoryWriterZipOutput(zipOutput); // Zip if requested
    fmsWrite.setTrajectoryWriterZipOutput(false); // Never zip

    // Configure full waveform writer
    fmsWrite.setFullWaveformWriter(make_shared<FullWaveformWriter>());
    fmsWrite.setFullWaveformWriterZipOutput(zipOutput);

    // Return
    return fms;
}
