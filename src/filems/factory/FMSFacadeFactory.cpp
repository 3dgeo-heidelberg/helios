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
    // Create FMS facade
    shared_ptr<FMSFacade> fms = make_shared<FMSFacade>();
    // Assign facade to interested survey components
    survey.scanner->fms = fms;
    survey.scanner->detector->fms = fms;

    // Configure write facade
    FMSWriteFacade &fmsWrite = fms->write;
    fmsWrite.setMeasurementWriter(make_shared<MeasurementWriter>());
    fmsWrite.getMeasurementWriter()->setScanner(survey.scanner);
    fmsWrite.setMeasurementWriterLasOutput(lasOutput);
    fmsWrite.setMeasurementWriterLas10(las10);
    fmsWrite.setMeasurementWriterZipOutput(zipOutput);
    fmsWrite.setMeasurementWriterLasScale(lasScale);

    // Determine root directory for output files, create it if necessary
    time_t t = std::time(nullptr);
    struct tm * tm = std::localtime(&t);
    stringstream ss;
    ss  << outdir << fs::path::preferred_separator
        << survey.name << fs::path::preferred_separator
        << std::put_time(tm, "%Y-%m-%d_%H-%M-%S")
        << fs::path::preferred_separator;
    fmsWrite.rootDir = ss.str();
    fs::create_directories(fmsWrite.rootDir);
    logging::INFO("Output directory: \""+fmsWrite.rootDir+"\"");

    // Configure trajectory facade
    fmsWrite.setTrajectoryWriter(make_shared<TrajectoryWriter>());

    // Configure full waveform facade
    fmsWrite.setFullWaveformWriter(make_shared<FullWaveformWriter>());

    // Return
    return fms;
}
