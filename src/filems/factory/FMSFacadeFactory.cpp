#include <FMSFacadeFactory.h>
#include <filems/write/core/MultiVectorialMeasurementWriter.h>
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
    bool const splitByChannel,
    Survey &survey,
    bool const updateSurvey
){
    // Try to find a non-existent root directory
    time_t t = std::time(nullptr);
    struct tm * tm = std::localtime(&t);
    char const pathsep = (char) fs::path::preferred_separator;
    stringstream ss;
    ss  << outdir << pathsep
        << survey.name << pathsep
        << std::put_time(tm, "%Y-%m-%d_%H-%M-%S")
        << pathsep;
    string rootDir = ss.str();
    bool rootDirExists = fs::exists(rootDir);
    for(size_t i = 0 ; i < 98 && rootDirExists; ++i){
        ss.str("");
        ss  << outdir << pathsep
            << survey.name << pathsep
            << std::put_time(tm, "%Y-%m-%d_%H-%M-%S")
            << "_" << (i+2)
            << pathsep;
        rootDir = ss.str();
        rootDirExists = fs::exists(rootDir);
    }
    if(rootDirExists){ // Exception if cannot create rootDir
        ss.str("");
        ss  << "The rootDir: \"" << rootDir
            << "\" does already exist.\n"
            << "Please use a different output directory";
        throw HeliosException(ss.str());
    }
    // Create the root directory
    fs::create_directories(rootDir);
    logging::INFO("Output directory: \""+rootDir+"\"");

    // Create FMS facade
    shared_ptr<FMSFacade> fms = make_shared<FMSFacade>();
    // Assign facade to interested survey components, if requested
    if(updateSurvey) {
        survey.scanner->fms = fms;
        for (size_t i = 0; i < survey.scanner->getNumDevices(); ++i) {
            survey.scanner->getDetector(i)->setFMS(fms);
        }
    }

    // Configure write facade
    FMSWriteFacade &fmsWrite = fms->write;
    fmsWrite.splitByChannel = splitByChannel;
    fmsWrite.outDir = outdir;
    fmsWrite.setRootDir(rootDir);

    // Configure measurement writer
    if(survey.scanner->getNumDevices() > 1 && splitByChannel){
        fmsWrite.setMeasurementWriter(
            make_shared<MultiVectorialMeasurementWriter>()
        );
    }
    else{
        fmsWrite.setMeasurementWriter(
            make_shared<VectorialMeasurementWriter>()
        );
    }
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
    fmsWrite.setFullWaveformWriter(make_shared<VectorialFullWaveformWriter>());
    fmsWrite.setFullWaveformWriterZipOutput(zipOutput);

    // Configure pulse writer
    fmsWrite.setPulseWriter(make_shared<VectorialPulseWriter>());
    fmsWrite.setPulseWriterZipOutput(zipOutput);

    // Return
    return fms;
}
