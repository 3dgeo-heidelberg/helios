#include <filems/write/core/FullWaveformWriter.h>
#include <filems/write/comps/SimpleSyncFileFullWaveformWriter.h>
#include <filems/write/comps/ZipSyncFileFullWaveformWriter.h>

#include <string>
#include <sstream>
#include <memory>


using namespace helios::filems;

using std::string;
using std::stringstream;
using std::shared_ptr;
using std::make_shared;

// ***   M E T H O D S   *** //
// ************************* //
void FullWaveformWriter::configure(
    string const &parent,
    string const &prefix,
    bool const computeWaveform
){
    // There is no need to configure output paths if there is no output at all
    if(!computeWaveform) return;

    // Configure output path
    stringstream ss;
    ss.str("");
    ss << parent << prefix;
    if(isZipOutput()) ss << "_fullwave.bin";
    else ss << "_fullwave.txt";
    setOutputFilePath(ss.str());
}

void FullWaveformWriter::writeFullWaveform(
    vector<double> const &fullwave,
    int const fullwaveIndex,
    double const minTime,
    double const maxTime,
    glm::dvec3 const &beamOrigin,
    glm::dvec3 const &beamDir,
    double const gpsTime
){
    sfw->write(
        fullwave,
        fullwaveIndex,
        minTime,
        maxTime,
        beamOrigin,
        beamDir,
        gpsTime
    );
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void FullWaveformWriter::setOutputFilePath(string const &path){
    logging::INFO("fw_path=" + path);
    if(isZipOutput()) sfw = make_shared<ZipSyncFileFullWaveformWriter>(path);
    else sfw = make_shared<SimpleSyncFileFullWaveformWriter>(path);
}
