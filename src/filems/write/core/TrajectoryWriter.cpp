#include <filems/write/core/TrajectoryWriter.h>
#include <filems/write/comps/ZipSyncFileTrajectoryWriter.h>
#include <filems/write/comps/SimpleSyncFileTrajectoryWriter.h>

#include <sstream>

using namespace helios::filems;

using std::stringstream;
using std::make_shared;

// ***   M E T H O D S   *** //
// ************************* //
void TrajectoryWriter::configure(
    string const &parent,
    string const &prefix
){
    stringstream ss;
    ss.str("");
    ss << parent << prefix << "_trajectory";
    if(isZipOutput()) ss << ".bin";
    else ss << ".txt";
    setOutputFilePath(ss.str());
}
void TrajectoryWriter::writeTrajectory(Trajectory &t){
    sfw->write(t);
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void TrajectoryWriter::setOutputFilePath(string const &path){
    if(zipOutput){
        setSyncFileWriter(
            make_shared<ZipSyncFileTrajectoryWriter>(path)
        );
    }
    else if(lasOutput){
        throw HeliosException(
            "TrajectoryWriter cannot export output in LAS format"
        );
    }
    else{
        setSyncFileWriter(
            make_shared<SimpleSyncFileTrajectoryWriter>(path)
        );
    }
}
