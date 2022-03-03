#include <filems/write/TrajectoryWriter.h>

#include <sstream>

using namespace helios::filems;

using std::stringstream;

// ***   M E T H O D S   *** //
// ************************* //
void TrajectoryWriter::configure(
    string const &parent,
    string const &prefix
){
    stringstream ss;
    ss.str("");
    ss << parent << prefix;
    ss << "_trajectory.txt";
    setOutputFilePath(ss.str());
}
void TrajectoryWriter::writeTrajectory(Trajectory &t){
    sfw->write(t);
}
