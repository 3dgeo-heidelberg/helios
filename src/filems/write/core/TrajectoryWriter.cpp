#include <filems/write/comps/SimpleSyncFileTrajectoryWriter.h>
#include <filems/write/core/TrajectoryWriter.h>

#include <sstream>

namespace helios::filems {

// ***   M E T H O D S   *** //
// ************************* //
void
TrajectoryWriter::configure(std::string const& parent,
                            std::string const& prefix)
{
  std::stringstream ss;
  ss.str("");
  ss << parent << prefix << "_trajectory.txt";
  setOutputFilePath(ss.str());
}

void
TrajectoryWriter::writeTrajectory(Trajectory const& t)
{
  sfw->write(t);
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void
TrajectoryWriter::setOutputFilePath(std::string const& path)
{
  if (isLasOutput()) {
    throw HeliosException(
      "TrajectoryWriter cannot export output in LAS format");
  }
  setSyncFileWriter(std::make_shared<SimpleSyncFileTrajectoryWriter>(path));
}

}
