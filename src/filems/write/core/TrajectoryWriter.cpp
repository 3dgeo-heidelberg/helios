#include <helios/filems/write/comps/SimpleSyncFileTrajectoryWriter.h>
#include <helios/filems/write/comps/ZipSyncFileTrajectoryWriter.h>
#include <helios/filems/write/core/TrajectoryWriter.h>

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
  ss << parent << prefix << "_trajectory";
  if (isZipOutput())
    ss << ".bin";
  else
    ss << ".txt";
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
  if (zipOutput) {
    setSyncFileWriter(std::make_shared<ZipSyncFileTrajectoryWriter>(path));
  } else if (lasOutput) {
    throw HeliosException(
      "TrajectoryWriter cannot export output in LAS format");
  } else {
    setSyncFileWriter(std::make_shared<SimpleSyncFileTrajectoryWriter>(path));
  }
}

}
