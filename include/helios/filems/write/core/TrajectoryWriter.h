#pragma once

#include <helios/filems/write/comps/SimpleSyncFileWriter.h>
#include <helios/filems/write/core/HeliosWriter.h>
#include <helios/scanner/Trajectory.h>

#include <boost/filesystem.hpp>

#include <memory>

namespace fs = boost::filesystem;

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of trajectories to generate HELIOS++ output
 *  virtual trajectories
 */
class TrajectoryWriter : public HeliosWriter<Trajectory const&>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for trajectory writer
   */
  TrajectoryWriter() = default;
  virtual ~TrajectoryWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Configure the output path for the trajectory writer
   * @param parent Path to output directory for trajectory files
   * @param prefix Prefix for the name of the output file
   */
  void configure(std::string const& parent, std::string const& prefix);
  /**
   * @brief Write a trajectory point
   * @param t Trajectory point to be written
   */
  void writeTrajectory(Trajectory const& t);
  /**
   * @brief Like filems::TrajectoryWriter::writeTrajectory but faster
   *  because there is no validation
   * @see filems::TrajectoryWriter::writeTrajectory
   */
  inline void writeTrajectoryUnsafe(Trajectory const& t) const
  {
    sfw->write(t);
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set synchronous file writer for trajectory writer
   * @param sfw Synchronous file writer to be used to write trajectory
   * @see filems::TrajectoryWriter::sfw
   */
  inline void setSyncFileWriter(
    std::shared_ptr<SyncFileWriter<Trajectory const&>> sfw)
  {
    this->sfw = sfw;
  }
  /**
   * @brief Get the synchronous file writer used to write trajectories
   * @see filems::TrajectoryWriter::sfw
   */
  inline std::shared_ptr<SyncFileWriter<Trajectory const&>> getSyncFileWriter()
    const
  {
    return sfw;
  }
  /**
   * @brief Set path to output file
   * @param path New path to output file
   */
  void setOutputFilePath(std::string const& path);
  /**
   * @brief Get the path to the output file
   * @return The path to the output file
   * @see filems::TrajectoryWriter::getOutputPath
   */
  inline fs::path getOutputFilePath() const
  {
    return fs::path(getOutputPath());
  }
  /**
   * @see filems::TrajectoryWriter::getOutputFilePath
   */
  inline std::string getOutputPath() const { return sfw->getPath(); }
};

}
}
