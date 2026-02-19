#pragma once

#include <filems/util/ZipRecordIO.h>
#include <filems/write/strategies/DirectTrajectoryWriteStrategy.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Like DirectTrajectoryWriteStrategy but zipping the output
 * @see filems::DirectTrajectoryWriteStrategy
 */
class ZipTrajectoryWriteStrategy : public DirectTrajectoryWriteStrategy
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The zipping output stream to do the writing. It must be
   *  associated to the file output stream of the parent
   *  DirectTrajectoryWriteStrategy
   */
  boost::iostreams::filtering_ostream& oa;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for zip trajectory write strategy
   * @see ZipTrajectoryWriteStrategy::oa
   * @see DirectTrajectoryWriteStrategy::DirectTrajectoryWriteStrategy
   */
  ZipTrajectoryWriteStrategy(std::ofstream& ofs,
                             boost::iostreams::filtering_ostream& oa)
    : DirectTrajectoryWriteStrategy(ofs)
    , oa(oa)
  {
  }
  ~ZipTrajectoryWriteStrategy() override = default;

  // ***  WRITE STRATEGY INTERFACE  *** //
  // ********************************** //
  /**
   * @brief Write trajectory to compressed file
   * @see DirectTrajectoryWriteStrategy::write
   */
  void write(Trajectory const& t) override
  {
    writeZippedStringRecord(oa, trajectoryToString(t));
  }
};

}
}
