#pragma once

#include <filems/util/ZipRecordIO.h>
#include <filems/write/strategies/DirectMeasurementWriteStrategy.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Like DirectMeasurementWriteStrategy but zipping the output
 * @see filems::DirectMeasurementWriteStrategy
 */
class ZipMeasurementWriteStrategy : public DirectMeasurementWriteStrategy
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The zipping output stream to do the writing. It must be
   *  associated to the file output stream of the parent
   *  DirectMeasurementWriteStrategy
   */
  boost::iostreams::filtering_ostream& oa;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for zip measurement write strategy
   * @see ZipMeasurementWriteStrategy::oa
   * @see DirectMeasurementWriteStrategy::DirectMeasurementWriteStrategy
   */
  ZipMeasurementWriteStrategy(std::ofstream& ofs,
                              boost::iostreams::filtering_ostream& oa)
    : DirectMeasurementWriteStrategy(ofs)
    , oa(oa)
  {
  }
  ~ZipMeasurementWriteStrategy() override = default;

  // ***  WRITE STRATEGY INTERFACE *** //
  // ********************************* //
  /**
   * @brief Write measurement to compressed file
   * @see DirectMeasurementWriteStrategy::write
   */
  void write(Measurement const& m, glm::dvec3 const& shift) override
  {
    writeZippedStringRecord(oa, measurementToString(m, shift));
  }
};

}
}
