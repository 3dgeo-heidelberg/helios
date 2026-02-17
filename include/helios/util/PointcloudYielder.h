#pragma once

#include <helios/filems/facade/FMSWriteFacade.h>
#include <helios/scanner/Measurement.h>
#include <helios/util/WriteYielder.h>

#include <cstdlib>
#include <mutex>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Yield a point cloud from measurements so it is written when buffer
 *  size has been reached or, alternatively, when yielder is directly forced
 *  to yield
 */
class PointcloudYielder : public WriteYielder<Measurement>
{
protected:
  // ***  USING  *** //
  // *************** //
  using WriteYielder<Measurement>::write;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for point cloud yielder
   * @see WriteYielder::write
   * @see Yielder::bufferSize
   */
  PointcloudYielder(helios::filems::FMSWriteFacade& write,
                    std::size_t bufferSize = 256)
    : WriteYielder<Measurement>(write, bufferSize)
  {
  }
  virtual ~PointcloudYielder() = default;

  // ***  YIELD METHODS  *** //
  // *********************** //
  /**
   * @brief Write the temporal copy of the measurements through the write
   *  facade of the filems
   * @param copy The temporal copy of measurements buffer to be digested
   * @see WriteYielder
   */
  void digest(std::vector<Measurement>& copy) override
  {
    if (!copy.empty())
      write.writeMeasurementsUnsafe(copy);
  }
};
