#pragma once

#include <helios/filems/facade/FMSWriteFacade.h>
#include <helios/scanner/detector/FullWaveform.h>
#include <helios/util/WriteYielder.h>

#include <cstdlib>
#include <mutex>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Yield a set of full waveforms from single full waveform data so it is
 *  written when buffer size has been reached or, alternatively, when yielder
 *  is directly forced to yield
 */
class FullWaveformYielder : public WriteYielder<FullWaveform>
{
protected:
  // ***  USING  *** //
  // *************** //
  using WriteYielder<FullWaveform>::write;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for full waveform yielder
   * @see WriteYielder::write
   * @see Yielder::bufferSize
   */
  FullWaveformYielder(helios::filems::FMSWriteFacade& write,
                      std::size_t bufferSize = 256)
    : WriteYielder<FullWaveform>(write, bufferSize)
  {
  }
  virtual ~FullWaveformYielder() = default;

  // ***  YIELD METHODS  *** //
  // *********************** //
  /**
   * @brief Write the temporal copy of the full waveforms through the write
   *  facade of the filems
   * @param copy The temporal copy of full waveforms buffer to be digested
   * @see WriteYielder
   */
  void digest(std::vector<FullWaveform>& copy) override
  {
    write.writeFullWaveformsUnsafe(copy);
  }
};
