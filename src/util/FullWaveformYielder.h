#pragma once

#include <filems/facade/FMSWriteFacade.h>
#include <scanner/detector/FullWaveform.h>
#include <util/WriteYielder.h>

#include <cstdlib>
#include <mutex>
#include <vector>

using helios::filems::FMSWriteFacade;

using std::size_t;
using std::vector;

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
  FullWaveformYielder(FMSWriteFacade& write, size_t bufferSize = 256)
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
  void digest(vector<FullWaveform>& copy) override
  {
    write.writeFullWaveformsUnsafe(copy);
  }
};
