#include <filems/write/core/FullWaveformWriter.h>

using namespace helios::filems;

// ***   M E T H O D S   *** //
// ************************* //
void
FullWaveformWriter::writeFullWaveform(FullWaveform const& fullWaveform)
{
  sfw->write(fullWaveform);
}
