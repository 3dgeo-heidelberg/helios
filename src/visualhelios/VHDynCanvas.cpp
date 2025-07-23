#ifdef PCL_BINDING

#include <visualhelios/VHDynCanvas.h>

using namespace visualhelios;

// ***  CANVAS METHODS  *** //
// ************************ //
void
VHDynCanvas::postUpdate()
{
  // Base canvas post update
  VHCanvas::postUpdate();

  // After update no updates are needed from canvas side until modified
  setNeedsUpdate(false);
}

#endif
