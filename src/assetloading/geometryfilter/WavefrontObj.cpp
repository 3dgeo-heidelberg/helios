//
// Created by miguelyermo on 5/7/21.
//

#include <helios/assetloading/geometryfilter/WavefrontObj.h>
#include <helios/scene/primitives/Primitive.h>
#include <iostream>

// *** CONSTRUCTION / DESTRUCTION *** //
// ********************************** //
WavefrontObj::~WavefrontObj()
{
  for (size_t i = 0; i < primitives.size(); i++) {
    delete primitives[i];
    primitives[i] = nullptr;
  }
}
