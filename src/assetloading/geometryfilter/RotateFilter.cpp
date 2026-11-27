#include <iostream>

#include "RotateFilter.h"
#include <variant>

ScenePart*
RotateFilter::run()
{
  if (primsOut == nullptr) {
    return nullptr;
  }

  if (useLocalRotation) {
    primsOut->mRotation = localRotation.applyTo(primsOut->mRotation);
  } else {
    Rotation rotation = std::get<Rotation>(params["rotation"]);
    primsOut->mRotation = rotation.applyTo(primsOut->mRotation);
  }

  return primsOut;
}
