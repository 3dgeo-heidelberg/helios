#include "TranslateFilter.h"

#include <variant>

ScenePart*
TranslateFilter::run()
{
  if (primsOut == nullptr)
    return nullptr;

  // Handle translation itself
  glm::dvec3 offset = localTranslation;
  if (!useLocalTranslation)
    offset = std::get<glm::dvec3>(params["offset"]);
  primsOut->mOrigin = offset;

  // Handle on ground
  if (params.find("onGround") != params.end()) {
    primsOut->forceOnGround = std::get<int>(params["onGround"]);
  }

  // Return
  return primsOut;
}
