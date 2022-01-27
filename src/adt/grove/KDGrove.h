#pragma once

#include <BasicDynGrove.h>
#include <KDTreeRaycaster.h>
#include <ScenePart.h>

#include <string>

class KDGrove : public BasicDynGrove<KDTreeRaycaster, ScenePart, std::string>{

};