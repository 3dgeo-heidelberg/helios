#pragma once

#include <BasicDynGrove.h>
#include <GroveKDTreeRaycaster.h>
#include <DynObject.h>

#include <string>

class KDGrove :
    public BasicDynGrove<GroveKDTreeRaycaster, DynObject, std::string>
{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    KDGrove() : BasicDynGrove<GroveKDTreeRaycaster, DynObject, std::string>(){}
    virtual ~KDGrove() = default;
};