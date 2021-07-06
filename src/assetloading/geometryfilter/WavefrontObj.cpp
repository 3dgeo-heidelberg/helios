//
// Created by miguelyermo on 5/7/21.
//

#include "WavefrontObj.h"
#include "Primitive.h"

// *** CONSTRUCTION / DESTRUCTION *** //
// ********************************** //
WavefrontObj::WavefrontObj(WavefrontObj & obj)
{
  this->primitives = std::vector<Primitive*>(0);
  Primitive * p;
  for (size_t i = 0; i < obj.primitives.size(); i++)
  {
    p = obj.primitives[i]->clone();
    this->primitives.push_back(p);
  }
}