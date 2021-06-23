//
// Created by miguelyermo on 23/6/21.
//

#ifndef HELIOS_WAVEFRONTOBJCACHE_H
#define HELIOS_WAVEFRONTOBJCACHE_H

#include "ScenePart.h"
#include <unordered_map>

class WavefrontObjCache
{
private:
  WavefrontObjCache() = default;

  std::unordered_map<std::string, ScenePart*> cache{};

public:
  static WavefrontObjCache & getInstance()
  {
    static WavefrontObjCache instance;
    return instance;
  }

  void addScenePart(const std::string & key, ScenePart* sp)
  {
    cache[key] = new ScenePart(*sp);
  }

public:
  // Avoid accidental copies of this class
  WavefrontObjCache(WavefrontObjCache const&) = delete;  // Don't Implement
  void operator=(WavefrontObjCache const&) = delete;     // Don't implement

};

#endif // HELIOS_WAVEFRONTOBJCACHE_H