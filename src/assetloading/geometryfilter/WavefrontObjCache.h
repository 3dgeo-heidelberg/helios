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

  void saveScenePart(const std::string & key, ScenePart* sp)
  {
    cache[key] = new ScenePart(*sp);
  }

  ScenePart * loadScenePart(const std::string & key)
  {
    // TODO: Hacer copia de miembros, no alojar de nuevo!
    return new ScenePart(*cache[key]);
  }

  ScenePart * get(const std::string & key)
  {
    return cache.at(key);
  }

  bool exists(const std::string & key)
  {
    return !(cache.find(key) == cache.end());
  }

public:
  // Avoid accidental copies of this class
  WavefrontObjCache(WavefrontObjCache const&) = delete;  // Don't Implement
  void operator=(WavefrontObjCache const&) = delete;     // Don't implement

};

#endif // HELIOS_WAVEFRONTOBJCACHE_H