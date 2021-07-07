//
// Created by miguelyermo on 23/6/21.
//

#include "WavefrontObjCache.h"

WavefrontObjCache &WavefrontObjCache::getInstance() {
  static WavefrontObjCache instance;
  return instance;
}

void WavefrontObjCache::saveScenePart(const std::string &key,
                                      WavefrontObj *obj) {
  // These objs are allocated at WavefrontObjFileLoader::loadObj()
  cache[key] = obj;
}

WavefrontObj *WavefrontObjCache::get(const std::string &key) {
  // No copy is needed because we are only interested in copying the primitives
  return cache.at(key);
}

bool WavefrontObjCache::exists(const std::string &key) {
  return !(cache.find(key) == cache.end());
}

void WavefrontObjCache::clear() {
  for (auto &object : cache)
    delete object.second;
}

WavefrontObjCache::~WavefrontObjCache() { clear(); }
