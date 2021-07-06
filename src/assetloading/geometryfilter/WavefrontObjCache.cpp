//
// Created by miguelyermo on 23/6/21.
//

#include "WavefrontObjCache.h"

WavefrontObjCache &WavefrontObjCache::getInstance() {
  static WavefrontObjCache instance;
  return instance;
}

void WavefrontObjCache::saveScenePart(const std::string &key, WavefrontObj *obj) {
  cache[key] = new WavefrontObj(*obj);
}

WavefrontObj *WavefrontObjCache::get(const std::string &key) {
  return cache.at(key);
}

bool WavefrontObjCache::exists(const std::string &key) {
  return !(cache.find(key) == cache.end());
}

WavefrontObjCache::~WavefrontObjCache() {
  for (auto &object : cache)
    delete object.second;
}
