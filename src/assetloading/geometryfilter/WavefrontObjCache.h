//
// Created by miguelyermo on 23/6/21.
//

#ifndef HELIOS_WAVEFRONTOBJCACHE_H
#define HELIOS_WAVEFRONTOBJCACHE_H

#include "ScenePart.h"
#include <unordered_map>

/**
 * @brief Class representing a cache for all the .obj loaded models
 */
class WavefrontObjCache {
private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Map where the loaded sceneparts will be stored
   */
  std::unordered_map<std::string, ScenePart *> cache{};

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for a default OBJ Cache
   */
  WavefrontObjCache() = default;
  ~WavefrontObjCache();

public:
  // *** M E T H O D S *** //
  // **********************//
  /**
   * @brief Get the caché object itself. This functions returns always
   * the same instance of the class
   * @return Instance of WavefrontObjCache
   */
  static WavefrontObjCache &getInstance();
  /**
   * @brief Saves an ScenePart using the loaded namefile as a key
   * @param key Key where the ScenePart will be stored
   * @param sp The ScenePart itself
   */
  void saveScenePart(const std::string &key, ScenePart *sp);
  /**
   * @brief Returns a ScenePart stored in key
   * @param key Key where the ScenePart to be loaded is located
   * @return A pointer to a stored ScenePart
   */
  ScenePart *get(const std::string &key);
  /**
   * @brief Checks if a key is already stored in the cache
   * @param key
   * @return
   */
  bool exists(const std::string &key);

  // Avoid accidental copies of this class
  WavefrontObjCache(WavefrontObjCache const &) = delete; // Don't Implement
  WavefrontObjCache
  operator=(WavefrontObjCache const &) = delete; // Don't implement
};

#endif // HELIOS_WAVEFRONTOBJCACHE_H