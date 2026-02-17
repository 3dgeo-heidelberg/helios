//
// Created by miguelyermo on 23/6/21.
//

#ifndef HELIOS_WAVEFRONTOBJCACHE_H
#define HELIOS_WAVEFRONTOBJCACHE_H

#include <helios/assetloading/geometryfilter/WavefrontObj.h>
#include <list>
#include <string>
#include <unordered_map>

typedef std::string key_type;
typedef WavefrontObj* value_type;
typedef std::list<key_type> list_type;
typedef typename list_type::iterator list_iterator;
typedef std::unordered_map<key_type, std::pair<value_type, list_iterator>>
  map_type;
typedef typename map_type::iterator map_iterator;

class WavefrontObjCache
{

public:
  /**
   * @brief Get the number of elements allocated in the cache
   * @return Number of elements allocated in the cache
   */
  size_t size() const;

  /**
   * @brief Get the maximum number of elements that can be stored in the cache
   * @return Number of elements that can be stored in the cache
   */
  size_t capacity() const;

  /**
   * @brief Checks whether the cache is empty
   * @return True if the cache is empty, false otherwise
   */
  bool empty() const;

  /**
   * @brief Checks whether a value is already stored in the cache looking for
   * its key
   * @param key Key of the value to be checked
   * @return True if the value exists, False otherwise
   */
  bool contains(const std::string& key);

  /**
   * @brief Inserts an element in the cache
   * @param key Key of the value used in future searches
   * @param value Element to be stored
   */
  void insert(const std::string& key, WavefrontObj* value);

  /**
   * @brief Returns a cache element by its key, if exists.
   * @param key Key used to look for an element stored in the cache
   * @return The element if found, nullptr otherwise
   */
  WavefrontObj* get(const std::string& key);

  /**
   * @brief Removes the last element in the list to make room to a new one.
   * This function is called only if the cache is full
   */
  void deallocate();

  /**
   * @brief Get an instance of the cache (Singleton Pattern)
   * @return Instance of the cache itself ready to be used.
   */
  static WavefrontObjCache& getInstance();

private:
  map_type m_map;
  list_type m_list;
  size_t m_capacity;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for a default OBJ Cache
   */
  WavefrontObjCache();

  /**
   * @brief Destructor with the responsibility of deallocating all the items
   * stored in the Heap
   */
  ~WavefrontObjCache();
};

#endif // HELIOS_WAVEFRONTOBJCACHE_H
