//
// Created by miguelyermo on 23/6/21.
//

#include <helios/assetloading/geometryfilter/WavefrontObjCache.h>

WavefrontObjCache&
WavefrontObjCache::getInstance()
{
  static WavefrontObjCache instance;
  return instance;
}

WavefrontObjCache::WavefrontObjCache()
{
  m_capacity = 32;
}

WavefrontObjCache::~WavefrontObjCache()
{
  for (auto& elem : m_map)
    delete elem.second.first;
}

size_t
WavefrontObjCache::size() const
{
  return m_map.size();
}

size_t
WavefrontObjCache::capacity() const
{
  return m_capacity;
}

bool
WavefrontObjCache::empty() const
{
  return m_map.empty();
}

bool
WavefrontObjCache::contains(const std::string& key)
{
  return m_map.find(key) != m_map.end();
}

void
WavefrontObjCache::insert(const std::string& key, WavefrontObj* value)
{
  auto i = m_map.find(key);

  if (i == m_map.end()) {
    if (size() >= m_capacity) {
      deallocate();
    }

    m_list.push_front(key);
    m_map[key] = std::make_pair(value, m_list.begin());
  }
}

value_type
WavefrontObjCache::get(const std::string& key)
{

  // Is the key already stored in the cache?
  auto i = m_map.find(key);
  if (i == m_map.end()) {
    return nullptr;
  }

  // return the value, but first update its place in the most
  // recently used list

  // If the element exists, put its key in the first place in the list and
  // return the element
  auto j = i->second.second;
  if (j != m_list.begin()) {

    // Move the item to the front of the most recently used list
    m_list.erase(j);
    m_list.push_front(key);

    // Update the iterator in the map
    j = m_list.begin();
    const value_type& value = i->second.first;
    m_map[key] = std::make_pair(value, j);

    // Return the element
    return value;
  } else {
    // If the element is already in the first place of the list, return it
    return i->second.first;
  }
}

void
WavefrontObjCache::deallocate()
{
  auto i = --m_list.end();
  delete m_map[*i].first;
  m_map[*i].first = nullptr;
  m_map.erase(*i);
  m_list.erase(i);
}
