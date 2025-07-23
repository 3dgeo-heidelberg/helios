#pragma once
#ifndef GLM_TYPE_CASTER_H
#define GLM_TYPE_CASTER_H
#include <glm/glm.hpp>
#include <pybind11/pybind11.h>

namespace pybind11 {
namespace detail {

template<>
struct type_caster<glm::dvec3>
{
public:
  PYBIND11_TYPE_CASTER(glm::dvec3, _("dvec3"));

  bool load(handle src, bool)
  {
    if (!src)
      return false;
    if (!pybind11::isinstance<pybind11::iterable>(src))
      return false;

    pybind11::tuple t = pybind11::cast<pybind11::tuple>(src);
    if (t.size() != 3)
      return false;

    value = glm::dvec3(static_cast<double>(pybind11::cast<double>(t[0])),
                       static_cast<double>(pybind11::cast<double>(t[1])),
                       static_cast<double>(pybind11::cast<double>(t[2])));

    return true;
  }

  static handle cast(const glm::dvec3& src, return_value_policy, handle)
  {
    return pybind11::make_tuple(src.x, src.y, src.z).release();
  }
};

template<>
struct type_caster<glm::dvec2>
{
public:
  PYBIND11_TYPE_CASTER(glm::dvec2, _("dvec2"));

  // Load function to convert from Python tuple/list to glm::dvec2
  bool load(handle src, bool)
  {
    if (!src)
      return false;
    if (!pybind11::isinstance<pybind11::iterable>(src))
      return false;

    pybind11::tuple t = pybind11::cast<pybind11::tuple>(src);
    if (t.size() != 2)
      return false; // glm::dvec2 needs two elements

    value = glm::dvec2(static_cast<double>(pybind11::cast<double>(t[0])),
                       static_cast<double>(pybind11::cast<double>(t[1])));

    return true;
  }

  // Cast function to convert glm::dvec2 to Python tuple
  static handle cast(const glm::dvec2& src, return_value_policy, handle)
  {
    return pybind11::make_tuple(src.x, src.y).release();
  }
};

}
}
#endif
