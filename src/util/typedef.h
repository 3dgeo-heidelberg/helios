#pragma once

#include <iostream>
#include <sstream>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "maths/Rotation.h"

#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>

/**
 * @brief Define boost::variant based type for common objects
 */
typedef boost::
  variant<bool, int, float, double, std::string, glm::dvec3, Rotation>
    ObjectT;

/**
 * @brief stringVisitor defines a different string building behavior for
 *  different printable objects
 */
struct stringVisitor : public boost::static_visitor<std::string>
{
  /**
   * @brief String visitor behavior fo bool type
   */
  std::string operator()(bool b) const
  {
    std::stringstream ss;
    ss << b;
    return ss.str();
  }
  /**
   * @brief String visitor behavior for int type
   */
  std::string operator()(int i) const
  {
    std::stringstream ss;
    ss << i;
    return ss.str();
  }
  /**
   * @brief String visitor behavior for float type
   */
  std::string operator()(float f) const
  {
    std::stringstream ss;
    ss << f;
    return ss.str();
  }
  /**
   * @brief String visitor behavior for double type
   */
  std::string operator()(double d) const
  {
    std::stringstream ss;
    ss << d;
    return ss.str();
  }
  /**
   * @brief String visitor behavior for string type
   */
  std::string operator()(std::string const& s) const
  {
    std::stringstream ss;
    ss << s;
    return ss.str();
  }
  /**
   * @brief String visitor behavior for glm::dvec3 type
   */
  std::string operator()(glm::dvec3 v) const
  {
    std::stringstream ss;
    ss << glm::to_string(v);
    return ss.str();
  }
  /**
   * @brief String visitor for Rotation type
   * @see Rotation
   */
  std::string operator()(Rotation r) const
  {
    std::stringstream ss;
    ss << r.getQ0() << '\n'
       << r.getQ1() << '\n'
       << r.getQ2() << '\n'
       << r.getQ3();
    return ss.str();
  }
};

/**
 * @brief Helper struct to get the type name of the ObjectT variants which are
 * retrieved from XML
 */

template<typename T>
struct typenameHelper
{
  static std::string name()
  {
    throw std::runtime_error("Unsupported type for typenameHelper");
  }
};

/** Specializations for supported types */

template<>
struct typenameHelper<bool>
{
  static std::string name() { return "bool"; }
};

template<>
struct typenameHelper<int>
{
  static std::string name() { return "int"; }
};

template<>
struct typenameHelper<float>
{
  static std::string name() { return "float"; }
};

template<>
struct typenameHelper<double>
{
  static std::string name() { return "double"; }
};

template<>
struct typenameHelper<std::string>
{
  static std::string name() { return "string"; }
};

// "C/C++, unfortunately, does not have a sgn function in its standard library,
// however this is defined in the Boost library."
/**
 * @brief Templated sgn function
 */
template<typename T>
int
sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}
