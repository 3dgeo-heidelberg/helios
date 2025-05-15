#pragma once

#include <boost/python/errors.hpp>

namespace pyhelios {

class PyHeliosUtils
{
public:
  /**
   * @brief Translate received index from python, where negative values have
   *  a special meaning (i.e. index -1 means index n-1), to C++ index domain
   * @param _index The index itself
   * @param n The number of elements so n-1 would be the last valid index
   */
  static size_t handlePythonIndex(long _index, size_t n)
  {
    size_t index = (size_t)_index;
    if (_index < 0) {
      index = (size_t)(n + _index);
    }
    if (index >= n) {
      std::stringstream ss;
      ss << "Index " << _index << " out of range";
      PyErr_SetString(PyExc_IndexError, ss.str().c_str());
      boost::python::throw_error_already_set();
    }
    return index;
  }
};

}
