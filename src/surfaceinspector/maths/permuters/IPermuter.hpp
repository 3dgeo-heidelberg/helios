#pragma once

#include <surfaceinspector/util/Object.hpp>

#include <vector>

namespace SurfaceInspector {
namespace maths {
namespace permuters {

/**
 * @brief Permuter interface provides methods to iterate over
 *  different permutation configurations. It is an interface, which means it
 *  cannot be directly instantiated. It must be implemented by concrete
 *  classes which implement its functions depending on the type of
 *  desired permutations
 *
 * @tparam T Type of data to permute
 */
template<typename T>
class IPermuter : SurfaceInspector::util::Object
{
public:
  // ***  INTERFACE  *** //
  // ******************* //
  /**
   * @brief Set permuter to its initial state
   */
  virtual void start() = 0;
  /**
   * @brief Check if there are permutations left to do. It can be used to
   *  restart a permuter.
   * @return True if there are more permutations to do, false otherwise
   */
  virtual bool hasNext() = 0;
  /**
   * @brief Obtain next permutation
   *
   * The first time next is called, or whenever it is called after invoking
   *  start method, the first permutation is returned. The second time it is
   *  called, the second permutation is returned.
   *
   * @return Vector representing a permutation. Empty vector when there are
   *  no more permutations left.
   * @see IPermuter::start
   * @see IPermuter::get
   */
  virtual std::vector<T> next() = 0;
  /**
   * @brief Obtain the current permutation
   *
   * For instance, in following code, permA will be equal to permB and
   *  permB will be different than permC
   * @code
   * IPermuter<T> permuter = ...;
   * permuter.start();
   * std::vector<T> permA = permuter.next();
   * std::vector<T> permB = permuter.get();
   * std::vector<T> permC = permuter.next();
   * @endcode
   *
   * @return Current permutation
   * @see IPermuter::start
   * @see IPermuter::next
   */
  virtual std::vector<T> get() = 0;
};

}
}
}
