#pragma once

#include <helios/surfaceinspector/util/Object.hpp>

#include <vector>

namespace SurfaceInspector {
namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing singular context descriptors
 *
 * Singular context descriptors must be built through singular context computer
 *
 * @see SurfaceInspector::maths::SingularContextComputer
 */
template<typename T>
class SingularContextDescriptors : public SurfaceInspector::util::Object
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The minimum singular value describing the singular context
   * \f$ \sigma^{-}\f$
   */
  T minSingularValue;
  /**
   * @brief The maximum singular value describing the singular context
   * \f$ \sigma^{+} \f$
   */
  T maxSingularValue;
  /**
   * @brief The indices of components from worst fitting vector which
   *  explain the least
   */
  std::vector<std::size_t> worstVectorMinIndices;
  /**
   * @brief The indices of components from worst fitting vector which
   *  explain the most
   */
  std::vector<std::size_t> worstVectorMaxIndices;
  /**
   * @brief The indices of components from best fitting vector which
   *  explain the least
   */
  std::vector<std::size_t> bestVectorMinIndices;
  /**
   * @brief The indices of components from best fitting vector which
   *  explain the most
   */
  std::vector<std::size_t> bestVectorMaxIndices;
  /**
   * @brief The unitary components of the worst fitting vector which
   *  explain the least
   */
  std::vector<T> worstVectorMinComponents;
  /**
   * @brief The unitary components of the worst fitting vector which
   *  explain the most
   */
  std::vector<T> worstVectorMaxComponents;
  /**
   * @brief The unitary components of the best fitting vector which
   *  explain the least
   */
  std::vector<T> bestVectorMinComponents;
  /**
   * @brief The unitary components of the best fitting vector which
   *  explain the most
   */
  std::vector<T> bestVectorMaxComponents;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Singular context descriptors constructor
   */
  SingularContextDescriptors() = default;
  virtual ~SingularContextDescriptors() {};
};
}
}
