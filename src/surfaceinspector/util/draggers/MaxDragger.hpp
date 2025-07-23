#pragma once

#include <surfaceinspector/util/draggers/MinDragger.hpp>

namespace SurfaceInspector {
namespace util {
namespace draggers {
/**
 * @author Alberto M. Esmoris PEna
 * @version 1.0
 *
 * @brief Max dragger drags the maximum element. The first time next is called
 *  the maximum element is returned, the second time the second maximum element
 *  is returned and so on
 * @tparam E Type of elements to drag
 */
template<typename E>
class MaxDragger : public MinDragger<E>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a maximum dragger
   * @param x The collection to drag over
   * @see SurfaceInspector::util::draggers::MinDragger::MinDragger(vector<E>)
   */
  MaxDragger(std::vector<E> x)
    : SurfaceInspector::util::draggers::MinDragger<E>(x) {};

  /**
   * @brief Default destructor
   */
  virtual ~MaxDragger() {};

protected:
  // ***  OPTIMIZATION DRAGGER METHODS  *** //
  // ************************************** //
  /**
   * @brief Pick the minimum element inside \f$[a, b]\f$ indices
   * @return Minimum element inside \f$[a, b]\f$ indices
   * @see SurfaceInspector::util::draggers::OptimizationDragger::pick
   */
  inline E pick() override { return this->x[this->x.size() - 1 - this->c]; };
};
}
}
}
