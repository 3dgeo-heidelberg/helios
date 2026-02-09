#pragma once

#include <surfaceinspector/util/draggers/OptimizationDragger.hpp>

namespace SurfaceInspector {
namespace util {
namespace draggers {
/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Min dragger drags the minimum element. The first time next is called
 *  the minimum element is returned, the second time the second minimum element
 *  is returned and so on
 * @tparam E Type of elements to drag
 */
template<typename E>
class MinDragger : public OptimizationDragger<E>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The collection (as vector) to min-drag from
   */
  std::vector<E> x;
  /**
   * @brief The number of elements in the collection x, minus 1
   *
   * It is necessary to handle iterative dragging.
   */
  std::size_t stopSize;
  /**
   * @brief Specify if the min dragger has been initialized (true) or nor
   *  (false).
   *
   * A min dragger is mean to be initialized after calling update method
   *  for the first time.
   */
  bool initialized = false;
  /**
   * @brief The start index
   */
  std::size_t a = 0;
  /**
   * @brief The end index
   */
  std::size_t b = 0;
  /**
   * @brief The current index
   */
  std::size_t c = 0;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a minimum dragger
   * @param x The collection to drag over
   * @see MinDragger::x
   */
  MinDragger(std::vector<E> x)
    : x(x)
    , stopSize(x.size() - 1)
    , initialized(false) {};

  /**
   * @brief Default destructor
   */
  virtual ~MinDragger() {};

protected:
  // ***  INITIALIZATION  *** //
  // ************************ //
  /**
   * @brief Initialize the minimum dragger to the initial status.
   *
   * The initialize method is called on first invocation to update method.
   * @see update
   */
  virtual void initialize();

protected:
  // ***  OPTIMIZATION DRAGGER METHODS  *** //
  // ************************************** //
  /**
   * @brief Pick the minimum element inside \f$[a, b]\f$ indices
   * @return Minimum element inside \f$[a, b]\f$ indices
   * @see SurfaceInspector::util::draggers::OptimizationDragger::pick
   */
  inline E pick() override { return x[c]; }
  /**
   * @brief Update the minimum dragger status which basically means
   *  updating \f$a\f$ and \f$b\f$ values while partially sorting the
   *  collection.
   *
   * First call to update method will invoke initialize.
   * @see initialize
   * @see partialSort
   */
  void update() override;

  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Do a partial sort.
   *
   * Let \f$\alpha = \min(\{x_a, \ldots, x_b\})\f$ and
   *  \f$\beta = \max(\{x_a, \ldots, x_b\})\f$.
   * Now, the \f$x\f$ vector is updated swapping element at \f$a\f$ with
   *  \f$\alpha\f$ element and then swapping element at \f$b\f$ with
   *  \f$\beta\f$ element.
   * @see MinDragger::a
   * @see MinDragger::b
   */
  virtual void partialSort();

public:
  // ***  DRAGGER METHODS  *** //
  // ************************* //
  /**
   * @see SurfaceInspector::util::draggers::IDragger::hasNext
   */
  bool hasNext() override { return c < stopSize; }
};

}
}
}

#include <surfaceinspector/util/draggers/MinDragger.tpp>
