#pragma once

#include <SAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for k-dimensional trees with surface
 *  area heuristic and greedy selection of best partition axis
 *
 * @see SAHKDTreeFactory
 */
class AxisSAHKDTreeFactory : public SAHKDTreeFactory
{
  using SAHKDTreeFactory::ci;
  using SAHKDTreeFactory::cl;
  using SAHKDTreeFactory::co;
  using SAHKDTreeFactory::lossNodes;

private:
  // *********************** //

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Axis surface area heuristic KDTree factory default constructor
   * @see SAHKDTreeFactory::SAHKDTreeFactory
   */
  AxisSAHKDTreeFactory(size_t const lossNodes = 21,
                       double const ci = 1,
                       double const cl = 1,
                       double const co = 1);
  ~AxisSAHKDTreeFactory() override = default;

  // ***  CLONE  *** //
  // *************** //
  /**
   * @see KDTreeFactory::clone
   */
  KDTreeFactory* clone() const override;
  /**
   * @brief Assign attributes from AxisSAHKDTreeFactory to its clone
   */
  void _clone(KDTreeFactory* kdtf) const override;

  // ***  BUILDING METHODS  *** //
  // ************************** //
  /**
   * @brief Extend SAHKDTreeFactory::defineSplit to handle greedy search of
   *  best split axis
   * @see SAHKDTreeFactory::defineSplit
   */
  void defineSplit(KDTreeNode* node,
                   KDTreeNode* parent,
                   std::vector<Primitive*>& primitives,
                   int const depth) const override;
};
