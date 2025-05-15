#pragma once

#include <SharedTaskSequencer.h>
#include <SimpleKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing a strategy for geometry-level parallelization of
 *  Simple KDTree building. The geometry-level parallelization is the one to
 *  be applied on upper tree nodes, where multiple threads work on the
 *  building of the same node.
 */
class SimpleKDTreeGeometricStrategy
{
  // ***  FRIENDS  *** //
  // ***************** //
  friend class MultiThreadKDTreeFactory;

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The simple KDTree factory to which geometric strategy shall be
   *  applied
   */
  SimpleKDTreeFactory& kdtf;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief SimpleKDTreeGeometricStrategy default constructor.
   *
   * @see SimpleKDTreeGeometricStrategy::kdtf
   */
  SimpleKDTreeGeometricStrategy(SimpleKDTreeFactory& kdtf)
    : kdtf(kdtf)
  {
  }
  virtual ~SimpleKDTreeGeometricStrategy() = default;

  // ***  CLONE  *** //
  // *************** //
  /**
   * @brief Create a clone of the SimpleKDTreeGeometricStrategy
   * @param kdtf The KDTreeFactory to be referenced by the clone
   * @return Clone of the SimpleKDTreeGeometricStrategy
   */
  virtual SimpleKDTreeGeometricStrategy* clone(SimpleKDTreeFactory* kdtf) const;

protected:
  // ***  GEOMETRY LEVEL BUILDING  *** //
  // ********************************* //
  /**
   * @brief Geometry-level parallel version of the
   *  SimpleKDTreeFactory::defineSplit function
   * @param assignedThreads How many threads can be used to parallelize
   *  computations
   * @see SimpleKDTreeFactory::defineSplit
   */
  virtual void GEOM_defineSplit(KDTreeNode* node,
                                KDTreeNode* parent,
                                vector<Primitive*>& primitives,
                                int const depth,
                                int const assignedThreads) const;
  /**
   * @brief Geometry-level parallel version of the
   *  SimpleKDTreeFactory::computeNodeBoundaries function
   * @param assignedThreads How many threads can be used to parallelize
   *  computations
   * @see SimpleKDTreeFactory::computeNodeBoundaries
   */
  virtual void GEOM_computeNodeBoundaries(KDTreeNode* node,
                                          KDTreeNode* parent,
                                          bool const left,
                                          vector<Primitive*> const& primitives,
                                          int assignedThreads);
  /**
   * @brief Geometry-level parallel version of the
   *  SimpleKDTreeFactory::populateSplits function
   * @param assignedThreads How many threads can be used to parallelize
   *  computations
   * @see SimpleKDTreeFactory::populateSplits
   */
  virtual void GEOM_populateSplits(vector<Primitive*> const& primitives,
                                   int const splitAxis,
                                   double const splitPos,
                                   vector<Primitive*>& leftPrimitives,
                                   vector<Primitive*>& rightPrimitives,
                                   int assignedThreads) const;
  /**
   * @brief Geometry-level parallel version of the
   *  SimpleKDTreeFactory::buildChildrenNodes function
   *
   * It is expected that this function is called at any depth before the
   *  last geometry-level depth. In consequence, the left node is delegated
   *  upon current thread while right node is delegated upon a new created
   *  thread for such purpose.
   *
   * @see SimpleKDTreeFactory::buildChildrenNodes
   * @return True if a new master thread has been created, false otherwise.
   *  This method will return when all pending work related to recursively
   *  building of children nodes and all its descendants has been completed
   */
  virtual void GEOM_buildChildrenNodes(
    KDTreeNode* node,
    KDTreeNode* parent,
    vector<Primitive*> const& primitives,
    int const depth,
    int const index,
    vector<Primitive*>& leftPrimitives,
    vector<Primitive*>& rightPrimitives,
    std::shared_ptr<SharedTaskSequencer> masters);
};
