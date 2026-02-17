#ifndef _BASIC_STATIC_GROVE_H_
#define _BASIC_STATIC_GROVE_H_

#include <helios/adt/grove/StaticGrove.h>

#include <memory>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Basic implementation of a StaticGrove
 *
 * @tparam Tree The type of tree to be handled
 * @see StaticGrove
 */
template<typename Tree>
class BasicStaticGrove : public StaticGrove<Tree>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The trees contained in the grove
   */
  std::vector<std::shared_ptr<Tree>> trees;
  /**
   * @brief Current iteration for while loop friendly methods
   */
  size_t whileIter;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for BasicStaticGrove
   * @param initTreesCapacity The initial capacity for the vector of trees.
   *  When it can be determined reallocations can be avoided leading to better
   *  performance. The initial capacity does not modify the initial size,
   *  which will be zero no matter the capacity. The initial capacity just
   *  allocates enough memory so the first inserted trees (as many as the
   *  initTreesCapacity argument) do not require memory reallocation.
   */
  BasicStaticGrove(size_t const initTreesCapacity = 1)
    : trees(0)
  {
    trees.reserve(initTreesCapacity);
  }
  virtual ~BasicStaticGrove() = default;

  // ***  QUERY METHODS  *** //
  // *********************** //
  /**
   * @see StaticGrove::hasTrees
   */
  bool hasTrees() const override;
  /**
   * @see StaticGrove::getNumTrees
   */
  size_t getNumTrees() const override;

  // ***  MANIPULATION METHODS  *** //
  // ****************************** //
  /**
   * @see StaticGrove::addTree
   */
  void addTree(std::shared_ptr<Tree> tree) override;
  /**
   * @see StaticGrove::removeTree
   */
  void removeTree(size_t const index) override;
  /**
   * @see StaticGrove::removeTrees
   */
  void removeTrees(size_t const startIndex, size_t const endIndex) override;
  /**
   * @see StaticGrove::removeAll
   */
  void removeAll() override;
  /**
   * @see StaticGrove::replaceTree
   */
  void replaceTree(size_t const index, std::shared_ptr<Tree> tree) override;

  // ***  FOR LOOP METHODS  *** //
  // ************************** //
  /**
   * @see StaticGrove::getTreeReference
   */
  Tree& getTreeReference(size_t const index) const override;
  /**
   * @see StaticGrove::getTreeShared
   */
  std::shared_ptr<Tree> getTreeShared(size_t const index) const override;
  /**
   * @see StaticGrove::getTreePointer
   */
  Tree* getTreePointer(size_t const index) const override;

  // ***  WHILE LOOP METHODS  *** //
  // **************************** //
  /**
   * @see StaticGrove::toZeroTree
   */
  void toZeroTree() override;
  /**
   * @see StaticGrove::hasNextTree
   */
  bool hasNextTree() const override;
  /**
   * @see StaticGrove::nextTreeReference
   */
  Tree& nextTreeReference() override;
  /**
   * @see StaticGrove::nextTreeShared
   */
  std::shared_ptr<Tree> nextTreeShared() override;
  /**
   * @see StaticGrove::nextTreePointer
   */
  Tree* nextTreePointer() override;
};

#include <helios/adt/grove/BasicStaticGrove.tpp>
#endif
