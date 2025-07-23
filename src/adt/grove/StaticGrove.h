#pragma once

#include <GroveTreeWrapper.h>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief A StaticGrove is an abstract class which declares methods to handle a
 *  set of trees.
 *
 * The StaticGrove::hasTrees and StaticGrove::getNumTrees functions allow to
 *  query if the grove has trees and how many.
 *
 * The StaticGrove::addTree, StaticGrove::removeTree,
 *  StaticGrove::removeTrees, StaticGrove::removeAll, StaticGrove::clear,
 *  StaticGrove::replaceTree and StaticGrove::setTree
 *  functions allow to add and remove trees from the grove.
 *
 * The StaticGrove::getNumTrees, StaticGrove::getTreeReference,
 *  StaticGrove::getTreeShared, StaticGrove::getTreePointer and
 *  StaticGrove::operator[] functions provide a friendly way to iterate over
 *  trees through a for loop and to retrieve trees based on their indices.
 *
 * @code
 * ...
 * std::shared_ptr<Tree> tree;
 * size_t const m = grove.getNumTrees();
 * for(size_t i = 0 ; i < m ; ++i){
 *  tree = grove.getTreeShared(i);
 *  tree.doSomething();
 * }
 * ...
 * @endcode
 *
 * The StaticGrove::hasNextTree, StaticGrove::nextTreeReference,
 *  StaticGrove::nextTreeShared and StaticGrove::nextTreePointer functions
 *  provide a friendly way to iterate over trees through a while loop.
 *
 * @code
 * ...
 * std::shared_ptr<Tree> tree;
 * grove.toZeroTree(); // Restart while loop state
 * while(grove.hasNextTree()){
 *  tree = grove.nextTreeShared();
 *  tree.doSomething();
 * }
 * ...
 * @endcode
 *
 * The StaticGrove::begind and StaticGrove::end methods support
 *  for-each loop.
 *
 * @code
 * ...
 * std::shared_ptr<Tree> tree;
 * for(shared_ptr<Tree> tree : grove){
 *  tree.doSomething();
 * }
 * ...
 * @endcode
 *
 * @tparam Tree The type of tree to be handled
 */
template<typename Tree>
class StaticGrove
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~StaticGrove() = default;

  // ***  QUERY METHODS  *** //
  // *********************** //
  /**
   * @brief Check whether the StaticGrove has trees (true) or not (false)
   * @return True if the StaticGrove has trees, false otherwise
   */
  virtual bool hasTrees() const = 0;
  /**
   * @brief Obtain the number of trees handled by the StaticGrove
   * @return Number of trees handled by the StaticGrove
   */
  virtual size_t getNumTrees() const = 0;

  // ***  MANIPULATION METHODS  *** //
  // ****************************** //
  /**
   * @brief Add a tree to be handled by the StaticGrove
   * @param tree Tree to be added to the StaticGrove
   */
  virtual void addTree(std::shared_ptr<Tree> tree) = 0;
  /**
   * @brief Remove a tree from the StaticGrove
   * @param index Index of tree to be removed from StaticGrove
   */
  virtual void removeTree(size_t const index) = 0;
  /**
   * @brief Remove trees inside given interval (start inclusive, end
   *  exclusive)
   * @param startIndex Start index (inclusive) of deletion interval
   * @param endIndex End index (exclusive) of deletion interval
   */
  virtual void removeTrees(size_t const startIndex, size_t const endIndex) = 0;
  /**
   * @brief Remove all trees from the StaticGrove
   * @see StaticGrove::clear
   */
  virtual void removeAll() = 0;
  /**
   * @brief Alias for StaticGrove::removeAll method
   * @see StaticGrove::removeAll
   */
  inline void clear() { removeAll(); }
  /**
   * @brief Replace tree at given index by given tree
   * @param index Index of tree to be replaced
   * @param tree New tree
   */
  virtual void replaceTree(size_t const index, std::shared_ptr<Tree> tree) = 0;
  /**
   * @brief Alias for StaticGrove::replaceTree
   * @see StaticGrove::replaceTree
   */
  inline void setTree(size_t const index, std::shared_ptr<Tree> tree)
  {
    replaceTree(index, tree);
  }

  // ***  FOR LOOP METHODS  *** //
  // ************************** //
  /**
   * @brief Obtain a reference to the tree at given index
   * @param index Index of the tree to be obtained
   * @return Reference to the tree at given index
   */
  virtual Tree& getTreeReference(size_t const index) const = 0;
  /**
   * @brief Obtain a shared pointer to the tree at given index
   * @param index Index of the tree to be obtained
   * @return Shared pointer to the tree at given index
   */
  virtual std::shared_ptr<Tree> getTreeShared(size_t const index) const = 0;
  /**
   * @brief Obtain a pointer to the tree at given index
   * @param index Index of the tree to be obtained
   * @return Pointer to the tree at given index
   */
  virtual Tree* getTreePointer(size_t const index) const = 0;
  /**
   * @brief Obtain a shared pointer to the tree at given index
   * @param index Index of the tree to be obtained
   * @return Shared pointer to the tree at given index
   */
  inline std::shared_ptr<Tree> operator[](size_t const index) const
  {
    return getTreeShared(index);
  }

  // ***  WHILE LOOP METHODS  *** //
  // **************************** //
  /**
   * @brief Restart the internal state of while loop handling.
   */
  virtual void toZeroTree() = 0;
  /**
   * @brief Check whether there are more trees to be iterated through
   *  while loop (true) or not (false)
   * @return True if there are more trees to be iterated through while loop,
   *  false otherwise
   */
  virtual bool hasNextTree() const = 0;
  /**
   * @brief Obtain the reference to next tree and advance internal handling
   *  so next call will return next true, if any
   * @return Reference to next tree
   */
  virtual Tree& nextTreeReference() = 0;
  /**
   * @brief Obtain the shared pointer to next tree and advance internal
   *  handling so next call will return next true, if any
   * @return Shared pointer to next tree
   */
  virtual std::shared_ptr<Tree> nextTreeShared() = 0;
  /**
   * @brief Obtain the pointer to next tree and advance internal
   *  handling so next call will return next true, if any
   * @return Pointer to next tree
   */
  virtual Tree* nextTreePointer() = 0;

  // ***  FOR-EACH LOOP METHODS  *** //
  // ******************************* //
  /**
   * @brief Obtain the first element of a for-each loop over trees
   * @return First element of a for-each loop over trees
   */
  virtual GroveTreeWrapper<Tree> begin()
  {
    return GroveTreeWrapper<Tree>(*this, 0);
  }
  /**
   * @brief Obtaint the last element of a for-each loop over trees
   * @return Last element of a for-each loop over trees
   */
  virtual GroveTreeWrapper<Tree> end()
  {
    return GroveTreeWrapper<Tree>(*this, getNumTrees());
  }
};
