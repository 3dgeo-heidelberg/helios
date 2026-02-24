#pragma once

#include <KDTreeFactoryThreadPool.h>
#include <KDTreeNode.h>
#include <SharedTaskSequencer.h>
#include <SimpleKDTreeFactory.h>
#include <SimpleKDTreeGeometricStrategy.h>

#include <boost/thread.hpp>

#include <optional>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Decorator for any KDTree factory which provides support for multi
 *  thread KDTree building
 *
 * When combining geometry-level and node-level parallel building strategies,
 *  both are assumed to support the same max number of threads.
 *  To better understand this, let \f$\Phi(t)\f$ be the number of currently
 *  available threads for the geometry-level strategy at time \f$t\f$ while
 *  \f$\Psi(t)\f$ denotes the number of currently available threads for the
 *  node-level thread pool at time \f$t\f$.
 * Now, if the maximum number of threads is \f$k\f$, at the beginning it
 *  would be \f$\Phi(t)=k\f$ and \f$\Psi(t)=0\f$. Once all geometry-level
 *  splits have been done, then \f$\Phi(t)=0\f$ and \f$\Psi(t)=k\f$. At the
 *  last depth for geometry-level strategy, always that a split is finished
 *  involved threads will go from geometry-level mode to node-level
 *  thread pool. At any time, threads for geometry-level and node-level are
 *  related by following expresion \f$\Psi(t) = k - \Phi(t)\f$.
 *
 */
class MultiThreadKDTreeFactory : public SimpleKDTreeFactory
{
private:
  // *********************** //

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The SimpleKDTreeFactory or derived to be used to build tree nodes
   */
  std::shared_ptr<SimpleKDTreeFactory> kdtf;
  /**
   * @brief The SimpleKDTreeGeometricStrategy or derived to be used to handle
   *  geometry-level parallelization during multi-thread KDTree building
   */
  std::shared_ptr<SimpleKDTreeGeometricStrategy> gs;
  /**
   * @brief The thread pool to handle concurrency during recursive KDTree
   *  building at node-level
   */
  std::optional<KDTreeFactoryThreadPool> tpNode;
  /**
   * @brief The minimum number of primitives on a given split so a new
   *  task is started to handle them
   */
  std::size_t minTaskPrimitives;
  /**
   * @brief The maximum geometry depth level \f$d^*\f$ as explained in the
   *  MultiThreadKDTreeFactory::buildRecursiveGeometryLevel
   * It is updated accordingly always that
   *  MultiThreadKDTreeFactory::makeFromPrimitivesUnsafe is called
   * @see MultiThreadKDTreeFactory::masters
   */
  int maxGeometryDepth;
  /**
   * @brief The maximum number of jobs (threads/workers) that this factory
   *  is allowed to use.
   */
  std::size_t numJobs;
  /**
   * @brief The number of jobs (threads/workers) that this factory must use
   *  when building upper KDTree nodes (geometry-level parallelization)
   */
  std::size_t geomJobs;
  /**
   * @brief All masters threads (except main thread) are handled by this
   *  shared task sequencer
   *
   * A master thread is any thread that will handle the building of a
   *  sub-KDTree composing the original KDTree. Master threads are required
   *  when using a geometry-level parallelization, so each node at max
   *  geoemtry depth is associated to one master thread.
   *
   * The master node at \f$(d^*, 0)\f$ is not handled by this shared task
   *  sequencer because it is associated to the main thread itself.
   *
   * @see MultiThreadKDTreeFactory::maxGeometryDepth
   */
  std::shared_ptr<SharedTaskSequencer> masters;
  /**
   * @brief How many geometry-level jobs have fully finished during current
   *  KDT building
   * @see MultiThreadKDTreeFactory::increaseFinishedGeomJobsCount
   */
  std::size_t finishedGeomJobs;
  /**
   * @brief Mutex to handle concurrent access to counter of finished
   *  geometry-level jobs
   * @see MultiThreadKDTreeFactory::finishedGeomJobs
   */
  boost::mutex finishedGeomJobsMutex;
  /**
   * @brief True if the factory has not been used before, false otherwise
   */
  bool notUsed;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief MultiThreadKDTreeFactory default constructor
   * @param kdtf The factory to be used to build the KDTree
   */
  MultiThreadKDTreeFactory(
    std::shared_ptr<SimpleKDTreeFactory> const kdtf,
    std::shared_ptr<SimpleKDTreeGeometricStrategy> const gs,
    std::size_t const numJobs = 2,
    std::size_t const geomJobs = 2);
  ~MultiThreadKDTreeFactory() override = default;

  // ***  CLONE  *** //
  // *************** //
  /**
   * @see KDTreeFactory::clone
   */
  KDTreeFactory* clone() const override;
  /**
   *
   * @brief Assign attributes from MultiThreadKDTreeFactory to its clone
   */
  void _clone(KDTreeFactory* kdtf) const override;

  // ***  KDTREE FACTORY METHODS  *** //
  // ******************************** //
  /**
   * @brief Build a KDTree which type depends on current KDTree factory
   *  (MultiThreadKDTreeFactory::kdtf) on a multi thread basis
   * @see MultiThreadKDTreeFactory::kdtf
   * @see KDTreeFactory::makeFromPrimitivesUnsafe
   */
  KDTreeNodeRoot* makeFromPrimitivesUnsafe(
    std::vector<Primitive*>& primitives,
    bool const computeStats = false,
    bool const reportStats = false) override;

protected:
  // ***  BUILDING METHODS  *** //
  // ************************** //
  /**
   * @brief Recursively build a KDTree for given primitives using given
   *  KDTreeFactory (kdtf). The building of upper nodes is delegated to
   *  the MultiThreadKDTreeFactory::buildRecursiveGeometryLevel method,
   *  while the building of middle and lower nodes is assumed by the
   *  MultiThreadKDTreeFactory::buildRecursiveNodeLevel method.
   * @param parent The parent node if any. For root nodes, it must be a
   *  nullptr
   * @param left True if given node is a left child, false otherwise.
   *  If the node is a root node, it should be false. If node is not a root
   *  node and left is true, it means it is a left child. If node is not a
   *  root node and left is false, it means it is a right child
   * @param primitives Primitives to build KDTree splitting them
   * @param depth Current depth at build process. Useful for tracking
   *  recursion level
   * @param index The node index inside current depth. Each node can be
   *  univocally identified by the ordered pair \f$(d, i)\f$ where \f$d\f$
   *  stands for the depth level and \f$i\f$ for the index. The root node
   *  is identified by \f$(0, 0)\f$. Any left child node will be
   *  \f$(d+1, 2i)\f$ and any right child node will be \f$(d+1, 2i+1)\f$,
   *  where \f$d\f$ and \f$i\f$ are the depth and index for the parent node.
   *  In consequence, all left nodes will have an even index while all right
   *  nodes will have an odd one. However, notice that for performance
   *  reasons it could preferable to check the left flag argument, as it is
   *  faster than checking if index is even or odd.
   * @return Built KDTree node
   * @see MultiThreadKDTreeFactory::buildRecursiveGeometryLevel
   * @see MultiThreadKDTreeFactory::buildRecursiveNodeLevel
   * @see MultiThreadKDTreeFactory::minTaskPrimitives
   * @see SimpleKDTreeFactory::buildRecursive
   */
  KDTreeNode* buildRecursive(KDTreeNode* parent,
                             bool const left,
                             std::vector<Primitive*>& primitives,
                             int const depth,
                             int const index) override;
  /**
   * @brief Recursively build a KDTree for given primitives using given
   *  KDTreeFactory (kdtf) in a geometry-level parallelization context.
   * The geometry-level parallelization implies distributing threads among
   *  splits as uniform as possible, while satisfying the constraint that
   *  any split must have at least one associated thread. It is the way to go
   *  for building the upper levels of the KDTree.
   * For the sake of understanding, let \f$P_i\left[a, b\right]\f$ note
   *  the \f$i\f$-th split associated to threads from \f$a\f$-th (inclusive)
   *  to \f$b\f$-th (inclusive). Thus, if \f$d\f$ is said to be
   *  the tree depth and \f$k\f$ is the number of threads,
   *  then it is possible to modellize the behavior of the geometry level
   *  parallel building process as follows:
   *
   * \f[
   * \left\{\begin{array}{ccc}
   *  d = 0 &:& \left\{
   *      P_0[a_0, b_0]
   *  \right\} \\
   *  d = 1 &:& \left\{
   *      P_0\left[a_0, b_0\right],
   *      P_1\left[a_1, b_1\right]
   *  \right\} \\
   *  d = 2 &:& \left\{
   *      P_0\left[a_0, b_0\right],
   *      P_1\left[a_1, b_1\right],
   *      P_2\left[a_2, b_2\right],
   *      P_3\left[a_3, b_3\right]
   *  \right\} \\
   *  \vdots & \vdots & \vdots
   * \end{array}\right.
   * \f]
   *
   * It is worth to mention that \f$a_{i+1} = b_{i}+1\f$.
   *  Now, for generalization purposes let
   *  \f$\alpha=\left\lfloor{\frac{k}{2^d}}\right\rfloor\f$ and
   *  \f$\beta \equiv k \mod 2^d\f$. In consequence, at any depth level
   *  \f$d\f$ it is possible to define \f$\forall i,\; P_i = [a_i, b_i]\f$
   *  where:
   *
   * \f[
   * a_i = \left\{\begin{array}{lll}
   *  i\alpha &,& i < 2^d - \beta \\
   *  i(\alpha+1) - 2^d + \beta &,& i \geq 2^d - \beta
   * \end{array}\right.
   * \f]
   *
   * \f[
   * b_i = \left\{\begin{array}{lll}
   *  \alpha(i+1)-1 &,& i < 2^d - \beta \\
   *  i(\alpha+1)-2^d+\beta+\alpha &,& i \geq 2^d - \beta
   * \end{array}\right.
   * \f]
   *
   * It can be seen that the maximum (also assumed as expected here) number
   *  of splits at a given depth is given by \f$2^d\f$. Therefore, geometry
   *  level building process is guaranteed to be applicable at least while
   *  \f$k \geq 2^d\f$ is satisfied.
   * Moreover, any split can be understood as a set of primitives
   *  \f$P_i = \left\{
   *      p_j \in \mathbb{R}^{n \times 3} :
   *      j \in [\phi \geq 1, \psi \leq m]
   *  \right\}\f$,
   *  so the total number of primitives is \f$m\f$ (which is the cardinality
   *  of \f$P_0\f$ at \f$d=0\f$) and each primitive is
   *  itself a set of \f$n\f$ points in \f$\mathbb{R}^3\f$. Of course, this
   *  is a simplification since different types of primitives are supported
   *  by Helios. However, it is not necessary to get into that level of
   *  detail to understand this algorithm. Just notice that \f$P_i\f$ at
   *  \f$d = x\f$ is distinct that \f$P_i\f$ at \f$d = y\f$ as long as
   *  \f$x \neq y\f$ is satisfied. It is because once a node is splitted,
   *  it is understood as destroyed in this context so it is simply replaced
   *  by its children at next depth level.
   *
   * Finally, the last depth at which geometry level parallelization applies
   *  can be deduced from the fact that the number of threads for
   *  a parallel algorithm is going to satisfy \f$k>1\f$. For then it follows
   *  that \f$\log_2{k} \geq 0\f$. Notice this would stand even for the
   *  sequential case because \f$\log_2{1} = 0\f$. In consequence, it is
   *  known that \f$k \geq 2^d \iff \log_2{k} \geq d\f$, because the
   *  logarithm will not change the sign. But, from aforementioned
   *  inequation, the last depth label can be defined as
   *  \f$d^* = \left\lfloor{\log_2{k}}\right\rfloor\f$
   *
   * @return Built KDTree node
   * @see MultiThreadKDTreeFactory::buildRecursive
   * @see MultiThreadKDTreeFactory::buildRecursiveNodeLevel
   * @see MultiThreadKDTreeFactory::maxGeometryDepth
   */
  KDTreeNode* buildRecursiveGeometryLevel(KDTreeNode* parent,
                                          bool const left,
                                          std::vector<Primitive*>& primitives,
                                          int const depth,
                                          int const index);
  /**
   * @brief Provide an alternative implementation for the building of left
   *  and right children nodes.
   * @see SimpleKDTreeFactory::buildChildrenNodes
   */
  void buildChildrenGeometryLevel(KDTreeNode* node,
                                  KDTreeNode* parent,
                                  std::vector<Primitive*> const& primitives,
                                  int const depth,
                                  int const index,
                                  std::vector<Primitive*>& leftPrimitives,
                                  std::vector<Primitive*>& rightPrimitives,
                                  int const auxiliarThreads);
  /**
   * @brief Recursively build a KDTree for given primitives using given
   *  KDTreeFactory (kdtf) in a node-level parallelization context.
   * The node-level parallelization implies a one thread per node
   *  distribution. While it can be used to build the entire KDTree, at the
   *  upper levels it leads to idle threads. This problem is easy to see
   *  at the first node (root node), because there is only one node and
   *  thus only one thread can be working on it. The same would apply for the
   *  second node if the number of threads is \f$>2\f$, because there would
   *  be only two working threads while the others will remain idle. However,
   *  this is inefficient and can be solved by delegating upper nodes to a
   *  geometry-level parallelization strategy instead of a node-level one.
   *
   * When using a node-level parallelization if the number of primitives for
   *  a given split is \f$\geq\f$ minTaskPrimitives and there are available
   *  threads in the thread pool, a new task will be started to handle node
   *  building in a parallel fashion.
   *
   * @return Built KDTree node
   * @see MultiThreadKDTreeFactory::buildRecursive
   * @see MultiThreadKDTreeFactory::buildRecursiveGeometryLevel
   */
  KDTreeNode* buildRecursiveNodeLevel(KDTreeNode* parent,
                                      bool const left,
                                      std::vector<Primitive*>& primitives,
                                      int const depth,
                                      int const index);
  /**
   * @brief Call the compute KDTree stats method of decorated KDTree factory
   * @see SimpleKDTreeFactory::computeKDTreeStats
   */
  void computeKDTreeStats(KDTreeNodeRoot* root) const override
  {
    kdtf->computeKDTreeStats(root);
  }
  /**
   * @brief Call the report KDTree stats method of decorated KDTree factory
   * @see SimpleKDTreeFactory::reportKDTreeStats
   */
  void reportKDTreeStats(
    KDTreeNodeRoot* root,
    std::vector<Primitive*> const& primitives) const override
  {
    kdtf->reportKDTreeStats(root, primitives);
  }

  // ***  UTIL METHODS  *** //
  // ********************** //
  /**
   * @brief Prepare the MultiThreadKDTreeFactory so it is ready to start
   *  making a new KDTree.
   *
   * This implies handling the number of start pending tasks for node thread
   *  pool so, when geometry-level parallelization is used, no node-level
   *  parallelization occurs before it must (at adequate depth). Besides,
   *  the maximum depth for geometry-level parallelization is properly
   *  handled. Also, the sequencer is initialized with masters threads for
   *  differents nodes if necessary. Finally, the count of finished
   *  geometry-level jobs is set to 0.
   */
  virtual void prepareToMake();
  /**
   * @brief Increase count of finished geometry-level jobs in a thread safe
   *  way.
   * @see MultiThreadKDTreeFactory::finishedGeomJobs
   */
  virtual inline void increaseFinishedGeomJobsCount(std::size_t const amount)
  {
    boost::unique_lock<boost::mutex> lock(finishedGeomJobsMutex);
    finishedGeomJobs += amount;
    lock.unlock();
  }

public:
  // *** GETTERs and SETTERs  *** //
  // **************************** //
  /**
   * @brief Obtain the SimpleKDTreeFactory used to build tree nodes
   * @return SimpleKDTreeFactory used to build tree nodes
   */
  virtual inline std::shared_ptr<SimpleKDTreeFactory> getKdtf() const
  {
    return kdtf;
  }
  /**
   * @brief Obtain the pool size of the thread pool (num jobs)
   * @return Pool size of the thread pool (num jobs)
   */
  virtual inline std::size_t getPoolSize() const
  {
    return tpNode->getPoolSize();
  }
  /**
   * @brief Obtain the number of threads for node-level parallelization
   * @return Number of threads for node-level parallelization
   */
  virtual inline std::size_t getNumJobs() const { return numJobs; }
  /**
   * @brief Obtain the number of threads for geometry-level parallelization
   * @return Number of threads for geometry-level parallelization
   */
  virtual inline std::size_t getGeomJobs() const { return geomJobs; }
  /**
   * @brief Obtain the geometric strategy
   * @return Geometric strategy
   */
  virtual inline std::shared_ptr<SimpleKDTreeGeometricStrategy> getGS() const
  {
    return gs;
  }
};
