#pragma once

#include <KDTreeBuildType.h>
#include <MDThreadPool.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a thread pool to deal with multi thread KDTree
 *  building process
 * @see ThreadPool
 * @see MDThreadPool
 * @see MultiThreadKDTreeFactory
 * @see KDTreeBuildType
 */
class KDTreeFactoryThreadPool
  : public MDThreadPool<KDTreeBuildType,
                        KDTreeNode*,
                        bool const,
                        vector<Primitive*>&,
                        int const,
                        int const>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief KDTree factory thread pool constructor
   * @see ThreadPool::pool_size
   */
  explicit KDTreeFactoryThreadPool(std::size_t const _pool_size)
    : MDThreadPool<KDTreeBuildType,
                   KDTreeNode*,
                   bool const,
                   vector<Primitive*>&,
                   int const,
                   int const>(_pool_size)
  {
  }
  /**
   * @brief KDTree factory thread pool constructor which uses concurrent
   *  threads supported by the system as the default pool size
   * @see KDTreeFactoryThreadPool::KDTreeFactoryThreadPool(std::size_t const)
   */
  explicit KDTreeFactoryThreadPool()
    : KDTreeFactoryThreadPool(std::thread::hardware_concurrency())
  {
  }
  virtual ~KDTreeFactoryThreadPool()
  {
    if (!finished) {
      threads_.interrupt_all();
      finished = false;
    }
  }

protected:
  /**
   * @brief Do a recursive build of KDTree node task
   * @param task Recursive build of KDTree node task
   * @param data It will be deleted after computing the task
   * @see ThreadPool::do_task
   */
  inline void do_md_task(
    boost::function<
      void(KDTreeNode*, bool const, vector<Primitive*>&, int const, int const)>&
      task,
    KDTreeBuildType* data) override
  {
    task(data->parent, data->left, data->primitives, data->depth, data->index);
    delete data;
  }
};
