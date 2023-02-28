#pragma once

#include <SimpleKDTreeFactory.h>
#include <KDTreeFactoryThreadPool.h>
#include <boost/thread.hpp>

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Decorator for any KDTree factory which provides support for multi
 *  thread KDTree building
 */
class MultiThreadKDTreeFactory : public SimpleKDTreeFactory{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a multi thread KDTree factory to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the multi thread K dimensional tree
     *  factory
     */
    template <class Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            MultiThreadKDTreeFactory,
            SimpleKDTreeFactory
        >();

        ar &boost::serialization::base_object<SimpleKDTreeFactory>(*this);
        ar &kdtf;
        ar &minTaskPrimitives;
        //ar &tp; // No need to serialize because default built one is used
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The SimpleKDTreeFactory or derived to be used to build tree nodes
     */
    shared_ptr<SimpleKDTreeFactory> kdtf;
    /**
     * @brief The thread pool to handle concurrency during recursive KDTree
     *  building
     */
    KDTreeFactoryThreadPool tp;
    /**
     * @brief The minimum number of primitives on a given split so a new
     *  task is started to handle them
     */
    size_t minTaskPrimitives;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief MultiThreadKDTreeFactory default constructor
     * @param kdtf The factory to be used to build the KDTree
     */
    MultiThreadKDTreeFactory(
        shared_ptr<SimpleKDTreeFactory> const kdtf,
        size_t const numJobs=2
    );
    virtual ~MultiThreadKDTreeFactory() = default;

    // ***  KDTREE FACTORY METHODS  *** //
    // ******************************** //
    /**
     * @brief Build a KDTree which type depends on current KDTree factory
     *  (MultiThreadKDTreeFactory::kdtf) on a multi thread basis
     * @see MultiThreadKDTreeFactory::kdtf
     * @see KDTreeFactory::makeFromPrimitivesUnsafe
     */
    KDTreeNodeRoot * makeFromPrimitivesUnsafe(
        vector<Primitive *> &primitives
    ) override;

protected:
    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Recursively build a KDTree for given primitives using given
     *  KDTreeFactory (kdtf). When the number of primitives for a given split
     *  is \f$\geq\f$ minTaskPrimitives and there are available threads
     *  in the thread pool, a new task will be started to handle node building
     *  in a parallel fashion.
     * @param parent The parent node if any. For root nodes, it must be a
     *  nullptr
     * @param left True if given node is a left child, false otherwise.
     *  If the node is a root node, it should be false. If node is not a root
     *  node and left is true, it means it is a left child. If node is not a
     *  root node and left is false, it means it is a right child
     * @param primitives Primitives to build KDTree splitting them
     * @param depth Current depth at build process. Useful for tracking
     *  recursion level
     * @return Built KDTree node
     * @see MultiThreadKDTreeFactory::minTaskPrimitives
     * @see SimpleKDTreeFactory::buildRecursive
     */
    KDTreeNode * buildRecursive(
        KDTreeNode *parent,
        bool const left,
        vector<Primitive*> &primitives,
        int const depth
    ) override;
    /**
     * @brief Call the compute KDTree stats method of decorated KDTree factory
     * @see SimpleKDTreeFactory::computeKDTreeStats
     */
    void computeKDTreeStats(KDTreeNodeRoot *root) const override
    {kdtf->computeKDTreeStats(root);}
    /**
     * @brief Call the report KDTree stats method of decorated KDTree factory
     * @see SimpleKDTreeFactory::reportKDTreeStats
     */
    void reportKDTreeStats(
        KDTreeNodeRoot *root,
        vector<Primitive *> const &primitives
    ) const override
    {kdtf->reportKDTreeStats(root, primitives);}

public:
    // *** GETTERs and SETTERs  *** //
    // **************************** //
    /**
     * @brief Obtain the SimpleKDTreeFactory used to build tree nodes
     * @return SimpleKDTreeFactory used to build tree nodes
     */
    virtual inline shared_ptr<SimpleKDTreeFactory> getKdtf() const
    {return kdtf;}
    /**
     * @brief Obtain the pool size of the thread pool (num jobs)
     * @return Pool size of the thread pool (num jobs)
     */
    virtual inline size_t getPoolSize() const
    {return tp.getPoolSize();}
};