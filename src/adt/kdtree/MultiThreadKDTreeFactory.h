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
        //ar &tp; // No need to serialize because default built one is used
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The SimpleKDTreeFactory or derived to be used to build partial
     *  trees
     */
    shared_ptr<SimpleKDTreeFactory> kdtf;
    /**
     * @brief The thread pool to handle concurrency during recursive KDTree
     *  building
     */
    KDTreeFactoryThreadPool tp;

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
    /**
     * @brief Call the lighten method of decorated KDTree factory
     * @see KDTreeFactory::lighten
     */
    inline void lighten(KDTreeNodeRoot *root) override {kdtf->lighten(root);}

    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Recursively build a KDTree for given primitives
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
};