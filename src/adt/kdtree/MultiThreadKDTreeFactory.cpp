#include <MultiThreadKDTreeFactory.h>
#include <KDTreeBuildType.h>
#include <surfaceinspector/maths/Scalar.hpp>

using SurfaceInspector::maths::Scalar;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiThreadKDTreeFactory::MultiThreadKDTreeFactory(
    shared_ptr<SimpleKDTreeFactory> const kdtf,
    size_t const numJobs
) :
    SimpleKDTreeFactory(),
    kdtf(kdtf),
    tpGeom(numJobs),
    tpNode(0),
    minTaskPrimitives(32)
{
    /*
     * See SimpleKDTreeFactory constructor implementation to understand why
     *  it is safe to call virtual function here.
     */
    _buildRecursive =
        [&](
            KDTreeNode *parent,
            bool const left,
            vector<Primitive *> &primitives,
            int const depth,
            int const index
        ) -> KDTreeNode * {
            return this->buildRecursive(
                parent, left, primitives, depth, index
            );
        }
    ;
    kdtf->_buildRecursive = _buildRecursive;
}

// ***  KDTREE FACTORY METHODS  *** //
// ******************************** //
KDTreeNodeRoot * MultiThreadKDTreeFactory::makeFromPrimitivesUnsafe(
    vector<Primitive *> &primitives
){
    // Update max geometry depth
    maxGeometryDepth = (int) std::floor(std::log2(tpNode.getPoolSize()+1.0));

    // Build the KDTree using a modifiable copy of primitives pointers vector
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) kdtf->buildRecursive(
        nullptr,        // Parent node
        false,          // Node is not left child, because it is root not child
        primitives,     // Primitives to be contained inside the KDTree
        0,              // Starting depth level (must be 0 for root node)
        0               // Starting index at depth 0 (must be 0 for root node)
    );
    tpNode.join();
    if(root == nullptr){
        /*
         * NOTICE building a null KDTree is not necessarily a bug.
         * It is a natural process that might arise for instance when upgrading
         *  from static scene to dynamic scene in the XmlSceneLoader.
         * This message is not reported at INFO level because it is only
         *  relevant for debugging purposes.
         */
        std::stringstream ss;
        ss  << "Null KDTree with no primitives was built";
        logging::DEBUG(ss.str());
    }
    else{
        computeKDTreeStats(root);
        reportKDTreeStats(root, primitives);
        if(buildLightNodes) lighten(root);
    }
    return root;
}

// ***  BUILDING METHODS  *** //
// ************************** //
KDTreeNode * MultiThreadKDTreeFactory::buildRecursive(
    KDTreeNode *parent,
    bool const left,
    vector<Primitive*> &primitives,
    int const depth,
    int const index
){
    if(depth <= maxGeometryDepth)
        return buildRecursiveGeometryLevel(
            parent, left, primitives, depth, index
        );
    return buildRecursiveNodeLevel(parent, left, primitives, depth, index);
}

KDTreeNode * MultiThreadKDTreeFactory::buildRecursiveGeometryLevel(
    KDTreeNode *parent,
    bool const left,
    vector<Primitive*> &primitives,
    int const depth,
    int const index
){
    // Compute work distribution strategy definition
    int const k = 0; // TODO Rethink : Replace by total number of threads
    int const maxSplits = Scalar<int>::pow2(depth);
    int const alpha = (int) std::floor(k/maxSplits);
    int const beta = k % maxSplits;
    bool const excess = index < (maxSplits - beta);
    int const a = (excess) ? index*alpha : index*(alpha+1) - maxSplits + beta;
    int const b = (excess) ?
        alpha*(index+1) - 1 :
        index*(alpha+1) - maxSplits + beta + alpha;
    int const assignedThreads = b-a;

    // Geometry-level parallel processing

    // Post-processing
    if(depth == maxGeometryDepth){
        // Move assigned threads from geometry thread pool to node thread pool
        tpGeom.setAvailable(tpGeom.getAvailable()-assignedThreads);
        tpNode.setAvailable(tpNode.getAvailable()+assignedThreads);
    }

    // TODO Rethink : Implement geometry level building
    return buildRecursiveNodeLevel(parent, left, primitives, depth, index);
}

KDTreeNode * MultiThreadKDTreeFactory::buildRecursiveNodeLevel(
    KDTreeNode *parent,
    bool const left,
    vector<Primitive*> &primitives,
    int const depth,
    int const index
){
    bool posted = false;
    KDTreeBuildType *data = nullptr;
    if(primitives.size() >= minTaskPrimitives){
        data = new KDTreeBuildType(
            parent,
            left,
            primitives,
            depth,
            index
        );
        posted = tpNode.try_run_md_task(
            [&] (
                KDTreeNode *parent,
                bool const left,
                vector<Primitive*> &primitives,
                int const depth,
                int const index
            ) ->  void {
                if(left){
                    parent->left = this->kdtf->buildRecursive(
                        parent, left, primitives, depth, 2*index
                    );
                }
                else{
                    parent->right = this->kdtf->buildRecursive(
                        parent, left, primitives, depth, 2*index+1
                    );
                }
            },
            data
        );
    }
    if(posted){ // Null placeholder
        primitives.clear(); // Discard primitives, copy passed through data
        return nullptr;
    }
    else{ // Continue execution on current thread
        delete data; // Release data, it will not be used at all
        return this->kdtf->buildRecursive(
            parent, left, primitives, depth, 2*index+(left ? 0:1)
        );
    }

}
