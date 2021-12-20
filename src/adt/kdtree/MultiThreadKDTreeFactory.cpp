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
    tpNode(numJobs),
    minTaskPrimitives(32)
{
    /*
     * Handle node thread pool start pending tasks so when geometry-level
     * parallelization is used no node-level parallelization occurs before
     * it must (at adequate depth)
     */
    tpNode.setPendingTasks(numJobs); // TODO Rethink : To MDTP constructor?
    // TODO Rethink : No to MDTP constructor, handle geometry/node case here

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
    // TODO Rethink : Change call to exploit geometry parallelization at root
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
    int const k = tpNode.getPoolSize(); // TODO Rethink : Replace by total number of threads
    int const maxSplits = Scalar<int>::pow2(depth);
    int const alpha = (int) std::floor(k/maxSplits);
    int const beta = k % maxSplits;
    bool const excess = index < (maxSplits - beta);
    int const a = (excess) ? index*alpha : index*(alpha+1) - maxSplits + beta;
    int const b = (excess) ?
        alpha*(index+1) - 1 :
        index*(alpha+1) - maxSplits + beta + alpha;
    int const assignedThreads = 1+b-a;

    // Geometry-level parallel processing
    // TODO Rethink : Prevent duplicated code (below is copy of SKDT::buildRec)
    if(primitives.empty()) return nullptr;
    KDTreeNode *node;
    if(depth > 0) node = new KDTreeNode();
    else node = new KDTreeNodeRoot();
    kdtf->computeNodeBoundaries(node, parent, left, primitives);
    kdtf->defineSplit(node, parent, primitives, depth);
    vector<Primitive*> leftPrimitives, rightPrimitives;
    kdtf->GEOM_populateSplits(
        primitives,
        node->splitAxis,
        node->splitPos,
        leftPrimitives,
        rightPrimitives,
        assignedThreads
    );

    // Post-processing
    if(depth == maxGeometryDepth){
        // Move assigned threads from geometry thread pool to node thread pool
        tpNode.safeSubtractPendingTasks(assignedThreads);
    }

    // TODO Rethink : Implement geometry level building
    //return this->kdtf->buildRecursive(parent, left, primitives, depth, index);
    kdtf->buildChildrenNodes(
        node,
        parent,
        primitives,
        depth,
        index,
        leftPrimitives,
        rightPrimitives
    );
    return node;
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
