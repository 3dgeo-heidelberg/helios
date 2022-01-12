#include <MultiThreadKDTreeFactory.h>
#include <KDTreeBuildType.h>
#include <surfaceinspector/maths/Scalar.hpp>

using SurfaceInspector::maths::Scalar;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiThreadKDTreeFactory::MultiThreadKDTreeFactory(
    shared_ptr<SimpleKDTreeFactory> const kdtf,
    shared_ptr<SimpleKDTreeGeometricStrategy> const gs,
    size_t const numJobs,
    size_t const geomJobs
) :
    SimpleKDTreeFactory(),
    kdtf(kdtf),
    gs(gs),
    tpNode(numJobs),
    minTaskPrimitives(32),
    numJobs(numJobs),
    geomJobs(geomJobs)
{
    /*
     * Handle node thread pool start pending tasks so when geometry-level
     * parallelization is used no node-level parallelization occurs before
     * it must (at adequate depth).
     * Also handle max depth for geometry-level parallelization and initialize
     * sequencer with masters thread for different nodes if necessary
     */
    if(geomJobs == 1){
        tpNode.setPendingTasks(0);
        maxGeometryDepth = -1;
        masters = nullptr;
    }
    else if(geomJobs > 1){
        tpNode.setPendingTasks(geomJobs);
        maxGeometryDepth = (int) std::floor(std::log2(geomJobs));
        masters = std::make_shared<SharedTaskSequencer>(
            Scalar<int>::pow2(maxGeometryDepth)-1
        );
    }
    else{
        std::stringstream ss;
        ss  << "MultiThreadKDTreeFactory failed to build because of "
            << "unexpected number of jobs (" << geomJobs
            << ") for geometry-level parallelization";
        throw HeliosException(ss.str());
    }

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
    vector<Primitive *> &primitives,
    bool const computeStats,
    bool const reportStats
){
    // Build the KDTree using a modifiable copy of primitives pointers vector
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) kdtf->_buildRecursive(
        nullptr,        // Parent node
        false,          // Node is not left child, because it is root not child
        primitives,     // Primitives to be contained inside the KDTree
        0,              // Starting depth level (must be 0 for root node)
        0               // Starting index at depth 0 (must be 0 for root node)
    );
    if(masters != nullptr){
        masters->joinAll(); // Join masters threads from geometry-level
    }
    tpNode.join(); // Join auxiliar threads from node-level
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
        if(computeStats){
            computeKDTreeStats(root);
            if(reportStats) reportKDTreeStats(root, primitives);
        }
        if(buildLightNodes){
            if(!computeStats){
                /*
                 * Efficient lighten requires tree stats (number of interior
                 * nodes and number of leaf nodes) to be known. Thus, if stats
                 * have not been computed but lighten is required, then
                 * SimpleKDTree stats are computed as they are the fastest
                 * ones which satisfy aforementioned requirements.
                 */
                SimpleKDTreeFactory::computeKDTreeStats(root);
            }
            lighten(root);
        }
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
    if(depth <= maxGeometryDepth){
        return buildRecursiveGeometryLevel(
            parent, left, primitives, depth, index
        );
    }
    else if(parent==nullptr){
        return kdtf->buildRecursive(parent, left, primitives, depth, index);
    }
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
    int const maxSplits = Scalar<int>::pow2(depth);
    int const alpha = (int) std::floor(geomJobs/maxSplits);
    int const beta = geomJobs % maxSplits;
    bool const excess = index < (maxSplits - beta); // True when 1 extra thread
    int const a = (excess) ? index*alpha : index*(alpha+1) - maxSplits + beta;
    int const b = (excess) ?
        alpha*(index+1) - 1 :
        index*(alpha+1) - maxSplits + beta + alpha;
    int const auxiliarThreads = b-a; // Subordinated threads
    int const assignedThreads = 1+auxiliarThreads; // Total threads

    // Geometry-level parallel processing
    return kdtf->buildRecursiveRecipe(
        parent,
        left,
        primitives,
        depth,
        index,
        [&] (
            KDTreeNode *node,
            KDTreeNode *parent,
            bool const left,
            vector<Primitive *> const &primitives
        ) -> void {
            gs->GEOM_computeNodeBoundaries(
                node,
                parent,
                left,
                primitives,
                assignedThreads
            );
        },
        [&] (
            KDTreeNode *node,
            KDTreeNode *parent,
            vector<Primitive *> &primitives,
            int const depth
        ) -> void {
            gs->GEOM_defineSplit(
                node, parent, primitives, depth, assignedThreads
            );
        },
        [&] (
            vector<Primitive *> const &primitives,
            int const splitAxis,
            double const splitPos,
            vector<Primitive *> &leftPrimitives,
            vector<Primitive *> &rightPrimitives
        ) -> void {
            gs->GEOM_populateSplits(
                primitives,
                splitAxis,
                splitPos,
                leftPrimitives,
                rightPrimitives,
                assignedThreads
            );
        },
        [&] (
            KDTreeNode *node,
            KDTreeNode *parent,
            vector<Primitive *> const &primitives,
            int const depth,
            int const index,
            vector<Primitive *> &leftPrimitives,
            vector<Primitive *> &rightPrimitives
        ) -> void {
            buildChildrenGeometryLevel(
                node,
                parent,
                primitives,
                depth,
                index,
                leftPrimitives,
                rightPrimitives,
                auxiliarThreads
            );
        }
    );
}

void MultiThreadKDTreeFactory::buildChildrenGeometryLevel(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> const &primitives,
    int const depth,
    int const index,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives,
    int const auxiliarThreads
){
    // Geometry-level building of children nodes
    if(depth == maxGeometryDepth){
        // Move auxiliar threads from geometry thread pool to node thread pool
        if(auxiliarThreads > 0)
            tpNode.safeSubtractPendingTasks(auxiliarThreads);
        // Recursively build children nodes
        kdtf->buildChildrenNodes(
            node,
            parent,
            primitives,
            depth,
            index,
            leftPrimitives,
            rightPrimitives
        );
        // Allow one more thread to node-level pool when master thread finishes
        tpNode.safeSubtractPendingTasks(1);
    }
    else{
        gs->GEOM_buildChildrenNodes(
            node,
            parent,
            primitives,
            depth,
            index,
            leftPrimitives,
            rightPrimitives,
            masters
        );
    }
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
