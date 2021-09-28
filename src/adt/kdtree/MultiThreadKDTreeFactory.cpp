#include <MultiThreadKDTreeFactory.h>
#include <KDTreeBuildType.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiThreadKDTreeFactory::MultiThreadKDTreeFactory(
    shared_ptr<SimpleKDTreeFactory> kdtf
) :
    SimpleKDTreeFactory(),
    kdtf(kdtf)
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
            int const depth
        ) -> KDTreeNode * {
            return this->buildRecursive(parent, left, primitives, depth);
        }
    ;
    kdtf->_buildRecursive = _buildRecursive;
}

// ***  MULTI THREAD KDTREE FACTORY METHODS  *** //
// ********************************************* //
KDTreeNodeRoot * MultiThreadKDTreeFactory::makeFromPrimitivesUnsafe(
    vector<Primitive *> &primitives
){
    // Build the KDTree using a modifiable copy of primitives pointers vector
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) kdtf->buildRecursive(
        nullptr,        // Parent node
        false,          // Node is not left child, because it is root not child
        primitives,     // Primitives to be contained inside the KDTree
        0               // Starting depth level (must be 0 for root node)
    );
    tp.join();
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
    int const depth
) {
    // TODO Rethink : Compare left async, right sync VS full async
    // Left on split task, right on current task
    if(left){
        tp.run_md_task(
            [&] (
                KDTreeNode *parent,
                bool const left,
                vector<Primitive*> &primitives,
                int const depth
            ) ->  void {
                parent->left = this->kdtf->buildRecursive(
                    parent, left, primitives, depth
                );
            },
            new KDTreeBuildType(
                parent,
                left,
                primitives,
                depth
            )
        );
        return nullptr;
    }
    else{
        return this->kdtf->buildRecursive(parent, left, primitives, depth);
    }
    // Double task split implementation
    /*tp.run_md_task(
        [&] (
            KDTreeNode *parent,
            bool const left,
            vector<Primitive*> &primitives,
            int const depth
        ) ->  void {
            if(left){
                parent->left = this->kdtf->buildRecursive(
                    parent, left, primitives, depth
                );
            }
            else{
                parent->right = this->kdtf->buildRecursive(
                    parent, left, primitives, depth
                );
            }
        },
        new KDTreeBuildType(
            parent,
            left,
            primitives,
            depth
        )
    );
    return nullptr;*/
}
