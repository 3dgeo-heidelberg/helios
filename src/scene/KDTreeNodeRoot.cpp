#include <KDTreeNodeRoot.h>
#include <logging.hpp>


// ***  CLASS METHODS  *** //
// *********************** //
KDTreeNodeRoot* KDTreeNodeRoot::build(const vector<Primitive*> & primitives) {
    KDTreeNodeRoot* root = (KDTreeNodeRoot *) buildRecursive(primitives, 0);
    root->computeKDTreeStats(root, 0);
    std::stringstream ss;
    ss  << "KDTree (num. primitives " << primitives.size() << ") :\n\t"
        << "Max. # primitives in leaf: "
        << root->stats_maxNumPrimsInLeaf << "\n\t"
        << "Min. # primitives in leaf: "
        << root->stats_minNumPrimsInLeaf << "\n\t"
        << "Max. depth reached: : "
        << root->stats_maxDepthReached;
    logging::INFO(ss.str());
    return root;
}
