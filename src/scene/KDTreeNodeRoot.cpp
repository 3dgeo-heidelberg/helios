#include <KDTreeNodeRoot.h>
#include <logging.hpp>


// ***  CLASS METHODS  *** //
// *********************** //
KDTreeNodeRoot* KDTreeNodeRoot::build(const vector<Primitive*> & primitives) {
    KDTreeNodeRoot* root = (KDTreeNodeRoot *) buildRecursive(primitives, 0);
    std::stringstream ss;
    if(root == nullptr){
        /*
         * NOTICE building a null KDTree is not necessarily a bug.
         * It is a natural process that might arise for instance when upgrading
         *  from static scene to dynamic scene in the XmlSceneLoader.
         * This message is not reported at INFO level because it is only
         *  relevant for debugging purposes.
         */
        ss  << "Null KDTree with no primitives was built";
        logging::DEBUG(ss.str());
    }
    else{
        root->computeKDTreeStats(root, 0);
        ss  << "KDTree (num. primitives " << primitives.size() << ") :\n\t"
        << "Max. # primitives in leaf: "
        << root->stats_maxNumPrimsInLeaf << "\n\t"
        << "Min. # primitives in leaf: "
        << root->stats_minNumPrimsInLeaf << "\n\t"
        << "Max. depth reached: : "
        << root->stats_maxDepthReached;
        logging::INFO(ss.str());
    }
    return root;
}
