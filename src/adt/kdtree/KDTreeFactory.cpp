#include <KDTreeFactory.h>

// ***  LIGHTEN  *** //
// ***************** //
LightKDTreeNode * KDTreeFactory::_lighten(KDTreeNode *node){
    // Build light node from node
    LightKDTreeNode *lightNode = lkdtnBlockAllocator.nextNew();
    lightNode->splitAxis = node->splitAxis;
    lightNode->splitPos = node->splitPos;
    lightNode->primitives = node->primitives;

    // Store children before deleting node
    void *left = (void *) node->left;
    void *right = (void *) node->right;

    // Delete node without deleting its children
    node->left = nullptr;
    node->right = nullptr;
    delete node;

    // Lighten children
    if(left != nullptr) left = _lighten((KDTreeNode *)left);
    if(right != nullptr) right = _lighten((KDTreeNode *)right);

    // Assign lightened children to parent
    lightNode->left = (LightKDTreeNode *) left;
    lightNode->right = (LightKDTreeNode *) right;

    // Return
    return lightNode;
}
void KDTreeFactory::lighten(KDTreeNodeRoot *root){
    // There is nothing to do with null trees
    if(root == nullptr) return;

    // Lighten children
    lkdtnBlockAllocator = LightKDTreeNodeBlockAllocator(); // Reset allocator
    size_t const maxBlockSize = 100000;
    size_t const maxNodes = root->stats_numInterior + root->stats_numLeaves;
    size_t curNodes = 0;
    vector<size_t> blocksSize;
    lkdtnBlockAllocator.setNextBlockSize([&] () -> size_t{
        size_t const nextSize = std::min(maxBlockSize, maxNodes - curNodes);
        curNodes += nextSize;
        blocksSize.push_back(nextSize);
        return nextSize;
    });
    if(root->left != nullptr)
        root->left = _lighten((KDTreeNode *)root->left);
    if(root->right != nullptr)
        root->right = _lighten((KDTreeNode *)root->right);
    root->blocks = lkdtnBlockAllocator.getBlocks();
    root->blocksSize = blocksSize;
}
