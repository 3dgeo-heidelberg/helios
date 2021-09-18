#include <KDTreeFactory.h>

// ***  LIGHTEN  *** //
// ***************** //
LightKDTreeNode * KDTreeFactory::_lighten(KDTreeNode *node){
    // Build light node from node
    LightKDTreeNode *lightNode = new LightKDTreeNode();
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
    if(root->left != nullptr)
        root->left = _lighten((KDTreeNode *)root->left);
    if(root->right != nullptr)
        root->right = _lighten((KDTreeNode *)root->right);
}
