#include <LightKDTreeNode.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
LightKDTreeNode::LightKDTreeNode(LightKDTreeNode const &kdtn){
    left = new LightKDTreeNode(*kdtn.left);
    right = new LightKDTreeNode(*kdtn.right);
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    primitives = kdtn.primitives;
}

LightKDTreeNode::LightKDTreeNode(LightKDTreeNode &&kdtn){
    left = new LightKDTreeNode(*kdtn.left);
    right = new LightKDTreeNode(*kdtn.right);
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    primitives = kdtn.primitives;
}

// ***  ASSIGNMENT OPERATORS  *** //
// ****************************** //
LightKDTreeNode& LightKDTreeNode::operator=(LightKDTreeNode const &kdtn){
    LightKDTreeNode tmp(kdtn);
    swap(tmp);
    return *this;
}

LightKDTreeNode& LightKDTreeNode::operator=(LightKDTreeNode &&kdtn){
    LightKDTreeNode tmp(kdtn);
    swap(tmp);
    return *this;
}

// ***   S W A P   *** //
// ******************* //
void LightKDTreeNode::swap(LightKDTreeNode &kdtn){
    std::swap(left, kdtn.left);
    std::swap(right, kdtn.right);
    std::swap(splitPos, kdtn.splitPos);
    std::swap(splitAxis, kdtn.splitAxis);
    std::swap(primitives, kdtn.primitives);
}
