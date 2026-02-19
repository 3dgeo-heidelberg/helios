#include "KDTreeNode.h"
#include "KDTreeNodeRoot.h"
#include "KDTreePrimitiveComparator.h"

#include <iostream>
#include <logging.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
KDTreeNode::KDTreeNode(KDTreeNode const& kdtn)
  : LightKDTreeNode(kdtn)
{
  bound = kdtn.bound;
  surfaceArea = kdtn.surfaceArea;
}

KDTreeNode::KDTreeNode(KDTreeNode&& kdtn)
  : LightKDTreeNode(kdtn)
{
  bound = kdtn.bound;
  surfaceArea = kdtn.surfaceArea;
}

// ***  ASSIGNMENT OPERATORS  *** //
// ****************************** //
KDTreeNode&
KDTreeNode::operator=(KDTreeNode const& kdtn)
{
  KDTreeNode tmp(kdtn);
  swap(tmp);
  return *this;
}

KDTreeNode&
KDTreeNode::operator=(KDTreeNode&& kdtn)
{
  KDTreeNode tmp(kdtn);
  swap(tmp);
  return *this;
}

// ***   S W A P   *** //
// ******************* //
void
KDTreeNode::swap(KDTreeNode& kdtn)
{
  LightKDTreeNode::swap(kdtn);
  std::swap(bound, kdtn.bound);
  std::swap(surfaceArea, kdtn.surfaceArea);
}
