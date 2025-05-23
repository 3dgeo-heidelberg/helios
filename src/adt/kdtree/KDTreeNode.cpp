#include "KDTreeNode.h"
#include "KDTreeNodeRoot.h"
#include "KDTreePrimitiveComparator.h"
#include <SerialIO.h>
#include <serial.h>

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

// ***  OBJECT METHODS  *** //
// ************************ //
void
KDTreeNode::writeObject(std::string path)
{
  std::stringstream ss;
  ss << "Writing " << path << "...";
  logging::INFO(ss.str());
  SerialIO::getInstance()->write<KDTreeNode>(path, this);
}

KDTreeNode*
KDTreeNode::readObject(std::string path)
{
  std::stringstream ss;
  ss << "Reading " << path << "...";
  logging::INFO(ss.str());
  return SerialIO::getInstance()->read<KDTreeNode>(path);
}
