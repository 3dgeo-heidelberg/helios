#include "KDTreeNode.h"
#include "KDTreeNodeRoot.h"
#include "KDTreePrimitiveComparator.h"
#include <SerialIO.h>
#include <serial.h>

#include <iostream>
#include <logging.hpp>
using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
KDTreeNode::KDTreeNode(KDTreeNode const &kdtn){
    left = new KDTreeNode(*kdtn.left);
    right = new KDTreeNode(*kdtn.right);
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    primitives = kdtn.primitives;
}

KDTreeNode::KDTreeNode(KDTreeNode &&kdtn){
    left = kdtn.left;
    right = kdtn.right;
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    primitives = kdtn.primitives;
}

// ***  ASSIGNMENT OPERATORS  *** //
// ****************************** //
KDTreeNode& KDTreeNode::operator=(KDTreeNode const &kdtn){
    KDTreeNode tmp(kdtn);
    swap(tmp);
    return *this;
}

KDTreeNode& KDTreeNode::operator=(KDTreeNode &&kdtn){
    KDTreeNode tmp(kdtn);
    swap(tmp);
    return *this;
}

// ***   S W A P   *** //
// ******************* //
void KDTreeNode::swap(KDTreeNode &kdtn){
    std::swap(left, kdtn.left);
    std::swap(right, kdtn.right);
    std::swap(splitPos, kdtn.splitPos);
    std::swap(splitAxis, kdtn.splitAxis);
    std::swap(primitives, kdtn.primitives);
}

// ***  OBJECT METHODS  *** //
// ************************ //
void KDTreeNode::writeObject(string path) {
    stringstream ss;
	ss << "Writing " << path << "...";
	logging::INFO(ss.str());
	SerialIO::getInstance()->write<KDTreeNode>(path, this);
}

KDTreeNode* KDTreeNode::readObject(string path) {
    stringstream ss;
	ss << "Reading " << path << "...";
	logging::INFO(ss.str());
	return SerialIO::getInstance()->read<KDTreeNode>(path);
}
