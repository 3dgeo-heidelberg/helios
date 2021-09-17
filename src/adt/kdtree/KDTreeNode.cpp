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
KDTreeNode::KDTreeNode(KDTreeNode const &kdtn) : LightKDTreeNode() {
    left = new KDTreeNode(*(KDTreeNode *)kdtn.left);
    right = new KDTreeNode(*(KDTreeNode *)kdtn.right);
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    bound = kdtn.bound;
    surfaceArea = kdtn.surfaceArea;
}

KDTreeNode::KDTreeNode(KDTreeNode &&kdtn) : LightKDTreeNode(kdtn){
    left = new KDTreeNode(*(KDTreeNode *)kdtn.left);
    right = new KDTreeNode(*(KDTreeNode *)kdtn.right);
    splitPos = kdtn.splitPos;
    splitAxis = kdtn.splitAxis;
    bound = kdtn.bound;
    surfaceArea = kdtn.surfaceArea;
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
    LightKDTreeNode::swap(kdtn);
    std::swap(bound, kdtn.bound);
    std::swap(surfaceArea, kdtn.surfaceArea);
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
