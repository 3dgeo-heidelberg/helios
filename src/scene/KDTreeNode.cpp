#include "KDTreeNode.h"
#include "KDTreeNodeRoot.h"
#include "KDTreePrimitiveComparator.h"
#include <SerialIO.h>

#include <iostream>
#include <logging.hpp>
using namespace std;

// ***  CLASS METHODS  *** //
// *********************** //
KDTreeNode* KDTreeNode::buildRecursive(vector<Primitive*> primitives, int depth) {
	size_t primsSize = primitives.size();
	if (primsSize == 0) {
		return nullptr;
	}

	KDTreeNode *node;
	if(depth > 0) node = new KDTreeNode();
	else node = new KDTreeNodeRoot();

	// TODO 5: Implement surface area heuristics?
	int splitAxis = depth % 3;

	// Sort faces along split axis:
	// ATTENTION: Sorting must happen BEFORE splitPos is computed as the median

	// Sort primitives along split axis:
	std::sort(primitives.begin(), primitives.end(), KDTreePrimitiveComparator(splitAxis));

	// Compute split position:
	auto p = next(primitives.begin(), primsSize / 2);
	double splitPos = (*p)->getCentroid()[splitAxis];

	// ########## BEGIN Fill children's primitive lists ##########

	vector<Primitive*> sublist_left;
	vector<Primitive*> sublist_right;
	
	for (auto p : primitives) {
		AABB* box = p->getAABB();

		if (box->getMin()[splitAxis] <= splitPos) {
			sublist_left.push_back(p);
		}

		if (box->getMax()[splitAxis] > splitPos) {
			sublist_right.push_back(p);
		}
	}

	if(sublist_left.size() != primsSize && sublist_right.size() != primsSize){
		node->splitAxis = splitAxis;
		node->splitPos = splitPos;

		if (sublist_left.size() > 0) {
			node->left = buildRecursive(sublist_left, depth + 1);
		}

		if (sublist_right.size() > 0) {
			node->right = buildRecursive(sublist_right, depth + 1);
		}
	}
	else {
		// Otherwise, make this node a leaf:
		node->splitAxis = -1;
		node->primitives = primitives;
	}

	return node;
}

// ***  OBJECT METHODS  *** //
// ************************ //
void KDTreeNode::computeKDTreeStats(KDTreeNode *_root, int depth){
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) _root;

    // Update max depth if necessary
    if(depth > root->stats_maxDepthReached)
        root-> stats_maxDepthReached = depth;

    if(splitAxis == -1){ // If leaf node
        int nPrimitives = primitives.size();
        if(nPrimitives > root->stats_maxNumPrimsInLeaf)
            root->stats_maxNumPrimsInLeaf = nPrimitives;
        if(nPrimitives < root->stats_minNumPrimsInLeaf){
            root->stats_minNumPrimsInLeaf = nPrimitives;
        }
    }

    if(left != nullptr) left->computeKDTreeStats(_root, depth+1);
    if(right != nullptr) right->computeKDTreeStats(_root, depth+1);

}

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
