#pragma once

#include <string>
#include <vector>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "Primitive.h"
#include "AABB.h"
#include <serial.h> // TODO Rethink : Solves the issue?
#include <Triangle.h> // TODO Rethink : Solves the issue?
#include <Voxel.h> // TODO Rethink : Solves the issue?
#include <DetailedVoxel.h> // TODO Rethink : Solves the issue?

/**
 * @brief Class representing a KDTree node
 */
class KDTreeNode {
    // ***  SERIALIZATION  *** //
    // *********************** //
	friend class boost::serialization::access;
	/**
	 * @brief Serialize a KDTreeNode to a stream of bytes
	 * @tparam Archive Type of rendering
	 * @param ar Specific rendering for the stream of bytes
	 * @param version Version number for the KDTreeNode
	 */
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
        // Register classes derived from Primitive
        ar.template register_type<AABB>();
        ar.template register_type<Triangle>();
        ar.template register_type<Vertex>();
        ar.template register_type<Voxel>();
        ar.template register_type<DetailedVoxel>();
	    std::cout << "Exporting KDTreeNode (1) ..." << std::endl; // TODO Remove
		ar & left;
        std::cout << "Exporting KDTreeNode (2) ..." << std::endl; // TODO Remove
        ar & right;
        std::cout << "Exporting KDTreeNode (3) ..." << std::endl; // TODO Remove
		ar & splitPos;
        std::cout << "Exporting KDTreeNode (4) ..." << std::endl; // TODO Remove
        ar & splitAxis;
        std::cout << "Exporting KDTreeNode (5) ..." << std::endl; // TODO Remove
		ar & primitives;
        std::cout << "Exported KDTreeNode!" << std::endl; // TODO Remove
    }

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Pointer to node at left side on space partition. Can be nullptr
     *  if there is no left side node
     */
	KDTreeNode* left = nullptr;
	/**
	 * @biref Pointer to node at right side on space partition. Can be nullptr
	 *  if there is no right side node
	 */
	KDTreeNode* right = nullptr;
	/**
	 * @brief Point position at corresponding split axis
	 */
    double splitPos = 0;
    /**
     * @brief Space axis to consider at current depth
     */
    int splitAxis = 0;
    /**
     * @brief Vector of primitives associated with the node
     */
    std::vector<Primitive*> primitives;

	//TODO serialization
	//KDTreeNode() {}
	// no default construct guarentees that no invalid object ever exists
	// > save_construct_data will have to be overridden
	// see https://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/serialization.html
	// see Triangle.h

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	virtual ~KDTreeNode() {
		delete left;
		delete right;
	}


	// ***  CLASS METHODS  *** //
	// *********************** //
	/**
	 * @brief Recursively build a KDTree for given primitives
	 * @param primitives Primitives to build KDTree splitting them
	 * @param depth Current depth at build process. Useful for tracking
	 *  recursion level
	 * @return Built node
	 */
	static KDTreeNode* buildRecursive(std::vector<Primitive*> primitives, int depth);


	// ***  OBJECT METHODS  *** //
	// ************************ //
	/**
	 * @brief Analyze KDTree computing its max depth and the minimum and
	 *  maximum number of primitives considering all nodes
	 * @param root Root node
	 * @param depth Current depth. Useful for tracking since computations
	 *  require recursive analysis of KDTree
	 */
    void computeKDTreeStats(KDTreeNode *root, int depth=0);
    /**
     * @brief Serialize KDTree
     * @param path Path to file where the serialized KDTree must be exported
     */
	void writeObject(std::string path);
	/**
	 * @brief Import a serialized KDTree from file
	 * @param path Path to the file containing a serialized KDTree
	 * @return Imported KDTree
	 */
	static KDTreeNode* readObject(std::string path);
};