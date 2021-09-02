#pragma once

#include <string>
#include <vector>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <IBinaryTreeNode.h>
#include <BinaryTreeDepthIterator.h>
#include <BinaryTreeFastDepthIterator.h>
// TODO Rethink : Implement and include below iterators
/*#include <BinaryTreeBreadthIterator.h>
#include <BinaryTreeFastBreadthIterator.h>*/
#include <Primitive.h>
#include <AABB.h>

// Including primitives below is necessary for serialization
#include <Triangle.h>
#include <Voxel.h>
#include <DetailedVoxel.h>

/**
 * @brief Class representing a KDTree node
 */
class KDTreeNode : public IBinaryTreeNode{
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
		ar & left;
        ar & right;
		ar & splitPos;
        ar & splitAxis;
		ar & primitives;
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
	/**
	 * @brief Default constructor for KDTreeNode
	 */
	KDTreeNode() {};
	/**
	 * @brief Copy constructor for KDTreeNode
	 * @param kdtn KDTreeNode to be copy-constructed
	 */
	KDTreeNode(KDTreeNode const &kdtn);
	/**
	 * @brief Move constructor for KDTreeNode
	 * @param kdtn KDTreeNode to be move-constructed
	 */
	KDTreeNode(KDTreeNode &&kdtn);
	/**
	 * @brief Destructor for KDTreeNode
	 */
	virtual ~KDTreeNode() {
		delete left;
		delete right;
	}

	// ***  ASSIGNMENT OPERATORS  *** //
	// ****************************** //
	/**
	 * @brief Copy assignment operator for KDTreeNode
	 * @param kdtn KDTreeNode to be copy-assigned
	 * @return Reference to copied KDTreeNode
	 */
	KDTreeNode& operator=(KDTreeNode const &kdtn);
	/**
	 * @brief Move assignment operator for KDTreeNode
	 * @param kdtn KDTreeNode to be move-assigned
	 * @return Reference to moved KDTreeNode
	 */
    KDTreeNode& operator=(KDTreeNode &&kdtn);

    // ***   S W A P   *** //
    // ******************* //
    /**
     * @brief Swap attributes of given KDTreeNode and current KDTreeNode
     * @param kdtn KDTreeNode to swap attributes with
     */
    void swap(KDTreeNode &kdtn);

    // ***  BINARY TREE INTERFACE  *** //
    // ******************************* //
    /**
     * @see IBinaryTree::getLeftChild
     */
    KDTreeNode * getLeftChild() const override
    {return left;}
    /**
     * @see IBinaryTree::getRightChild
     */
    KDTreeNode * getRightChild() const override
    {return right;}

    // ***  ITERATION METHODS  *** //
    // *************************** //
    /**
     * @brief Build a depth iterator starting at this node
     * @return Depth iterator starting at this node
     * @see BinaryTreeDepthIterator
     */
    inline BinaryTreeDepthIterator<KDTreeNode> buildDepthIterator(
        int const depth=0
    )
    {return {this, depth};}
    /**
     * @brief Build a fast depth iterator starting at this node
     * @return Fast depth iterator starting at this node
     * @see BinaryTreeFastDepthIterator
     */
    inline BinaryTreeFastDepthIterator<KDTreeNode> buildFastDepthIterator()
    {return {this};}
    // TODO Rethink : Uncomment below iterators
    /**
     * @brief Build a breadth iterator starting at this node
     * @return Breadth iterator starting at this node
     * @see BinaryTreeBreadthIterator
     */
    /*inline BinaryTreeBreadthIterator<KDTreeNode> buildBreadthIterator(
        int const depth=0
    )
    {return {this, depth};}*/
    /**
     * @brief Build a fast breadth iterator starting at this node
     * @return Fast breadth iterator starting at this node
     * @see BinaryTreeFastBreadthIterator
     */
    /*inline BinaryTreeFastBreadthIterator<KDTreeNode> buildFastBreadthIterator()
    {return {this};}*/

	// ***  OBJECT METHODS  *** //
	// ************************ //
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