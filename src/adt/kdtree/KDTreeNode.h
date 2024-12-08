#pragma once

#include <LightKDTreeNode.h>


/**
 * @brief Class representing a KDTree node
 */
class KDTreeNode : public LightKDTreeNode{
private:
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
	    /*
	     * Code below is commented because it is no longer needed as it is now
	     * done in LightKDTreeNode. However, in case problems arise or
	     * KDTreeNode should become the basis for KDTree node implementation,
	     * it must be uncommented or serialization will not work
	     */
        // Register classes derived from Primitive
        /*ar.template register_type<Vertex>();
        ar.template register_type<AABB>();
        ar.template register_type<Triangle>();
        ar.template register_type<Voxel>();
        ar.template register_type<DetailedVoxel>();*/

	    // Serialization itself
        boost::serialization::void_cast_register<
            KDTreeNode, LightKDTreeNode
        >();
        ar & boost::serialization::base_object<LightKDTreeNode>(*this);
		ar & bound;
        ar & surfaceArea;
    }

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The axis-aligned boundary of the node
     */
    AABB bound;
    /**
     * @brief The summation of areas for all faces at node boundaries
     */
    double surfaceArea = std::numeric_limits<double>::quiet_NaN();

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
    ~KDTreeNode() override = default;

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