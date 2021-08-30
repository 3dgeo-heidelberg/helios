#pragma once

#include "Primitive.h"
#include "Vertex.h"
#include "AABB.h"


/**
 * @brief Class representing a voxel primitive
 */
class Voxel : public Primitive {
private:
    // ***  BOOST SERIALIZATION  *** //
    // ***************************** //
    friend class boost::serialization::access;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
        boost::serialization::void_cast_register<Voxel, Primitive>();
        ar & boost::serialization::base_object<Primitive>(*this);
        ar & v;
        ar & numPoints;
        ar & r & g & b;
        ar & normal;
        ar & bbox;
        ar & color;
        ar & halfSize;
    }

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Vertex representing the voxel center
     */
    Vertex v;
    /**
     * @brief Number of points inside the voxel. Useful when the voxel has
     * been built from a point cloud
     */
    int numPoints = 0;
    /**
     * @brief Aggregated red component from points inside voxel
     */
    double r = 0;
    /**
     * @brief Aggregated green component from points inside voxel
     */
    double g = 0;
    /**
     * @brief Aggregated blue component from points inside voxel
     */
    double b = 0;
    /**
     * @brief Voxel normal vecctor
     */
    glm::dvec3 normal = glm::dvec3(0, 0, 0);
    /**
     * @brief Axis aligned bounding box containing the voxel
     */
    AABB* bbox = nullptr;
    /**
     * @brief Voxel color. This attribute is not used at the moment and might
     * be removed in the future
     */
    Color4f color;
    /**
     * @brief Half of the voxel sizxe
     */
    double halfSize;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Default voxel constructor
     */
    Voxel() : Primitive() {};
    /**
     * @brief Voxel constructor
     * @param center Voxel center coordinates
     * @param voxelSize Voxel size
     */
	Voxel(glm::dvec3 center, double voxelSize);
	/**
	 * @brief Voxel constructor
	 * @param x X coordinate of voxel center
	 * @param y Y coordinate of voxel center
	 * @param z Z coordinate of voxel center
	 * @param halfVoxelSize Half of the voxel size
	 */
    Voxel(double x, double y, double z, double halfVoxelSize);
	~Voxel() override{if(bbox!=nullptr) delete bbox;}
	/**
	 * @see Primitive::clone
	 */
    Primitive* clone() override;
    /**
     * @see Primitive::_clone
     */
    void _clone(Primitive *p) override;

	// ***  COPY / MOVE SEMANTICS *** //
	// ****************************** //
	/**
	 * @brief Swap semantic implementation for voxel
	 * @param voxel Voxel to swap with
	 */
	void swap(Voxel &voxel){ // Swap function
	    std::swap(this->v, voxel.v);
	    std::swap(this->numPoints, voxel.numPoints);
	    std::swap(this->r, voxel.r);
	    std::swap(this->g, voxel.g);
	    std::swap(this->b, voxel.b);
	    std::swap(this->bbox, voxel.bbox);
	    std::swap(this->color, voxel.color);
	    std::swap(this->halfSize, voxel.halfSize);
	}
	Voxel(Voxel const &voxel) : // Copy constructor
        v(voxel.v),
        numPoints(voxel.numPoints),
        r(voxel.r),
        g(voxel.g),
        b(voxel.b),
        bbox(new AABB(*voxel.bbox)),
        color(voxel.color),
        halfSize(voxel.halfSize)
    {}
	Voxel & operator= (Voxel const &voxel){ // Copy assignment
	    Voxel newVoxel = Voxel(voxel);
	    swap(newVoxel);
        return *this;
	}
	Voxel(Voxel &&voxel){ // Move constructor
	    swap(voxel);
	}
	Voxel & operator= (Voxel &&voxel){ // Move assignment
	    swap(voxel);
	    return *this;
	}


    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see Primitive::getAABB
     */
	AABB* getAABB() override;
	/**
	 * @see Primitive::getCentroid
	 */
	glm::dvec3 getCentroid() override;
	/**
	 * @see Primitive::getIncidenceAngle_rad
	 */
	double getIncidenceAngle_rad(
	    const glm::dvec3& rayOrigin,
	    const glm::dvec3& rayDir,
	    const glm::dvec3& intersectionPoint
    ) override;
	/**
	 * @brief Obtain the incidence angle with respect to closest face for
	 * given intersection point.
	 *
	 * This function is meant to be used when there is no valid voxel normal
	 * available to compute incidence angle.
	 *
	 * @return Incidence angle (radians) with respect to closest face
	 */
	double getIncidenceAngleClosestFace_rad(
        const glm::dvec3& rayOrigin,
        const glm::dvec3& rayDir,
        const glm::dvec3& intersectionPoint
	 );
	/**
	 * @see Voxel::getRayIntersection
	 */
	std::vector<double> getRayIntersection(
	    const glm::dvec3& rayOrigin,
	    const glm::dvec3& rayDir
    ) override;
	/**
	 * @see Voxel::getRayIntersectionDistance
	 */
    double getRayIntersectionDistance(
        const glm::dvec3& rayOrigin,
        const glm::dvec3& rayDir
    ) override;
    /**
     * @see Voxel::getNumVertices
     */
    inline virtual size_t getNumVertices() override {return 1;}
    /**
     * @see Voxel::getVertices
     */
	Vertex* getVertices() override;
	/**
	 * @see Voxel::getNumFullVertices
	 */
    inline virtual size_t getNumFullVertices() override {return 2;}
    /**
     * @see Voxel::getFullVertices
     */
    Vertex* getFullVertices() override;
    /**
     * @see Voxel::getGroundZOffset
     */
    inline double getGroundZOffset() override {return halfSize*2.0;}
    /**
     * @brief Check if voxel has a valid normal
     *
     * A valid normal is any normal distinct than (0, 0, 0)
     *
     * @return True if voxel has a valid normal, false otherwise
     */
    bool hasNormal()
        {return normal[0]!=0.0 || normal[1]!=0.0 || normal[2]!=0.0;}


    // ***  TRANSFORMATIONS  *** //
    // ************************* //
    /**
     * @brief <span style="color: #AA0000">
     *      Voxel <b>cannot</b> be rotated
     *  </span>
     * @see Primitive::rotate
     */
    void rotate(Rotation &r) override{
        Primitive::rotate(r);
        update();
    }
    /**
     * @see Primitive::scale
     */
    void scale(double const factor) override{
        halfSize *= factor;
        update();
    }
    /**
     * @see Primitive::translate
     */
    void translate(glm::dvec3 const &shift) override{
        Primitive::translate(shift);
        update();
    }

    /**
     * @see Primitive::update
     */
	void update() override;
};

// ***  BOOST SERIALIZATION  *** //
// ***************************** //
/*
 * Below code is commented because it is not necessary and it might cause
 * memory leaks when called from external sources (i.e. from python)
 */
/*namespace boost {
	namespace serialization {

		template<class Archive>
		inline void save_construct_data(
			Archive & ar, const Voxel * t, const unsigned int file_version)
		{
			// save data required to construct instance
			ar << t->v.pos;
			ar << t->halfSize * 2;
		}

		template<class Archive>
		inline void load_construct_data(
			Archive & ar, Voxel * t, const unsigned int file_version)
		{
			// retrieve data from archive required to construct new instance
			glm::dvec3 center;
			double voxelSize;
			ar >> center;
			ar >> voxelSize;

			// invoke inplace constructor to initialize instance of my_class
			::new(t)Voxel(center, voxelSize);
		}
	}
}*/