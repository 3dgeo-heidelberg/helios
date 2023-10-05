#pragma once
#include <set>
#include <unordered_set>
#include "Primitive.h"
#include "Vertex.h"

/**
 * @brief Class representing an Axis Aligned Bounding Box (AABB)
 */
class AABB : public Primitive {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
	friend class boost::serialization::access;
    /**
     * @brief Serialize an axis-aligned bounding box to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the axis-aligned bounding box
     */
	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version) {
        boost::serialization::void_cast_register<AABB, Primitive>();
        ar & boost::serialization::base_object<Primitive>(*this);
		ar & vertices;
		ar & bounds;
	}

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Vertices defining the axis aligned bounding box
     *
     * The first vertex represents the minimum boundaries while the second one
     * represents the maximum boundaries
     */
    Vertex vertices[2]; // 0 min, 1 max
    /**
     * @brief Cached bounds to speed-up intersection computation
     */
    glm::dvec3 bounds[2];

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for axis aligned bounding box
     */
    AABB() = default;
    /**
     * @brief Build an axis aligned bounding box through min and max values
     * for each coordinate
     * @param min Min value for each 3D coordinate
     * @param max Max value for each 3D coordinate
     */
    AABB(glm::dvec3 min, glm::dvec3 max);
    /**
     * @brief Build an axis aligned bounding box through given points \f$a\f$
     *  and \f$b\f$ which are the minimum and maximum vertex of the axis
     *  aligned bounding box respectively
     * @param ax \f$x\f$ component of the minimum vertex
     * @param ay \f$y\f$ component of the minimum vertex
     * @param az \f$z\f$ component of the minimum vertex
     * @param bx \f$x\f$ component of the maximum vertex
     * @param by \f$y\f$ component of the maximum vertex
     * @param bz \f$z\f$ component of the maximum vertex
     */
    AABB(
        double const ax,
        double const ay,
        double const az,
        double const bx,
        double const by,
        double const bz
    ) :
        AABB(glm::dvec3(ax, ay, az), glm::dvec3(bx, by, bz))
    {}
    /**
     * @brief Default destructor for axis aligned bounding box
     */
    ~AABB() override = default;
    /**
     * @brief Generate a clone of the primitive
     * @return Pointer to the clone of the primitive
     * @see Primitive::clone
     */
    Primitive* clone() override;
    /**
     * @brief Assist clone function
     * @param p Pointer to the clone to be updated
     * @see Primitive::_clone
     */
    void _clone(Primitive *p) override;


    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Get the min value for each coordinate of the axis aligned
     * bounding box
     * @return Min value for each coordinate of the axis aligned bounding box
     */
	inline glm::dvec3 const & getMin() const {return vertices[0].pos;}
	/**
	 * @brief Get the max value for each coordinate of the axis aligned
	 * bounding box
	 * @return Max value for each coordinate of the axis aligned bounding box
	 */
    inline glm::dvec3 const & getMax() const {return vertices[1].pos;}
    /**
     * @brief Obtain this axis aligned bounding box
     * @return The axis aligned bounding box itself
     * @see Primitive::getAABB
     */
	AABB* getAABB() override;
    /**
     * @brief Get the centroid of the axis aligned bounding box
     * @return Centroid of the axis aligned bounding box
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
	 * <b><span style="color: red">WARNING</span>!</b> The returned
	 *  intersection times should be interpreted as follows, where \f$t_*\f$
	 *  is the minimum time and \f$t^*\f$ the maximum:
	 *
	 * If \f$t_* \geq 0, t^* \geq 0\f$ then the ray intersects the box twice,
	 *  one time to enter the box, another time to leave it.
	 *
	 * If \f$t_* < 0, t^* < 0\f$ then the ray does not intersect the
	 *  box, the line will do, but not the ray which is not allowed to
	 *  go backward (see Convex Optimization by Stephen Boyd and Lieven
	 *  Vandenberghe, where a ray is defined as
	 *  \f$\left\{o + tv : t \geq 0\right\}\f$).
	 *
	 * If either \f$t_* < 0\f$ or \f$t^* < 0\f$ then the ray intersects the
	 *  box once, it starts inside the box and the intersection occurs when
	 *  the ray leaves the box.
	 *
	 * If the returned vector is empty, then there is no line-box intersection
	 *  at all.
	 *
	 *
	 * @see Primitive::getRayIntersection
	 */
	std::vector<double> getRayIntersection(
	    const glm::dvec3& rayOrigin,
	    const glm::dvec3& rayDir
    ) override;
	/**
	 * @see Primitive::getRayIntersectionDistance
	 */
    double getRayIntersectionDistance(
        const glm::dvec3& rayOrigin,
        const glm::dvec3& rayDir
    )override;
    /**
     * @see Primitive::getNumVertices
     */
    size_t getNumVertices() override {return 2;}
    /**
     * @see Primitive::getVertices
     */
	Vertex* getVertices() override;

	/**
	 * @brief Build an axis aligned bounding box containing given primitives
	 * @param primitives Primitives to build axis aligned bounding box around
	 * @return Axis aligned bounding box containing given primitives
	 */
	static std::shared_ptr<AABB> getForPrimitives(
	    std::vector<Primitive*> & primitives
    );
	/**
	 * @brief Build an axis aligned bounding box containing given vertices
	 * @param verts Vertices to build axis aligned bounding box around
	 * @return Axis aligned bounding box containing given vertices
	 */
	static std::shared_ptr<AABB> getForVertices(
	    std::vector<Vertex> & verts
    );
	/**
	 * @brief Build an axis aligned bounding box containing given vertices
	 * @param verts Vertices to build axis aligned bounding box around
	 * @return Axis aligned bounding box containing given vertices
	 */
	static std::shared_ptr<AABB> getForVertices(
	    std::unordered_set<Vertex *, VertexKeyHash, VertexKeyEqual> & verts
    );
	/**
	 * @see Primitive::update
	 */
	void update() override {}

	/**
	 * @brief Obtain size along each axis for the axis aligned bounding box
	 * @return Size along each axis for the axis aligned bounding box
	 */
	glm::dvec3 getSize();
	/**
	 * @brief Bbuild a string representing the axis aligned bounding box
	 * @return String representing the axis aligned bounding box
	 */
	std::string toString();
};

namespace boost {
	namespace serialization {

		template<class Archive>
		inline void save_construct_data(
			Archive & ar, const AABB * t, const unsigned int file_version)
		{
			// save data required to construct instance
			ar << t->vertices[0].pos;
			ar << t->vertices[1].pos;
		}

		template<class Archive>
		inline void load_construct_data(
			Archive & ar, AABB * t, const unsigned int file_version)
		{
			// retrieve data from archive required to construct new instance
			glm::dvec3 min, max;
			ar >> min;
			ar >> max;

			// invoke inplace constructor to initialize instance of my_class
			::new(t)AABB(min, max);
		}
	}
}