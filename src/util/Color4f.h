#pragma once

/**
 * @brief Class representing a color with 4 float components: RGBA
 */
class Color4f {
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)	{
		ar & x & y & z & w;
	}
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Color red component (R)
     */
	float x;
	/**
	 * @brief Color green component (G)
	 */
	float y;
	/**
	 * @brief Color blue component (B)
	 */
	float z;
	/**
	 * @brief Color alpha component (A)
	 */
	float w;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
    /**
     * @brief Color in 4 float components default constructor
     */
    Color4f() = default;

	/**
	 * @brief Color in 4 float components constructor
	 * @see Color4f::x
	 * @see Color4f::y
	 * @see Color4f::z
	 * @see Color4f::w
	 */
	Color4f(float x, float y, float z, float w) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}

};