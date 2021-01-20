#pragma once

/**
 * @brief Class representing LAS specification
 *
 * For more information visit:
 *  https://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf
 */
class LasSpecification {
	
	// See https://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Unclassified class integer
     */
	static const int UNCLASSIFIED = 0;
	/**
	 * @brief Unknown class integer
	 */
	static const int UNKNOWN = 1;
	/**
	 * @brief Ground class integer
	 */
	static const int GROUND = 2;
	/**
	 * @brief Low vegetation class integer
	 */
	static const int LOW_VEGETATION = 3;
	/**
	 * @brief Medium vegetation class integer
	 */
	static const int MEDIUM_VEGETATION = 4;
	/**
	 * @brief High vegetation class integer
	 */
	static const int HIGH_VEGETATION = 5;
	/**
	 * @brief Building class integer
	 */
	static const int BUILDING = 6;
	/**
	 * @brief Low point class integer
	 */
	static const int LOW_POINT = 7;
	/**
	 * @brief Water class integer
	 */
	static const int WATER = 9;
	/**
	 * @brief Rail class integer
	 */
	static const int RAIL = 10;
	/**
	 * @brief Road class integer
	 */
	static const int ROAD = 11;
	/**
	 * @brief Wire guard class integer
	 */
	static const int WIRE_GUARD = 13;
	/**
	 * @brief Wire conductor class integer
	 */
	static const int WIRE_CONDUCTOR = 14;
	/**
	 * @brief Transmission tower class integer
	 */
	static const int TRANSMISSION_TOWER = 14;
	/**
	 * @brief Wire structure class integer
	 */
	static const int WIRE_STRUCTURE = 16;
};