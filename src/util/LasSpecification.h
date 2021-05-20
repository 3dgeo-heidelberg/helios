#pragma once

/**
 * @brief Class representing LAS specification
 * Classes from 0 to 63 are defined by the specification
 * Classes from 64 to 255 are user definable.
 *
 * For more information visit:
 *  http://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf
 */
class LasSpecification {


  // See http://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf
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
	static const int TRANSMISSION_TOWER = 15;
	/**
	 * @brief Wire structure class integer
	 */
	static const int WIRE_STRUCTURE = 16;
        /**
         * @brief Bridge Deck class integer
         */
        static const int BRIDGE_DECK = 17;
        /**
         * @brief High Noise class integer
         */
        static const int HIGH_NOISE = 18;
        /**
         * @brief Overhead Structure class integer
         * E.g. conveyors, mining equipment, traffic lights
         */
        static const int OVERHEAD_STRUCTURE = 19;
        /**
         * @brief Ignored ground class integer
         * E.g. breakline proximity
         */
        static const int IGNORED_GROUND = 20;
        /**
         * @brief Snow class integer
         */
        static const int SNOW = 21;

};