#pragma once

#include "Primitive.h"

/**
 * @brief KDTree primitive comparator
 */
struct KDTreePrimitiveComparator {
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Index of axis considered for the comparison
     */
	int axis;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for KDTree primitive comparator
     * @param axis
     */
	explicit KDTreePrimitiveComparator(int axis) {
		this->axis = axis;
	}

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief KDTree primitive comparator functor
	 * @param a First primitive for the comparison
	 * @param b Second primitive for the comparison
	 * @return True of coordinate of primitive a at corresponding axis is
	 *  greater than coordinate of primitive b at the same axis, false
	 *  otherwise
	 */
	bool operator () (Primitive* a, Primitive* b) {
		auto ax = a->getCentroid()[axis];
		auto bx = b->getCentroid()[axis];
		if (ax > bx) return true;
		return false;
	}
};