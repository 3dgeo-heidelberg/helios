#pragma once

#include <memory>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper to have a vector of pointers such that when it is destroyed,
 *  all the pointers are deleted.
 *
 * The shared vector can be safely accessed by different threads since it
 *  it is implemented to be concurrency-aware.
 *
 * @tparam T The type of elements to which each pointer in the vector refers
 */
template <typename T> class PointerVector {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The vector of pointers
     */
    std::vector<T *> v;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor
     */
    PointerVector() = default;
    /**
     * @brief Constructor that initializes a shared vector to hold for
     *  numElements pointers
     * @param numElements The number of pointers that the vector must be
     *  initialized to contain
     */
    PointerVector(size_t const numElements) {v.reserve(numElements);}
    /**
     * @brief Destructor that deletes what is hold for each pointer in the
     *  pointer vector before deleting the PointerVector itself
     */
    virtual ~PointerVector() {for(T * vi : v) delete vi;}

    // ***  OPERATORS  *** //
    // ******************* //
    /**
     * @brief Access a reference to the vector
     */
    inline std::vector<T *> & operator* () {return v;}
    /**
     * @biref Access the pointer at the \f$i\f$-th position
     * @param i The position of the requested pointer
     * @return Pointer at the \f$i\f$-th position
     */
    inline T * operator[] (size_t const i){return v[i];}
};