#ifndef _SURFACEINSPECTOR_MATHS_PERMUTERS_CNRINDEXPERMUTER_HPP_
#define _SURFACEINSPECTOR_MATHS_PERMUTERS_CNRINDEXPERMUTER_HPP_

#include <surfaceinspector/maths/permuters/IPermuter.hpp>

using SurfaceInspector::maths::permuters::IPermuter;

namespace SurfaceInspector { namespace maths{ namespace permuters {

/**
 * @brief Permuter of Combinations with No Repetition (CNR) for indices
 *
 * @tparam T Must be an integer type (int, long, size_t, ...) because it is
 *  going to be used to represent an index
 */
template <typename T>
class CNRIndexPermuter : public IPermuter<T>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many indices there are, so it defines a range \f$[0, n)\f$
     */
    T n;
    /**
     * @brief How many indices are selected for each permutation
     */
    T k;
    /**
     * @brief How many combinations without repetitions,
     *  \f${{n}{\choose{k}}\f$, there exist
     */
    T max;
    /**
     * @brief Index of current permutation
     */
    T current;
    /**
     * @brief Current indices permutation
     */
    vector<T> indices;
    /**
     * @brief Index to be used as stop condition
     */
    T stopIndex;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a permuter for combinations with no repetitions of \f$n\f$
     *  indices selecting \f$k\f$
     * @param n How many indices, so they will range in interval \f$[0, n)\f$
     * @param k How many selections
     */
    CNRIndexPermuter(T n, T k);
    virtual ~CNRIndexPermuter() = default;

    // ***  IPERMUTER  *** //
    // ******************* //
    /**
     * @see IPermuter::start
     */
    inline void start() {indices.clear(); current=0;}
    /**
     * @see IPermuter::hasNext
     */
    bool hasNext() {return current < max;}
    /**
     * @see IPermuter::next
     */
    vector<T> next();
    /**
     * @see IPermuter::get
     */
    inline vector<T> get() {return indices;}
};

}}}

#include <maths/permuters/CNRIndexPermuter.tpp>

#endif
