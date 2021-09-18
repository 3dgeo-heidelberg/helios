#pragma once

#include <vector>

#include <surfaceinspector/util/Object.hpp>

using std::vector;

using SurfaceInspector::util::Object;

namespace SurfaceInspector { namespace maths{ namespace functions{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @tparam Tin Type of element the function operate over (input)
 * @tparam Tout Type of element the function generates (output)
 * @brief Interface defining math function core mechanics
 */
template <typename Tin, typename Tout>
class IMathFunction : public Object{
public:
    // ***  MATH FUNCTION INTERFACE  *** //
    // ********************************* //
    /**
     * @brief Math function callable. It must be implemented by any concrete
     *  implementation of a valid math function.
     * @param x Function input
     * @return Function output
     */
    virtual Tout operator() (Tin const &x);
    /**
     * @brief RValue reference callable
     * @see IMathFunction::operator() (Tin const &)
     */
    virtual inline Tout operator() (Tin const &&x) {(*this)(x);}

    // ***  MATH FUNCTION BASE  *** //
    // **************************** //
    /**
     * @brief Base implementation for math function computation over a vector.
     *  It can be overridden for the sake of convenience by any subclass.
     * @param u Input vector
     * @return Output vector
     */
    virtual inline vector<Tout> operator() (vector<Tin> const &u)
    {
        vector<Tout> v(0);
        for(Tin const &x : u) v.push_back((*this)(x));
        return v;
    }
};

}}}
