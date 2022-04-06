#pragma once

#include <fluxionum/IterativeEulerMethod.h>

#include <armadillo>

namespace fluxionum{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Fixed iterative Euler method
 *
 * It is like the IterativeEulerMethod, but the values at the frontiers are
 *  given and thus can be used to fix/correct the function value at those
 *  points. This helps preserving the stability of the approximation, specially
 *  for those cases which have a significant number of frontiers
 *
 * @see fluxionum::IterativeEulerMethod
 * @see fluxionum::FixedParametricIterativeEulerMethod
 * @see fluxionum::Function
 */
template <typename A, typename B>
class FixedIterativeEulerMethod : public IterativeEulerMethod<A, B>{
protected:
    // ***  USING  *** //
    // *************** //
    using IterativeEulerMethod<A, B>::t;
    using IterativeEulerMethod<A, B>::y;

    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The \f$m\f$ frontiers such that
     *  \f$\forall t,\, \exists i \ni t \in [a_i, a_{i+1})\f$.
     * For the last frontier, the interval is \f$[a_{m}, \infty)\f$.
     */
    arma::Col<A> const &ta;
    /**
     * @brief The value of \f$y(t_i)\f$ at each of the \f$m\f$ frontiers such
     *  that \f$\forall i,\, y_i = y(t_i)\f$
     */
    arma::Col<B> const &ya;
    /**
     * @brief The index of the current piece
     *
     * \f[
     *  i : t_{i} \in [a_{i}, a_{i+1})
     * \f]
     */
    size_t i;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief FixedIterativeEulerMethod default constructor
     * @param i The index of the initial piece for the iterative method. By
     *  default, it is assumed to be 0, it is the first piece.
     * @see fluxionum::IterativeEulerMethod::IterativeEulerMethod
     * @see fluxionum::FixedIterativeEulerMethod::ta
     * @see fluxionum::FixedIterativeEulerMethod::ya
     * @see fluxionum::FixexdIterativeEulerMethod::i
     */
    FixedIterativeEulerMethod(
        Function<A, B> &dydt,
        A const &t0,
        B const &y0,
        arma::Col<A> const &ta,
        arma::Col<B> const &ya,
        size_t const i=0
    ) :
        IterativeEulerMethod<A, B>(dydt, t0, y0),
        ta(ta),
        ya(ya),
        i(i)
    {}
    virtual ~FixedIterativeEulerMethod() = default;

    // ***  FUNCTION METHODS  *** //
    // ************************** //
    /**
     * @brief Compute the iterative Euler method but considering given fixed
     *  frontiers
     * @param h The step between current \f$t_i\f$ and next \f$t_{i+1}\f$ such
     *  that \f$h = t_{i+1} - t_i\f$
     * @see fluxionum::IterativeEulerMethod::eval
     */
    B eval(A const &h) override{
        // Handle fixed frontiers (assuming time is sorted)
        A th = t+h;
        A tdiff = h;
        if((i<ta.n_elem-1) && (th >= ta.at(i+1))){ // Update frontier
            for(size_t j = ta.n_elem-1 ; j > i ; --j){
                if(th >= ta.at(j)){
                    tdiff = h-(ta.at(j)-t);
                    t = ta.at(j);
                    y = ya.at(j);
                    i = j;
                    break;
                }
            }
        }
        // Compute iterative Euler method
        return IterativeEulerMethod<A, B>::eval(tdiff);
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see fluxionum::FixedIterativeEulerMethod::ta
     */
    inline arma::Col<A> const & getTa() const {return ta;}
    /**
     * @see fluxionum::FixedIterativeEulerMethod::ya
     */
    inline arma::Col<B> const & getYa() const {return ya;}
    /**
     * @see fluxionum::FixedIterativeEulerMethod::i
     */
    inline size_t getCurrentPieceIndex() const {return i;}
    /**
     * @see fluxionum::FixedIterativeEulerMethod::i
     */
    inline void setCurrentPieceIndex(size_t const i) {this->i = i;}
};

}