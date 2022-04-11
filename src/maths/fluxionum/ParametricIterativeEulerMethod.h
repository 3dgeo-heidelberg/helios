#pragma once

#include <fluxionum/Function.h>

#include <armadillo>

namespace fluxionum{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Parametric iterative Euler method
 *
 * \f[
 *  \vec{y}(t+h) = \vec{y}(t) + h \frac{d}{dt}\vec{y}(t)
 * \f]
 *
 * The parametric iterative Euler method computes the Euler method for
 *  independent vector components assuming at each iteration the given step.
 *  It has an error order \f$Ch\f$ where \f$h\f$ is the step size.
 *
 * @see fluxionum::Function
 * @see fluxionum::IterativeEulerMethod
 */
template <typename A, typename B>
class ParametricIterativeEulerMethod : public Function<A, arma::Col<B>>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Reference to the parametric derivative function
     * \f[
     *  \frac{d}{dt}\vec{y}(t)
     * \f]
     * @see fluxionum::ParametricIterativeEulerMethod::y
     * @see fluxionum::ParametricIterativeEulerMethod::t
     */
    Function<A, arma::Col<B>> &dydt;
    /**
     * @brief The initial value of \f$t\f$, \f$t_0\f$
     * @see fluxionum::ParametricIterativeEulerMethod::y0
     */
    A t0;
    /**
     * @brief The current value of \f$t\f$
     * @see fluxionum::ParametricIterativeEulerMethod::y
     * @see fluxionum::ParametricIterativeEulerMethod::dydt
     */
    A t;
    /**
     * @brief The initial value of \f$\vec{y}\f$, \f$\vec{y}(t_0)\f$
     * @see fluxionum::ParametricIterativeEulerMethod::t0
     */
    arma::Col<B> y0;
    /**
     * @brief The current value of \f$\vec{y}(t)\f$
     * @see fluxionum::ParametricIterativeEulerMethod::t
     * @see fluxionum::ParametricIterativeEulerMethod::dydt
     */
    arma::Col<B> y;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ParametricIterativeEulerMethod default constructor
     * @see fluxionum::ParametricIterativeEulerMethod::dydt
     * @see fluxionum::ParametricIterativeEulerMethod::t0
     * @see fluxionum::ParametricIterativeEulerMethod::y0
     */
    ParametricIterativeEulerMethod(
        Function<A, arma::Col<B>> &dydt,
        A const &t0,
        arma::Col<B> const &y0
    ):
        Function<A, arma::Col<B>>(),
        dydt(dydt),
        t0(t0),
        t(t0),
        y0(y0),
        y(y0)
    {}
    virtual ~ParametricIterativeEulerMethod() = default;

    // ***  FUNCTION METHODS  *** //
    // ************************** //
    /**
     * @brief Iteratively compute the next value using Euler method.
     *
     * It is assumed that the instance knows the current time, the current
     *  value and the current derivative. Thus, it can approximate the next
     *  value.
     *
     * Calling this method updates the internal status of the
     *  ParametricIterativeEulerMethod so it is representing then ew state at
     *  \f$t+h\f$
     *
     * @param h The step size
     * @return The computed vector
     */
    arma::Col<B> eval(A const &h) override{
        y = y + h*dydt(t);
        t = t+h;
        return y;
    }

    /**
     * @brief Restart the ParametricIterativeEulerMethod so it is at its
     *  initial state again \f$y(t_0) = y_0\f$
     */
    virtual void restart(){
        setT(getT0());
        setY(getY0());
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::dydt
     */
    inline Function<A, B> const& getDydt() const {return dydt;};
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::t
     */
    inline A getT() const {return t;};
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::t
     */
    inline void setT(A const t) {this->t = t;}
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::t0
     */
    inline A getT0() const {return t0;}
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::y
     */
    inline arma::Col<B> getY() const {return y;}
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::y
     */
    inline void setY(arma::Col<B> const y) {this->y = y;}
    /**
     * @see fluxionum::ParametricIterativeEulerMethod::y0
     */
    inline arma::Col<B> getY0() const {return y0;}
};

}