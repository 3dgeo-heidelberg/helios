#ifndef _UNIVARIATENEWTONRAHPSONMINIMIZER_H_
#define _UNIVARIATENEWTONRAHPSONMINIMIZER_H_

#include <DiffMinimizer.h>
#include <IterativeMethodHandler.h>

#include <boost/serialization/serialization.hpp>

namespace fluxionum{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Implementation of univariate Newton-Raphson minimizer
 *
 * @tparam IT Type of input for the function to be minimized and its first
 *  and second derivative
 * @tparam OT Type of output for the function to be minimized and its first
 *  and second derivative
 */
template <typename IT, typename OT>
class UnivariateNewtonRaphsonMinimizer : public DiffMinimizer<IT, OT>{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize the univariate Newton-Raphson minimizer to a stream
     *  of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the univariate Newton-Raphson
     *  minimizer
     */
    template <typename Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            UnivariateNewtonRaphsonMinimizer,
            DiffMinimizer
        >();
        ar &boost::serialization::base_object<
            DiffMinimizer
        >(*this);
        ar &imh;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Iterative method handler for the univariate Newton-Raphson
     *  minimization
     * @see fluxion::IterativeMethodHandler
     */
    IterativeMethodHandler<IT, OT> imh;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for univariate Newton-Raphson minimizer
     * @param f Univariate function to be minimized
     * @param df First and second derivatives of the function to be minimized
     * @see fluxionum::DiffMinimizer::DiffMinimizer
     */
    UnivariateNewtonRaphsonMinimizer(
        function<OT(IT)> f,
        vector<function<OT(IT)>> df
    ) :
        DiffMinimizer<IT, OT>(f, df),
        imh(IterativeMethodHandler<IT, OT>(
            1000,
            false,
            0.000000001,
            3,
            0.000000001,
            true
        ))
    {}
    /**
     * @brief Alternative constructor for univariate Newton-Raphson minimizer
     * @param f Univariate function to be minimized
     * @param df First derivative of the function to be minimized
     * @param d2f Second derivative of the function to be minimized
     * @see fluxionum::DiffMinimizer::DiffMinimizer
     */
    UnivariateNewtonRaphsonMinimizer(
        function<OT(IT)> f,
        function<OT(IT)> df,
        function<OT(IT)> d2f
    ) :
        DiffMinimizer<IT, OT>(f, vector<function<OT(IT)>>({df, d2f})),
        imh(IterativeMethodHandler<IT, OT>(
            1000,
            false,
            0.000000001,
            3,
            0.000000001,
            true
        ))
    {}
    virtual ~UnivariateNewtonRaphsonMinimizer() = default;

    // ***  MINIMIZATION  *** //
    // ********************** //
    /**
     * @brief Implementation of the univariate Newton-Raphson minimization
     *
     * \f[
     *  x_{k+1} = x_k - \frac{f'(x_k)}{f''(x_k)} =
     *      x_k - \frac{df}{dx}(x_k) \left[
     *          \frac{d^2f}{dx^2}(x_k)
     *      \right]^{-1}
     * \f]
     * @see fluxionum::Minimizer::argmin
     */
    IT argmin(IT x) override;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the iterative method handler
     * @return Iterative method handler
     * @see fluxionum::UnivariateNewtonRaphsonMinimizer::imh
     */
    IterativeMethodHandler<IT, OT> & getIterativeMethodHandler() {return imh;}
    /**
     * @brief Set the iterative method handler
     * @param imh New iterative method handler
     * @see fluxionum::UnivariateNewtonRaphsonMinimizer::imh
     */
    void setIterativeMethodHandler(
        IterativeMethodHandler<IT, OT> const &imh
    )
    {this->imh = imh;}
};


}
#endif

#include <UnivariateNewtonRaphsonMinimizer.tpp>
