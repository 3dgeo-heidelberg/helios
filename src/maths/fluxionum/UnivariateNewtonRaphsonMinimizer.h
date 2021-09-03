#include <Minimizer.h>

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
    }

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
        vector<function<OT(IT)> df
    ) : DiffMinimizer(f, df) {}
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
    ) : DiffMinimizer(f, df(vector<function<OT(IT)>{df, d2f})) {}
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
    IT argmin() override;
};

}