#include <boost/serialization/serialization.hpp>

#include <functional>

namespace fluxionum{

using std::function;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base abstract class providing basic structure for minimization
 *  optimization of a given function.
 *
 * Minimizer serves as basis for implementation of analytical minimizers,
 *  numerical minimizers, heuristic minimizers and any minimization-like
 *  optimization. Thus, it is only aware of the mathematical core of any
 *  minimization, which is:
 *
 * \f[
 *  \mathrm{arg min}_{x} \; f(x)
 * \f]
 *
 * @tparam IT Type of input the function to be minimized receives
 * @tparam OT Type of output the function to be minimized gives
 */
template <typename IT, typename OT>
class Minimizer {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize the minimizer to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the minimizer
     */
    template <typename Archive>
    void serialize(Archive &ar, unsigned int const version){
        ar &f;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The function to be minimized
     */
    function<OT(IT)> f;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Minimizer default constructor
     * @param f Function to be minimized
     * @see fluxionum::Minimizer::f<OT(IT)>
     */
    Minimizer(function<OT(IT)> f) : f(f) {}
    virtual ~Minimizer() = default;

    // ***  MINIMIZATION  *** //
    // ********************** //
    /**
     * @brief Find the argument which minimizes minimizer's function
     * @return Argument which minimizes minimizer's function
     * @see fluxionum::Minimizer::f<OT(IT)>
     */
    virtual IT argmin() = 0;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the function to be minimized
     * @return Function to be minimized
     * @see fluxionum::Minimizer::f<OT(IT)>
     */
    virtual function<OT(IT)> getF() const {return f;}
    /**
     * @brief Set the function to be minimized
     * @param f New function to be minimized
     * @see fluxionum::Minimizer::f<OT(IT)>
     */
    virtual void setF(function<OT(IT)> f) {this->f = f;}
};

}