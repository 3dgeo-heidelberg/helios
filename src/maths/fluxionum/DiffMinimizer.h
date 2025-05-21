#pragma once

#include <Minimizer.h>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/void_cast.hpp>

#include <functional>
#include <vector>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base abstract class providing basic structure for minimization
 *  optimization of a given function based on its derivatives.
 *
 * @tparam IT Type of input for the function to be minimized and its
 *  derivatives
 * @tparam OT Type of output for the function to be minimized and its
 *  derivatives
 *
 * @see fluxionum::Minimizer
 */
template<typename IT, typename OT>
class DiffMinimizer : public Minimizer<IT, OT>
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize the differential minimizer to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the differential minimizer
   */
  template<typename Archive>
  void serialize(Archive& ar, unsigned int const version)
  {
    boost::serialization::void_cast_register<DiffMinimizer, Minimizer>();
    ar& boost::serialization::base_object<Minimizer>(*this);
    ar & df;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The derivatives of the function to be minimized such that
   *  df[i] corresponds with \f$\frac{d^if}{dx^i}\f$
   */
  std::vector<std::function<OT(IT)>> df;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Differential minimizer default constructor
   * @see fluxionum::DiffMinimizer::df
   */
  DiffMinimizer(std::function<OT(IT)> f, std::vector<std::function<OT(IT)>> df)
    : Minimizer<IT, OT>(f)
    , df(df)
  {
  }
  virtual ~DiffMinimizer() = default;

  // ***  GETTERs and SETTERs *** //
  // **************************** //
  /**
   * @brief Obtain the derivatives of the function to be minimized
   * @return Derivatives of the function to be minimized
   * @see fluxionum::DiffMinimizer::df
   */
  virtual std::vector<std::function<OT(IT)>> getDerivatives() const
  {
    return df;
  }
  /**
   * @brief Set the derivatives of the function to be minimized
   * @param df New vector of derivatives of the function to be minimized
   * @see fluxionum::DiffMinimizer::df
   */
  virtual void setDerivatives(std::vector<std::function<OT(IT)>> df)
  {
    this->df = df;
  }
  /**
   * @brief Obtain the number of available derivatives for the function to be
   *  minimized
   * @return Number of available derivatives for the function to be minimized
   * @see fluxionum::DiffMinimizer::df
   */
  virtual std::size_t numDerivatives() const { return df.size(); }
  /**
   * @brief Obtain the i-th derivative for the function being minimized
   * @param i Index of the derivative to be obtained
   * @return i-th derivative for the function being minimized
   * @see fluxionum::DiffMinimizer::df
   */
  virtual std::function<OT(IT)> getDerivative(std::size_t const i)
  {
    return df[i];
  }
  /**
   * @brief Set the i-th derivative for the function being minimized
   * @param i Index of the derivative to be setted
   * @param df New i-th derivative for the function being minimized
   * @see fluxionum::DiffMinimizer::df
   */
  virtual void setDerivative(std::size_t const i, std::function<OT(IT)> df)
  {
    this->df[i] = df;
  }
  /**
   * @brief Remove the i-th derivative for the function being minimized
   * @param i Index of the derivative to be removed
   * @see fluxionum::DiffMinimizer::df
   */
  virtual void removeDerivative(std::size_t const i)
  {
    df.erase(df.begin() + i);
  }
  /**
   * @brief Append given derivative
   * @see fluxionum::DiffMinimizer::df
   */
  virtual void addDerivative(std::function<OT(IT)> df)
  {
    this->df.push_back(df);
  }
};

}
