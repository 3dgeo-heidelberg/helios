#pragma once

#include <fluxionum/DiffDesignMatrix.h>
#include <fluxionum/LinearPiecesFunction.h>
#include <fluxionum/ParametricLinearPiecesFunction.h>
#include <fluxionum/FixedIterativeEulerMethod.h>
#include <fluxionum/FixedParametricIterativeEulerMethod.h>
#include <fluxionum/ClosestLesserSampleFunction.h>
#include <fluxionum/ParametricClosestLesserSampleFunction.h>
#include <fluxionum/FluxionumException.h>

#include <armadillo>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Util methods to interpolate functions from given DiffDesignMatrix
 */
namespace DiffDesignMatrixInterpolator{

// ***  MAKE METHODS  *** //
// ********************** //
/**
 * @brief Obtain a linear pieces function from given DiffDesignMatrix and
 *  known values
 * @param ddm The DiffDesignMatrix itself
 * @param slope The vector of known derivatives (line slopes)
 *  \f$\frac{dy}{dt}(t_i)\f$
 * @param intercept The vector of known values such that \f$y(t_i) = y_i\f$
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return LinearPiecesFunction from given arguments
 */
template <typename A, typename B>
LinearPiecesFunction<A, B> makeLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Col<B> const &slope,
    arma::Col<B> const &intercept
){
    return LinearPiecesFunction<A, B>(
        ddm.getTimeVector(),
        slope,
        intercept
    );
}

/**
 * @brief Like DiffDesignMatrixInterpolator::makeLinearPiecesFunction but
 *  taking the vector of known values from given DesignMatrix
 * @see fluxionum::DiffDesignMatrixInterpolator::makeLinearPiecesFunction
 */
template <typename A, typename B>
LinearPiecesFunction<A, B> makeLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    DesignMatrix<B> const &dm,
    size_t const colIdx,
    arma::Col<B> *intercept,
    arma::Col<B> *slope
){
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            *intercept = dm.getColumnCopy(colIdx).subvec(0, dm.getNumRows()-2);
            *slope = ddm.getA().col(colIdx);
            return makeLinearPiecesFunction(
                ddm,
                *slope,
                *intercept
            );
        }
        case DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::makeLinearPiecesFunction:\n"
                "\tCentral finite differences not supported"
            );
        }
        default:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::makeLinearPiecesFunction:\n"
                "\tUnexpected differential type"
            );
        }
    }
}

/**
 * @brief Obtain a parametric linear pieces function from given
 *  DiffDesignMatrix and known values
 * @param ddm  The DiffDesignMatrix itself
 * @param intercepts The matrix of known values such that
 *  \f$y_j(t_i) = y_{ij}\f$
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return ParametricVectorialLinearPiecesFunction from given arguments
 */
template <typename A, typename B>
ParametricLinearPiecesFunction<A, B> makeParametricLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Mat<B> const &intercepts
){
    return ParametricLinearPiecesFunction<A, B>(
        ddm.getTimeVector(),
        ddm.getA(),
        intercepts
    );
}

/**
 * @brief Like DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction
 *  but taking the vector of known values from given DesignMatrix
 * @see DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction
 */
template <typename A, typename B>
ParametricLinearPiecesFunction<A, B> makeParametricLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    DesignMatrix<B> const &dm
){
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            return makeParametricLinearPiecesFunction(
                ddm,
                dm.getX()
            );
        }
        case DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeParametricLinearPiecesFunction:\n"
                "\tCentral finite differences not supported"
            );
        }
        default:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeParametricLinearPiecesFunction:\n"
                "\tUnexpected differential type"
            );
        }
    }
}

/**
 * @brief Obtain a function representing a fixed iterative Euler method from
 *  given DiffDesignMatrix, known values and derivative function
 * @param ddm The DiffDesignMatrix itself
 * @param[in] y The vector of known values such that \f$y(t_i) = y_i\f$
 * @param[in] dydt The derivative function
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return FixedIterativeEulerMethod from given arguments
 */
template <typename A, typename B>
FixedIterativeEulerMethod<A, B> makeFixedIterativeEulerMethod(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Col<B> const &y,
    Function<A, B> &dydt
){
    return FixedIterativeEulerMethod<A, B>(
        dydt,
        ddm.getTimeVector()(0),
        y(0),
        ddm.getTimeVector(),
        y,
        0
    );
}

/**
 * @brief Like DiffDesignMatrixInterpolator::makeFixedIterativeEulerMethod but
 *  automatically generating the vector of known values, the derivative and
 *  the samples of the derivative from given DiffDesignMatrix and DesignMatrix
 * @see DiffDesignMatrixInterpolator::makeFixedIterativeEulerMethod
 */
template <typename A, typename B>
FixedIterativeEulerMethod<A, B> makeFixedIterativeEulerMethod(
    DiffDesignMatrix<A, B> const &ddm,
    DesignMatrix<B> const &dm,
    size_t const colIdx,
    arma::Col<B> *y,
    ClosestLesserSampleFunction<A, B> *dydt,
    arma::Col<B> *dydtSamples
){
    *y = arma::Col<B>(dm.getColumn(colIdx).subvec(0, dm.getNumRows()-2));
    *dydtSamples = ddm.getA().col(colIdx);
    *dydt = ClosestLesserSampleFunction<A, B>(
        ddm.getTimeVector(),
        *dydtSamples,
        0
    );
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            return makeFixedIterativeEulerMethod(
                ddm,
                *y,
                *dydt
            );
        }
        case DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeFixedIterativeEulerMethod:\n"
                "\tCentral finite differences not supported"
            );
        }
        default:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeFixedIterativeEulerMethod:\n"
                "\tUnexpected differential type"
            );
        }
    }
}

/**
 * @brief Obtain a function representing a fixed parametric iterative Euler
 *  method from given DiffDesignMatrix, known values and parametric derivative
 *  function
 * @param ddm The DiffDesignMatrix itself
 * @param[in] y The matrix of known values such that
 *  \f$\vec{y}(t_i) = \vec{y_i}\f$a
 * @param dydt The parametric derivative function
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return FixedParametricIterativeEulerMethod from given arguments
 */
template <typename A, typename B>
FixedParametricIterativeEulerMethod<A, B>
makeFixedParametricIterativeEulerMethod(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Mat<B> const &y,
    Function<A, arma::Col<B>> &dydt
){
    return FixedParametricIterativeEulerMethod<A, B>(
        dydt,
        ddm.getTimeVector()(0),
        y.row(0).as_col(),
        ddm.getTimeVector(),
        y,
        0
    );
}

/**
 * @brief Like
 *  DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod but
 *  automatically generating the matrix of known values, the parametric
 *  derivative and the samples of the derivative from given DiffDesignMatrix
 *  and DesignMatrix
 * @see DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod
 */
template <typename A, typename B>
FixedParametricIterativeEulerMethod<A, B>
makeFixedParametricIterativeEulerMethod(
    DiffDesignMatrix<A, B> const &ddm,
    DesignMatrix<B> const &dm,
    ParametricClosestLesserSampleFunction<A, B> *dydt
){
    *dydt = ParametricClosestLesserSampleFunction<A, B>(
        ddm.getTimeVector(),
        ddm.getA(),
        0
    );
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            return makeFixedParametricIterativeEulerMethod(
                ddm,
                dm.getX(),
                *dydt
            );
        }
        case DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeFixedParametricIterativeEulerMethod:\n"
                "\tCentral finite differences not supported"
            );
        }
        default:{
            throw FluxionumException(
                "DiffDesignMatrixInterpolator::"
                "makeFixedParametricIterativeEulerMethod:\n"
                "\tUnexpected differential type"
            );
        }
    }
}


}}