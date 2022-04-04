#pragma once

#include <fluxionum/DiffDesignMatrix.h>
#include <fluxionum/LinearPiecesFunction.h>
#include <fluxionum/ParametricLinearPiecesFunction.h>
#include <fluxionum/FluxionumException.h>

#include <armadillo>

namespace fluxionum { namespace DiffDesignMatrixInterpolator{

// ***  MAKE METHODS  *** //
// ********************** //
/**
 * @brief Obtain a linear pieces function from given DiffDesignMatrix and
 *  known values
 * @param ddm The DiffDesignMatrix itself
 * @param y The vector of known values such that \f$y(t_i) = y_i\f$
 * @param colIdx The index of the column to generate LinearPiecesFunction from
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return LinearPiecesFunction from given arguments
 */
template <typename A, typename B>
LinearPiecesFunction<A, B> makeLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Col<B> const &y,
    size_t const colIdx=0
){
    return LinearPiecesFunction<A, B>(
        ddm.getTimeVector(),
        ddm.getA().col(colIdx),
        y
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
    size_t const colIdx=0
){
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            return makeLinearPiecesFunction(
                ddm,
                arma::Col<double>(
                    dm.getColumnCopy(colIdx).subvec(0, dm.getNumRows()-2)
                ),
                colIdx
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
 * @param y The matrix of known values such that \f$y_j(t_i) = y_{ij}\f$
 * @param colIdx The index of the column to generate
 *  ParametricVectorialLinearPiecesFunction from
 * @tparam A The time's domain
 * @tparam B The non time's domain
 * @return ParametricVectorialLinearPiecesFunction from given arguments
 */
template <typename A, typename B>
ParametricLinearPiecesFunction<A, B> makeParametricLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    arma::Mat<B> const &y
){
    return ParametricLinearPiecesFunction<A, B>(
        ddm.getTimeVector(),
        ddm.getA(),
        y
    );
}

template <typename A, typename B>
ParametricLinearPiecesFunction<A, B> makeParametricLinearPiecesFunction(
    DiffDesignMatrix<A, B> const &ddm,
    DesignMatrix<B> const &dm
){
    switch(ddm.getDiffType()){
        case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
            return makeParametricLinearPiecesFunction(
                ddm,
                arma::Mat<double>(dm.getX().rows(0, dm.getNumRows()-2))
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

}}