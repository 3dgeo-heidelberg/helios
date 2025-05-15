#ifndef _FLUXIONUM_INDEXED_DESIGN_MATRIX_H_
#include <fluxionum/IndexedDesignMatrix.h>
#endif
#include <fluxionum/TemporalDesignMatrix.h>
#include <fluxionum/DiffDesignMatrix.h>



// ***  METHODS  *** //
// ***************** //
template <typename IndexType, typename VarType>
DiffDesignMatrix<double, VarType>
IndexedDesignMatrix<IndexType, VarType>::toLinearTimeDiffDesignMatrix(
    double const ta,
    double const tb,
    DiffDesignMatrixType diffType
) const {
    return TemporalDesignMatrix<double, VarType>(
        X,
        arma::linspace(ta, tb, X.n_rows)
    ).toDiffDesignMatrix(diffType);
}

template <typename IndexType, typename VarType>
shared_ptr<DiffDesignMatrix<double, VarType>>
IndexedDesignMatrix<IndexType, VarType>::toLinearTimeDiffDesignMatrixPointer(
    double const ta,
    double const tb,
    DiffDesignMatrixType diffType
) const {
    return TemporalDesignMatrix<double, VarType>(
        X,
        arma::linspace(ta, tb, X.n_rows)
    ).toDiffDesignMatrixPointer(diffType);
}
