#ifndef _FLUXIONUM_TEMPORAL_DESIGN_MATRIX_H_
#include <fluxionum/TemporalDesignMatrix.h>
#endif
#include <fluxionum/DiffDesignMatrix.h>

using namespace fluxionum;

// ***  METHODS  *** //
// ***************** //
template <typename TimeType, typename VarType>
DiffDesignMatrix<TimeType, VarType>
TemporalDesignMatrix<TimeType, VarType>::toDiffDesignMatrix(
    DiffDesignMatrixType diffType,
    bool const sort
) const {
    return DiffDesignMatrix<TimeType, VarType>(*this, diffType, sort);
}

template <typename TimeType, typename VarType>
shared_ptr<DiffDesignMatrix<TimeType, VarType>>
TemporalDesignMatrix<TimeType, VarType>::toDiffDesignMatrixPointer(
    DiffDesignMatrixType diffType,
    bool const sort
) const {
    return make_shared<DiffDesignMatrix<TimeType, VarType>>(
        *this, diffType, sort
    );
}

