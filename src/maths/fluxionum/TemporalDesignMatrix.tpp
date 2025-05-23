// ***  METHODS  *** //
// ***************** //
template <typename TimeType, typename VarType>
fluxionum::DiffDesignMatrix<TimeType, VarType>
fluxionum::TemporalDesignMatrix<TimeType, VarType>::toDiffDesignMatrix(
    DiffDesignMatrixType diffType,
    bool const sort
) const {
    return fluxionum::DiffDesignMatrix<TimeType, VarType>(*this, diffType, sort);
}

template <typename TimeType, typename VarType>
std::shared_ptr<fluxionum::DiffDesignMatrix<TimeType, VarType>>
fluxionum::TemporalDesignMatrix<TimeType, VarType>::toDiffDesignMatrixPointer(
    DiffDesignMatrixType diffType,
    bool const sort
) const {
    return std::make_shared<fluxionum::DiffDesignMatrix<TimeType, VarType>>(
        *this, diffType, sort
    );
}
