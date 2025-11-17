// ***  METHODS  *** //
// ***************** //
template<typename IndexType, typename VarType>
fluxionum::DiffDesignMatrix<double, VarType>
fluxionum::IndexedDesignMatrix<IndexType, VarType>::
  toLinearTimeDiffDesignMatrix(double const ta,
                               double const tb,
                               fluxionum::DiffDesignMatrixType diffType) const
{
  return fluxionum::TemporalDesignMatrix<double, VarType>(
           X, arma::linspace(ta, tb, X.n_rows))
    .toDiffDesignMatrix(diffType);
}

template<typename IndexType, typename VarType>
std::shared_ptr<fluxionum::DiffDesignMatrix<double, VarType>>
fluxionum::IndexedDesignMatrix<IndexType, VarType>::
  toLinearTimeDiffDesignMatrixPointer(
    double const ta,
    double const tb,
    fluxionum::DiffDesignMatrixType diffType) const
{
  return fluxionum::TemporalDesignMatrix<double, VarType>(
           X, arma::linspace(ta, tb, X.n_rows))
    .toDiffDesignMatrixPointer(diffType);
}
