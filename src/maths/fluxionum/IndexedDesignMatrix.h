#ifndef _FLUXIONUM_INDEXED_DESIGN_MATRIX_H_

#include <filems/read/core/DesignMatrixReader.h>
#include <fluxionum/DesignMatrix.h>
#include <fluxionum/FluxionumTypes.h>
#include <fluxionum/TemporalDesignMatrix.h>

#include <memory>
#include <vector>

namespace fluxionum {

using std::make_shared;
using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * An indexed design matrix is simply a matrix which values are ordered by
 *  its indices so it is known that the \f$i\f$-th row precedes the
 *  \f$i+1\f$ row. It can be seen as the discrete version of a temporal design
 *  matrix.
 *
 * Obtaining a DiffDesignMatrix from an IndexedDesignMatrix is not possible
 *  without an assumption about the time domain. For instance, it can be
 *  assumed that indices are linearly spaced inside a normalized time interval
 *  \f$t \in [0, 1]\f$. However, this assumption must accurately pair with
 *  the reality of data. Otherwise, the generated DiffDesignMatrix will be
 *  wrong. It is also possible that there is no useful time domain
 *  interpretation for the indices, thus there might be some
 *  IndexedDesignMatrix for which it is not possible to have a corresponding
 *  DiffDesignMatrix.
 *
 * @tparam IndexType The type of index or index domain
 * @tparam VarType The non index domain (basic DesignMatrix domain)
 * @see fluxionum::TemporalDesignMatrix
 * @see fluxionum::DiffDesignMatrix
 * @see fluxionum::DesignMatrix
 */
template<typename IndexType, typename VarType>
class IndexedDesignMatrix : public DesignMatrix<VarType>
{
protected:
  // ***  USING  *** //
  // *************** //
  using DesignMatrix<VarType>::X;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The indices vector
   *  \f$\vec{\iota} = \left(\iota_1, \ldots, \iota_m\right)
   *      \in \mathbb{Z}^{m}\f$.
   *
   * Note indices are explicitly defined. Thus, instead of having the
   *  \f$i\f$-th row from DesignMatrix \f$X\f$ as implicitly corresponding
   *  to the index \f$i\f$, the \f$i\f$-th row corresponds to the
   *  \f$\iota_i\f$ index. It is done this way since obtaining a
   *  DiffDesignMatrix requires some assumption on the relationship between
   *  indices and time domain. For instance, in the case of linearly
   *  spaced indices in a time domain \f$t \in [0, 1]\f$, explicitly
   *  specifying the indices makes it possible to have the time difference
   *  between \f$i\f$ and \f$i+1\f$ rows of
   *  \f$X \in \mathbb{R}^{m \times n}\f$ to be
   *  \f$(\iota_{i+1} - \iota_{i}) {\Delta t}\f$ where \f${\Delta t}\f$ is
   *  the time step between consecutive indices.
   */
  vector<IndexType> indices;
  /**
   * @brief The name of the index field in the original DesignMatrix
   *
   * By default, it is "index"
   */
  string indexName;

public:
  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Do a copy of the indices column from given DesignMatrix \f$X\f$
   * @param X The DesignMatrix \f$X\f$ containing a column of indices
   * @param indicesColumnIndex The index of the indices column in given
   *  DesignMatrix \f$X\f$
   * @return The copy of the indices column from DesignMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   */
  static inline vector<IndexType> extractIndices(
    arma::Mat<VarType> const& X,
    size_t const indicesColumnIndex)
  {
    arma::Col<VarType> inds = X.col(indicesColumnIndex);
    size_t const m = inds.n_rows;
    vector<IndexType> indices(inds.n_rows);
    for (size_t i = 0; i < m; ++i)
      indices[i] = (IndexType)inds[i];
    return indices;
  }

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build an IndexedDesignMatrix from given DesignMatrix and
   *  specified indices column
   * @param designMatrix The full design matrix, including indices
   * @param indicesColumnIndex Index of the column containing indices
   * @param indexName The default name for the index attribute to be used in
   *  case the given DesignMatrix does not specify a column name for its
   *  indices column
   */
  IndexedDesignMatrix(DesignMatrix<VarType> const& designMatrix,
                      size_t const indicesColumnIndex,
                      string const indexName = "index",
                      vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(
        TemporalDesignMatrix<double, VarType>::extractNonTimeMatrix(
          designMatrix.getX(),
          indicesColumnIndex),
        designMatrix.hasColumnNames()
          ? TemporalDesignMatrix<double, VarType>::extractNonTimeNames(
              designMatrix.getColumnNames(),
              indicesColumnIndex)
          : columnNames)
    , indices(extractIndices(designMatrix.getX(), indicesColumnIndex))
    , indexName(designMatrix.hasColumnNames()
                  ? designMatrix.getColumnName(indicesColumnIndex)
                  : indexName)
  {
  }
  /**
   * @brief Build an IndexedDesignMatrix from given DesignMatrix and indices
   *  vector
   * @param designMatrix The design matrix (with no indices column)
   * @param indices The vector of indices
   * @param indexName The name for the index attribute
   */
  IndexedDesignMatrix(DesignMatrix<VarType> const& designMatrix,
                      vector<IndexType> const& indices,
                      string const indexName = "index",
                      vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(designMatrix)
    , indices(indices)
    , indexName(indexName)
  {
  }
  /**
   * @brief Build an IndexedDesignMatrix from given \f$X\f$ matrix and
   *  specified indices column
   * @param X The matrix containing both data points and its associated index
   * @param indicesColumnIndex Index of the column containing indices
   * @param indexName The name for the index attribute
   */
  IndexedDesignMatrix(arma::Mat<VarType> const& X,
                      size_t const indicesColumnIndex,
                      string const indexName = "index",
                      vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(
        TemporalDesignMatrix<double, VarType>::extractNonTimeMatrix(
          X,
          indicesColumnIndex),
        columnNames)
    , indices(extractIndices(X, indicesColumnIndex))
    , indexName(indexName)
  {
  }
  /**
   * @brief Build an IndexedDesignMatrix from given DesignMatrix \f$X\f$
   *  and indices
   * @param X The DesignMatrix (with no indices column)
   * @param indices The indices
   * @param indexName The name for the index attribute
   */
  IndexedDesignMatrix(arma::Mat<VarType> const& X,
                      vector<IndexType> const& indices,
                      string const indexName = "index",
                      vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(X, columnNames)
    , indices(indices)
    , indexName(indexName)
  {
  }
  /**
   * @brief Build an IndexedDesignMatrix from data in file at given path and
   *  specified index column
   * @param path Path to the file containing both the data and the indices
   * @param indexName The default name for the index attribute to be used in
   *  case the read DesignMatrix does not specify a column name for its
   *  indices column
   */
  IndexedDesignMatrix(string const& path, string const indexName = "index")
  {
    helios::filems::DesignMatrixReader<VarType> reader(path);
    std::unordered_map<string, string> kv;
    DesignMatrix<VarType> const dm = reader.read(&kv);
    size_t const idxCol =
      (size_t)std::strtoul(kv.at("INDEX_COLUMN").c_str(), nullptr, 10);
    *this = IndexedDesignMatrix<IndexType, VarType>(
      dm, idxCol, dm.hasColumnNames() ? dm.getColumnName(idxCol) : indexName);
  }
  virtual ~IndexedDesignMatrix() = default;

  // ***  OPERATORS  *** //
  // ******************* //
  /**
   * @brief Access to the \f$i\f$-th index
   * @param i The index (in the indices vector) of the index being accessed
   * @return Reference to the \f$i\f$-th index
   */
  inline IndexType& operator[](size_t const i) { return indices[i]; }

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @brief Build a DiffDesignMatrix from the IndexedDesignMatrix assuming
   *  that the index 0 lies at \f$t_a\f$ while the maximum index lies at
   *  \f$t_b\f$. All \f$m\f$ indices are expected to be linearly spaced with
   *  a step \f$\Delta t = \frac{t_b - t_a}{m}\f$
   *
   * @param ta The start time
   * @param tb The end time
   * @param diffType The type of differential to be used
   * @return DiffDesignMatrix built from IndexedDesignMatrix
   * @see IndexedDesignMatrix::toLinearTimeDiffDesignMatrixPointer
   * @see fluxionum::DiffDesignMatrix
   */
  DiffDesignMatrix<double, VarType> toLinearTimeDiffDesignMatrix(
    double const ta = 0.0,
    double const tb = 1.0,
    DiffDesignMatrixType diffType =
      DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES) const;
  /**
   * @see Like toLinearTimeDiffDesignMatrix(double const, double const) but
   *  returning a pointer to the object
   * @return Pointer to the DiffDesignMatrix built from IndexedDesignMatrix
   * @see IndexedDesignMatrix::toLinearTimeDiffDesignMatrix
   * @see fluxionum::DiffDesignMatrix
   */
  shared_ptr<DiffDesignMatrix<double, VarType>>
  toLinearTimeDiffDesignMatrixPointer(
    double const ta = 0.0,
    double const tb = 1.0,
    DiffDesignMatrixType diffType =
      DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES) const;

  /**
   * @brief Extend DesignMatrix::mergeInPlace method so the indices are also
   *  merged
   * @param dm The IndexedDesignMatrix to be merged
   */
  void mergeInPlace(DesignMatrix<VarType> const& dm) override
  {
    DesignMatrix<VarType>::mergeInPlace(dm);
    IndexedDesignMatrix const& idm =
      static_cast<IndexedDesignMatrix<IndexType, VarType> const&>(dm);
    indices.insert(
      indices.end(), idm.getIndices().begin(), idm.getIndices().end());
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain a constant/read reference to the vector of indices
   * @return Constant/read reference to the vector of indices
   * @see fluxionum::IndexedDesignMatrix::indices
   */
  inline vector<IndexType> const& getIndices() const { return indices; }
  /**
   * @brief Obtain the name of the index attribute
   * @return The name of the index attribute
   * @see fluxionum::IndexedDesignMatrix::indexName
   */
  inline string const getIndexName() const { return indexName; }
  /**
   * @brief Set the name of the index attribute
   * @param indexName The new name for the index attribute
   * @see fluxionum::IndexedDesignMatrix::indexName
   */
  inline void setIndexName(string const& indexName)
  {
    this->indexName = indexName;
  }
};

}

#define _FLUXIONUM_INDEXED_DESIGN_MATRIX_H_
#include <fluxionum/IndexedDesignMatrix.tpp>
#endif
