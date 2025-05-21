#pragma once

#include <filems/read/core/DesignMatrixReader.h>
#include <fluxionum/AbstractDesignMatrix.h>

#include <armadillo>

#include <string>
#include <vector>

namespace fluxionum {

using namespace helios::filems;

using std::string;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief This class represents a DesignMatrix
 *  \f$X \in \mathbb{R}^{m \times n}\f$.
 *
 * It is assumed that each row represents a different point while each column
 *  defines a different attribute for the point. Thus, the DesignMatrix can be
 *  interpreted as a set of \f$m\f$ different points defined in a \f$n\f$
 *  dimensional space. The DesignMatrix might even be decoded into a structure
 *  subset (structure space) that defines the structural relationship between
 *  points (i.e. their spatial relationships) and a feature subset (feature
 *  space) that defines the characteristics of each point (for instance, as
 *  descriptors of their local neighborhood).
 *
 * @tparam T Type \f$\mathcal{T}\f$ of element for the design matrix \f$X\f$
 *  such that \f$x_{ij} \in \mathcal{T}\f$.
 * @see fluxionum::AbstractDesignMatrix
 * @see fluxionum::TemporalDesignMatrix
 * @see fluxionum::IndexedDesignMatrix
 */
template<typename T>
class DesignMatrix : public AbstractDesignMatrix<T>
{
public: // Originally it was protected, it has been changed to public
  // ***  USING  *** //
  // *************** //
  using AbstractDesignMatrix<T>::translateColumnNameToIndex;

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The design matrix \f$X \in \mathbb{R}^{m \times n}\f$
   */
  arma::Mat<T> X;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a DesignMatrix with no data
   * @param columnNames Either the name for each column or an empty vector
   *  if there are no names
   */
  DesignMatrix(vector<string> const& columnNames = vector<string>(0))
    : AbstractDesignMatrix<T>(columnNames)
  {
  }
  /**
   * @brief Build a DesignMatrix from given armadillo matrix
   * @param X Armadillo matrix to build DesignMatrix from
   * @param columnNames Either the name for each column or an empty vector
   *  if there are no names
   */
  DesignMatrix(arma::Mat<T> const& X,
               vector<string> const& columnNames = vector<string>(0))
    : AbstractDesignMatrix<T>(columnNames)
    , X(X)
  {
  }
  /**
   * @brief Build a DesignMatrix from data in file at given path
   * @param path Path to the file containing the data for the DesignMatrix
   * @param columnNames The default column names to be used if no column
   *  names are read from file at given path
   */
  DesignMatrix(string const& path, string const& sep = ",")
    : AbstractDesignMatrix<T>()
  {
    helios::filems::DesignMatrixReader<T> reader(path, sep);
    *this = reader.read();
  }
  virtual ~DesignMatrix() = default;

  // ***  OPERATORS  *** //
  // ******************* //
  /**
   * @brief Access to the \f$x_{ij}\f$ component of the \f$X\f$ design matrix
   * @param i The row of the element being accessed
   * @param j The column of the element being accessed
   * @return Reference to the element at \f$i\f$-th row and \f$j\f$-th
   *  column
   * @see fluxionum::DesignMatrix::X
   * @see AbstractDesignMatrix::operator()(size_t const, size_t const)
   */
  inline T& operator()(size_t const i, size_t const j) override
  {
    return X.at(i, j);
  }

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @brief Merge given DesignMatrix into this DesignMatrix
   *
   * After merging, the values of this DesignMatrix will be the union
   *  of the previous values and the new ones from the given DesignMatrix dm
   *
   * @param dm DesignMatrix to be merged into this
   */
  virtual void mergeInPlace(DesignMatrix const& dm)
  {
    X.insert_rows(X.n_rows, dm.getX());
  }
  /**
   * @see DesignMatrix::swapColumns(arma::uvec const &)
   */
  inline void swapColumns(vector<long unsigned int> const& indices)
  {
    return swapColumns(
      vector<unsigned long long>(indices.begin(), indices.end()));
  }
  /**
   * @see DesignMatrix::swapColumns(arma::uvec const &)
   */
  inline void swapColumns(vector<unsigned long long> const& indices)
  {
    return swapColumns(arma::uvec(indices));
  }
  /**
   * @brief Swap the columns of the DesignMatrix
   *
   * Let \f$\vec{u} = (u_1, \ldots, u_n)\f$ be the vector of indices for the
   *  \f$n\f$ columns of the matrix \f$X \in \mathbb{R}^{m \times n}\f$.
   *  Thus, the new matrix \f$X' \in \mathbb{R}^{m \times n}\f$ can be
   *  obtained from the \f$X\f$ matrix simply by assuming that the \f$i\f$-th
   *  column of the \f$X'\f$ matrix is the \f$u_i\f$-th column of the \f$X\f$
   *  matrix
   *
   * @param indices The vector of indices \f$\vec{u} = (u_1, \ldots, u_n)\f$
   */
  inline void swapColumns(arma::uvec const& indices) { X = X.cols(indices); }
  /**
   * @see DesignMatrix::dropColumns(arma::uvec const &)
   */
  inline void dropColumns(vector<long unsigned int> const& indices)
  {
    return dropColumns(
      vector<unsigned long long>(indices.begin(), indices.end()));
  }
  /**
   * @see DesignMatrix::dropColumns(arma::uvec const &)
   */
  inline void dropColumns(vector<unsigned long long> const& indices)
  {
    return dropColumns(arma::uvec(indices));
  }
  /**
   * @brief Remove the columns from the DesignMatrix
   *
   * Let \f$\vec{u} = (u_1, \ldots, u_k)\f$ be the vector of indices for the
   *  \f$k \leq n\f$ columns to be removed from the matrix
   *  \f$X \in \mathbb{R}^{m \times n}\f$.
   *  Thus, the new matrix \f$X' \in \mathbb{R}^{m \times (n-k)}\f$ can be
   *  obtained from the \f$X\f$ matrix simply by removing all the columns
   *  which index is the \f$u_i\f$ component of vector \f$\vec{u}\f$
   *
   * @param indices The vector of indices \f$vec{u} = (u_1, \ldots, u_n)\f$
   */
  inline void dropColumns(arma::uvec const& indices) { X.shed_cols(indices); }
  /**
   * @see DesignMatrix::dropRows(arma::uvec const &)
   */
  inline void dropRows(vector<long unsigned int> const& indices)
  {
    return dropRows(vector<unsigned long long>(indices.begin(), indices.end()));
  }
  /**
   * @see DesignMatrix::dropRows(arma::uvec const &)
   */
  inline void dropRows(vector<unsigned long long> const& indices)
  {
    return dropRows(arma::uvec(indices));
  }
  /**
   * @brief Remove the rows from the DesignMatrix
   *
   * Let \f$\vec{u} = (u_1, \ldots, u_k)\f$ be the vector of indices for the
   *  \f$k \leq m\f$ rows to be removed from the matrix
   *  \f$X \in \mathbb{R}^{m \times n}\f$.
   *  Thus, the new matrix \f$X' \in \mathbb{R}^{(m-k) \times n}\f$ can be
   *  obtained from the \f$X\f$ matrix simply by removing all the rows which
   *  index if the \f$u_i\f$ component of vector \f$\vec{u}\f$
   *
   * @param indices The vector of indices \f${u} = (u_1, \ldots, u_n)\f$
   */
  virtual void dropRows(arma::uvec const& indices) { X.shed_rows(indices); }
  /**
   * @brief Add \f$x\f$ to all elements of \f$j\f$-th column
   * @param colIdx The index (\f$j\f$) of the column to be modified
   * @param x The value to be added to each element of given column
   */
  inline void addToColumn(size_t const colIdx, T const x)
  {
    X.col(colIdx) += x;
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain a constant/read reference to the \f$X\f$ matrix
   * @return Constant/read reference to the \f$X\f$ matrix
   * @see fluxionum::DesignMatrix::X
   */
  inline arma::Mat<T> const& getX() const { return X; }
  /**
   * @brief Obtain a writable reference to the \f$X\f$ matrix
   *
   * <span style="color: red"><b>WARNING</b></span> use this method only if
   *  you know how to manipulate underlying data structures of armadillo
   *  library
   *
   * @return Writable reference to the \f$X\f$ matrix
   * @see fluxionum::DesignMatrix::X
   */
  inline arma::Mat<T>& getWritableX() { return X; }
  /**
   * @brief Obtain the number of rows of the DesignMatrix \f$X\f$
   * @return The number of rows of the DesignMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   * @see fluxionum::AbstractDesignMatrix::getNumRows
   */
  inline size_t getNumRows() const override { return X.n_rows; }
  /**
   * @brief Obtain the number of columns of the DesignMatrix \f$X\f$
   * @return The number of columns of the DesignMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   * @see fluxionum::AbstractDesignMatrix::getNumColumns
   */
  inline size_t getNumColumns() const override { return X.n_cols; }
  /**
   * @brief Obtain the number of elements of the DesignMatrix \f$X\f$
   * @return The number of elements of the DesignMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   * @see fluxionum::AbstractDesignMatrix::getNumElements
   */
  inline size_t getNumElements() const override { return X.n_elem; }
  /**
   * @brief Obtain the \f$i\f$-th row of the DesignMatrix \f$X\f$
   * @param i Index of the row to be obtained
   * @return The \f$i\f$-th row of the DesignMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   */
  inline arma::subview_row<T> const getRow(size_t const i) const
  {
    return X.row(i);
  }
  /**
   * @brief Like DesignMatrix::getRow(size_t const) but returning a copy by
   *  value instead of a view-like reference
   * @see fluxionum::DesignMatrix::getRow(size_t const)
   */
  inline arma::Row<T> getRowCopy(size_t const i) const { return getRow(i); }
  /**
   * @brief Obtain the \f$j\f$-th column of the DesignMatrix \f$X\f$
   * @param j Index of the column to be obtained
   * @return The \f$j\f$-th column of the DesignMatrix \f$X\f$X
   * @see fluxionum::DesignMatrix::X
   */
  inline arma::subview_col<T> const getColumn(size_t const j) const
  {
    return X.col(j);
  }
  /**
   * @brief Like DesignMatrix::getColumn(size_t const) but returning a copy
   *  by value instead of a view-like reference
   * @see fluxionum::DesignMatrix::getColumn(size_t const)
   */
  inline arma::Col<T> getColumnCopy(size_t const j) const
  {
    return getColumn(j);
  }
  /**
   * @brief Like the fluxionum::DesignMatrix::getColumn(size_t const) method
   *  but specifying the column by name
   * @param columnName The name of the column to be obtained
   * @see fluxionum::DesignMatrix::getColumn(size_t const)
   */
  inline arma::subview_col<T> const getColumn(string const& columnName) const
  {
    return getColumn(translateColumnNameToIndex(columnName));
  }
  /**
   * @brief Like DesignMatrix::getColumn(string const &) but returning a copy
   *  by value instead of a view-like reference
   * @see fluxionum::DesignMatrix::getColumn(string const &)
   */
  inline arma::Col<T> getColumnCopy(string const& columnName) const
  {
    return getColumn(columnName);
  }
  /**
   * @brief Set the \f$j\f$-th column of the DesignMatrix \f$X\f$
   * @param j Index of the column to be setted
   * @param col New values for the column to be setted
   * @see fluxionum::DesignMatrix::X
   */
  inline void setColumn(size_t const j, arma::Col<T> const& col)
  {
    X.col(j) = col;
  }
};

}
