#ifndef _FLUXIONUM_TEMPORAL_DESIGN_MATRIX_H_

#include <fluxionum/DesignMatrix.h>
#include <fluxionum/FluxionumTypes.h>

#include <memory>
#include <unordered_map>

namespace fluxionum {

template<typename TimeType, typename VarType>
class DiffDesignMatrix;

using std::make_shared;
using std::shared_ptr;
using std::string;
using std::unordered_map;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 *
 * @brief This class represents a DesignMatrix where each row satisfy that
 *  its elements can be placed into a continuous succession order
 *  representation (i.e. continuous time domain). For a discrete succession
 *  order representation see fluxionum::IndexedDesignMatrix better.
 *
 *
 * Let \f$x_i\f$ be the \f$i\f$-th row of the temporal design matrix such that
 * \f[
 *  \exists j \in \mathbb{Z} \ni \forall i,\, x_i =  \left\{
 *      t_{i} = x_{ij},
 *      x_{i}' = \bigg{(}
 *          x_{i,1}, \ldots, x_{i,j-1} , x_{i,j+1}, \ldots, x_{i,n}
 *       \bigg{)}
 *  \right\}
 * \f]
 *
 * For which its valid status constraint can be defined as:
 * \f[
 *  \forall i, t_{i+1} > t_{i}
 * \f]
 * Notice that implicitly implies the following is <b>not</b> allowed:
 * \f[
 *  \exists i \ni t_{i+1} = t_{i}
 * \f]
 * It is, no repeated time values are allowed because they make the
 *  TemporalDesignMatrix lose its valid status constraint, thus any calculus
 *  involving it is undefined.
 *
 * Thus, it is expected that a
 *  \f$\frac{DX}{DT} \in \mathbb{R}^{(m-1) \times n}\f$
 *  DiffDesignMatrix exists for the concrete state of any valid
 *  TemporalDesignMatrix such that:
 * \f[
 *  \frac{DX}{DT} = \left[\begin{array}{ccc}
 *      \frac{x_{21}' - x_{11}'}{t_2 - t_1} &
 *          \ldots &
 *          \frac{x_{2n}' - x_{1n}'}{t_2 - t_1} \\
 *      \vdots & \ddots & \vdots \\
 *      \frac{x_{m1}' - x_{m-1,1}'}{t_m - t_{m-1}} &
 *          \ldots &
 *          \frac{x_{mn}' - x_{m-1,n}'}{t_m - t_{m-1}}
 *  \end{array}\right]
 * \f]
 *
 * @tparam TimeType The time's domain
 * @tparam VarType The non time's domain (basic DesignMatrix domain)
 * @see fluxionum::IndexedDesignMatrix
 * @see fluxionum::DiffDesignMatrix
 * @see fluxionum::DesignMatrix
 */
template<typename TimeType, typename VarType>
class TemporalDesignMatrix : public DesignMatrix<VarType>
{
protected:
  // ***  USING  *** //
  // *************** //
  using DesignMatrix<VarType>::X;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The time values such that \f$t_{i}\f$ is associated with the
   *  \f$i\f$-th row of the design matrix \f$X\f$. It must strictly satisfy
   *  \f$\forall i, t_{i+1} > t_i\f$
   */
  arma::Col<TimeType> t;
  /**
   * @brief The name of the time field in the original DesignMatrix.
   *
   * By default, it is "time"
   */
  string timeName;

public:
  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Do a copy of the DesignMatrix \f$X\f$ with all its columns but
   *  removing the time column
   * @param X  The DesignMatrix \f$X\f$ to be copied
   * @param timeColumnIndex The index of the time column in given
   *  DesignMatrix \f$X\f$X
   * @return Copy of given DesignMatrix \f$X\f$ with no time column
   * @see fluxionum::DesignMatrix::X
   */
  static inline arma::Mat<VarType> extractNonTimeMatrix(
    arma::Mat<VarType> const& X,
    size_t const timeColumnIndex)
  {
    arma::Mat<VarType> Y(X);
    Y.shed_col(timeColumnIndex);
    return Y;
  }
  /**
   * @brief Do a copy of the names of the DesignMatrix but discarding the
   *  name of the time column
   * @param names The names of the DesignMatrix to be copied
   * @param timeColumnIndex The index of the time column
   * @return Copy of the names of given DesignMatrix but discarding the name
   *  of the time column
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  static inline vector<string> extractNonTimeNames(vector<string> const& names,
                                                   size_t const timeColumnIndex)
  {
    vector<string> newNames;
    size_t numNames = names.size();
    for (size_t i = 0; i < numNames; ++i) {
      if (i == timeColumnIndex)
        continue;
      newNames.push_back(names[i]);
    }
    return newNames;
  }
  /**
   * @brief Do a copy of the time column from given DesignMatrix \f$X\f$
   * @param X The DesignMatrix \f$X\f$ containing a time column
   * @param timeColumnIndex The index of the time column in given
   *  DesginMatrix \f$X\f$
   * @return The copy of the time column from DesginMatrix \f$X\f$
   * @see fluxionum::DesignMatrix::X
   */
  static inline arma::Col<TimeType> extractTimeVector(
    arma::Mat<VarType> const& X,
    size_t const timeColumnIndex)
  {
    return arma::Col<TimeType>(X.col(timeColumnIndex));
  }

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a TemporalDesignMatrix from given DesignMatrix and
   *  specified time column
   * @param designMatrix The full design matrix, including time values
   * @param timeColumnIndex Index of the column containing time values
   * @param timeName The default name for the time attribute to be used in
   *  case the given DesignMatrix does not specify a column name for its time
   *  column
   */
  TemporalDesignMatrix(DesignMatrix<VarType> const& designMatrix,
                       size_t const timeColumnIndex,
                       string const timeName = "time",
                       vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(
        extractNonTimeMatrix(designMatrix.getX(), timeColumnIndex),
        designMatrix.hasColumnNames()
          ? extractNonTimeNames(designMatrix.getColumnNames(), timeColumnIndex)
          : columnNames)
    , t(extractTimeVector(designMatrix.getX(), timeColumnIndex))
    , timeName(designMatrix.hasColumnNames()
                 ? designMatrix.getColumnName(timeColumnIndex)
                 : timeName)
  {
  }
  /**
   * @brief Build a TemporalDesignMatrix from given DesignMatrix and time
   *  vector
   * @param designMatrix The design matrix (with no time column)
   * @param timeVector The time vector
   * @param timeName The name for the time attribute
   */
  TemporalDesignMatrix(DesignMatrix<VarType> const& designMatrix,
                       arma::Col<TimeType> const& timeVector,
                       string const timeName = "time")
    : DesignMatrix<VarType>(designMatrix)
    , t(timeVector)
    , timeName(timeName)
  {
  }
  /**
   * @brief Build a TemporalDesignMatrix from given \f$X\f$ matrix and
   *  specified time column
   * @param X The matrix containing both data points and its associated time
   * @param timeColumnIndex Index of the column containing time values
   * @param timeName The name for the time attribute
   */
  TemporalDesignMatrix(arma::Mat<VarType> const& X,
                       size_t const timeColumnIndex,
                       string const timeName = "time",
                       vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(extractNonTimeMatrix(X, timeColumnIndex),
                            columnNames)
    , t(extractTimeVector(X, timeColumnIndex))
    , timeName(timeName)
  {
  }
  /**
   * @brief Build a TemporalDesignMatrix from given DesignMatrix \f$X\f$
   *  and time vector \f$\vec{t}\f$
   * @param X The DesignMatrix (with no time column)
   * @param t The time vector
   * @param timeName The name for the time attribute
   */
  TemporalDesignMatrix(arma::Mat<VarType> const& X,
                       arma::Col<TimeType> const& t,
                       string const timeName = "time",
                       vector<string> const& columnNames = vector<string>(0))
    : DesignMatrix<VarType>(X, columnNames)
    , t(t)
    , timeName(timeName)
  {
  }
  /**
   * @brief Build a TemporalDesignMatrix from data in file at given path and
   *  specified time column
   * @param path Path to the file containing both the data and the time
   *  vector
   * @param timeName The default name for the time attribute to be used in
   *  case the read DesignMatrix does not specify a column name for its time
   *  column
   */
  TemporalDesignMatrix(string const& path,
                       string const& sep = ",",
                       string const& timeName = "time")
  {
    helios::filems::DesignMatrixReader<VarType> reader(path, sep);
    std::unordered_map<string, string> kv;
    DesignMatrix<VarType> const dm = reader.read(&kv);
    size_t const tCol =
      (kv.find("TIME_COLUMN") == kv.end())
        ? dm.translateColumnNameToIndex("t")
        : (size_t)std::strtoul(kv.at("TIME_COLUMN").c_str(), nullptr, 10);
    *this = TemporalDesignMatrix<TimeType, VarType>(
      dm, tCol, dm.hasColumnNames() ? dm.getColumnName(tCol) : timeName);
  }
  virtual ~TemporalDesignMatrix() = default;

  // ***  OPERATORS  *** //
  // ******************* //
  /**
   * @brief Access to the \f$t_{i}\f$ component of the \f$\vec{t}\f$ time
   *  vector
   * @param i The index of the time being accessed (its position in the time
   *  vector)
   * @return Reference to the \f$t_{i}\f$ component of the time vector
   *  \f$\vec{t}\f$
   */
  inline TimeType& operator[](size_t const i) { return t.at(i); }

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @brief Build a DiffDesignMatrix from the TemporalDesignMatrix
   * @param diffType The type of differential to be used
   * @param sort When true, the DiffDesignMatrix construction will lead to an
   *  expensive sort operation to assure the finite differences
   *  computation makes sense.
   *  When false, the expensive sort computation will be skipped as it is
   *  expected that the TemporalDesignMatrix is already sorted
   * @return DiffDesignMatrix built from TemporalDesignMatrix
   * @see fluxionum::TemporalDesignMatrix::toDiffDesignMatrixPointer
   * @see fluxionum::DiffDesignMatrixType
   * @see fluxionum::DiffDesignMatrix
   */
  DiffDesignMatrix<TimeType, VarType> toDiffDesignMatrix(
    DiffDesignMatrixType diffType =
      DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES,
    bool const sort = true) const;
  /**
   * @brief Like fluxionum::TemporalDesignMatrix::toDiffDesignMatrix but
   *  returning a pointer to the object
   * @return Pointer to the DiffDesignMatrix built from TemporalDesignMatrix
   * @see fluxionum::TemporalDesignMatrix::toDiffDesignMatrix
   * @see fluxionum::DiffDesignMatrix
   */
  shared_ptr<DiffDesignMatrix<TimeType, VarType>> toDiffDesignMatrixPointer(
    DiffDesignMatrixType diffType =
      DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES,
    bool const sort = true) const;

  /**
   * @brief Extend DesignMatrix::mergeInPlace method so the time values are
   *  also merged
   * @param dm The TemporalDesignMatrix to be merged
   */
  void mergeInPlace(DesignMatrix<VarType> const& dm) override
  {
    DesignMatrix<VarType>::mergeInPlace(dm);
    TemporalDesignMatrix const& tdm =
      static_cast<TemporalDesignMatrix<TimeType, VarType> const&>(dm);
    t.insert_rows(t.n_rows, tdm.getTimeVector());
  }
  /**
   * @brief Extend DesignMatrix::dropRows(arma::uvec const &) method so the
   *  time values are also dropped
   * @param indices Indices of the rows to be dropped
   */
  inline void dropRows(arma::uvec const& indices) override
  {
    DesignMatrix<VarType>::dropRows(indices);
    t.shed_rows(indices);
  }

  /**
   * @brief Add given time \f$x\f$ to all time values in vector \f$\vec{t}\f$
   *
   * \vec{t'} = \vec{t} + x = \left[\begin{array}{c}
   *  t_1 + x \\
   *  \vdots
   *  t_m + x
   * \end{array}\right.
   *
   * @param x The time shift for each time component
   */
  virtual void shiftTime(TimeType const x) { t += x; }
  /**
   * @brief Sort all rows of \f$X\f$ matrix and all components of
   *  \f$\vec{t}\f$ vector so \f$t_{i+1} > t_{i}\f$ \f$vec{t}\f$, considering
   *  the row \f$X(i,:)\f$ is associated with the time component \f$t_i\f$
   */
  virtual void sortByTime()
  {
    arma::uvec const tSort = arma::sort_index(getTimeVector());
    t = t(tSort);
    X = X.rows(tSort);
  }

  /**
   * @brief Apply a filter based on the first derivative understood as the
   *  slope.
   *
   * Let \f$X \in \mathbb{R}^{m \times n}\f$ such that
   * \f$x_j(t_i) = X(i,j) = x_{ij}\f$, so it is possible to approximate the
   * derivative as follows:
   * \f[
   *  \frac{\Delta x_j}{\Delta t}(t_i) =
   *  \frac{x_j(t_{i+1}) - x_j(t_i)}{t_{i+1} - t_i}
   * \f]
   *
   * But then, the differences between consecutive derivatives can be
   *  calculated as shown below:
   * \f[
   *  Dx_j(t_i) = \frac{\Delta x_j}{\Delta t}(t_{i+1}) -
   *      \frac{\Delta x_j}{\Delta t}(t_i)
   * \f]
   *
   * Therefore, the vector
   * \f$DX(t_i) = \left(Dx_1(t_i), \ldots, Dx_n(t_i)\right)\f$ arises.
   *  And the maximum slope difference at time \f$t_i\f$ can be defined as
   *  the maximum absolute component from the vector \f$DX(t_i)\f$ such that
   *  \f$d_i^* = \max_{1 \leq j \leq n} \vert{Dx_j(t_i)}\vert\f$.
   *
   * Finally, let \f$\tau\f$ be the slope difference threshold. Thus, always
   *  that \f$d_i^* > \tau\f$ the \f$i+1\f$ row/point will be removed from
   *  matrix \f$X\f$ as it is considered that it does not bring useful enough
   *  information.
   *
   * @param tau The threshold \f$\tau\f$ to decide whether to preserver or
   *  to discard a row/point
   * @return The number of discarded rows/points
   */
  virtual size_t slopeFilter(VarType const tau)
  {
    // Obtain DX
    arma::Mat<VarType> DX = arma::diff(this->getX());
    DX.each_col() /= arma::diff(getTimeVector());
    DX = arma::diff(DX);
    // Filter by DX
    size_t const m = DX.n_rows;
    arma::Row<VarType> S(DX.n_cols, arma::fill::zeros);
    vector<unsigned long long> remove;
    for (size_t i = 0; i < m; ++i) {
      S += DX.row(i);
      VarType const maxDiff = arma::max(arma::abs(S));
      if (maxDiff > tau)
        S = arma::zeros<arma::Row<VarType>>(DX.n_cols);
      else
        remove.push_back(i + 1);
    }
    dropRows(remove);
    return remove.size();
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain a constant/read reference to the time vector \f$\vec{t}\f$
   * @return Constant/read reference to the time vector \f$\vec{t}\f$
   * @see fluxionum::TemporalDesignMatrix::t
   */
  inline arma::Col<TimeType> const& getTimeVector() const { return t; }
  /**
   * @brief Obtain the name of the time attribute
   * @return The name of the time attribute
   * @see fluxionum::TemporalDesignMatrix::timeName
   */
  inline string const& getTimeName() const { return timeName; }
  /**
   * @brief Set the name of the time attribute
   * @param timeName The new name for the time attribute
   * @see fluxionum::TemporalDesignMatrix::timeName
   */
  inline void setTimeName(string const& timeName) { this->timeName = timeName; }
};

}

#define _FLUXIONUM_TEMPORAL_DESIGN_MATRIX_H_
#include <fluxionum/TemporalDesignMatrix.tpp>
#endif
