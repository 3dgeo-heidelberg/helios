#pragma once

#include <fluxionum/DesignMatrix.h>
#include <fluxionum/DiffDesignMatrix.h>

#include <memory>

namespace fluxionum {

using std::string;
using std::shared_ptr;
using std::make_shared;

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
template <typename TimeType, typename VarType>
class TemporalDesignMatrix : public DesignMatrix<VarType> {
protected:
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
        arma::Mat<VarType> const &X, size_t const timeColumnIndex
    ){
        arma::Mat<VarType> Y(X);
        Y.shed_col()(tileColumnIndex);
        return Y;
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
        arma::Mat<VarType> const &X, size_t const timeColumnIndex
    ){
        return arma::Col<TimeType> t(X.col(timeColumnIndex));
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
    TemporalDesignMatrix(
        DesignMatrix<VarType> const &designMatrix,
        size_t const timeColumnIndex,
        string const timeName="time"
    ) :
        DesignMatrix(extractNonTimeMatrix(
            designMatrix.getX(), timeColumnIndex
        )),
        t(extractTimeVector(designMatrix.getX(), timeColumnIndex)),
        timeName(
            designMatrix.hasColumnNames() ?
                designMatrix.getColumnName(timeColumnIndex) :
                timeName
        )
    {}
    /**
     * @brief Build a TemporalDesignMatrix from given DesignMatrix and time
     *  vector
     * @param designMatrix The design matrix (with no time column)
     * @param timeVector The time vector
     * @param timeName The name for the time attribute
     */
    TemporalDesignMatrix(
        DesignMatrix<VarType> const &designMatrix,
        arma::Col<TimeType> const &timeVector,
        string const timeName="time"
    ) :
        DesignMatrix(designMatrix),
        t(timeVector),
        timeName(timeName)
    {}
    /**
     * @brief Build a TemporalDesignMatrix from given \f$X\f$ matrix and
     *  specified time column
     * @param X The matrix containing both data points and its associated time
     * @param timeColumnIndex Index of the column containing time values
     * @param timeName The name for the time attribute
     */
    TemporalDesignMatrix(
        arma::Mat<VarType> const &X,
        size_t const timeColumnIndex,
        string const timeName="time"
    ) :
        DesignMatrix<VarType>(extractNonTimeMatrix(
            X, timeColumnIndex
        )),
        t(extractTimeVector(X, timeColumnIndex)),
        timeName(timeName)
    {}
    /**
     * @brief Build a TemporalDesignMatrix from given DesignMatrix \f$X\f$
     *  and time vector \f$\vec{t}\f$
     * @param X The DesignMatrix (with no time column)
     * @param t The time vector
     * @param timeName The name for the time attribute
     */
    TemporalDesignMatrix(
        arma::Mat<VarType> const &X,
        arma::Col<TimeType> const &t,
        string const timeName="time"
    ) :
        DesignMatrix<VarType>(X),
        t(t),
        timeName(timeName)
    {}
    /**
     * @brief Build a TemporalDesignMatrix from data in file at given path and
     *  specified time column
     * @param path Path to the file containing both the data and the time
     *  vector
     * @param timeColumnIndex Index of the column containing time values
     * @param timeName The default name for the time attribute to be used in
     *  case the read DesignMatrix does not specify a column name for its time
     *  column
     */
    TemporalDesignMatrix(
        string const &path,
        size_t const timeColumnIndex,
        string const timeName="time"
    ) :
        TemporalDesignMatrix(
            DesignMatrix<VarType>(path),
            timeColumnIndex,
            timeName
        )
    {
        // TODO Rethink : Implement as read and swap
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
    inline TimeType& operator[] (size_t const i) {return t.at(i);}


    // ***  METHODS  *** //
    // ***************** //
    /**
     * @brief Build a DiffDesignMatrix from the TemporalDesignMatrix
     * @param diffType The type of differential to be used
     * @return DiffDesignMatrix built from TemporalDesignMatrix
     * @see fluxionum::TemporalDesignMatrix::toDiffDesignMatrixPointer
     * @see fluxionum::DiffDesignMatrix::DiffType
     * @see fluxionum::DiffDesignMatrix
     */
    DiffDesignMatrix<TimeType, VarType> toDiffDesignMatrix(
        DiffDesignMatrix<TimeType, VarType>::DiffType diffType =
            FORWARD_FINITE_DIFFERENCES
    ) const {return DiffDesignMatrix<TimeType, VarType>(*this, diffType);}
    /**
     * @brief Like fluxionum::TemporalDesignMatrix::toDiffDesignMatrix but
     *  returning a pointer to the object
     * @return Pointer to the DiffDesignMatrix built from TemporalDesignMatrix
     * @see fluxionum::TemporalDesignMatrix::toDiffDesignMatrix
     * @see fluxionum::DiffDesignMatrix
     */
    shared_ptr<DiffDesignMatrix> toDiffDesignMatrixPointer(
        DiffDesignMatrix<TimeType, VarType>::DiffType diffType =
            FORWARD_FINITE_DIFFERENCES
    ) const
    {return make_shared<DiffDesignMatrix<TimeType, VarType>>(*this, diffType);}


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain a constant/read reference to the time vector \f$\vec{t}\f$
     * @return Constant/read reference to the time vector \f$\vec{t}\f$
     * @see fluxionum::TemporalDesignMatrix::t
     */
    inline arma::Col<TimeType> const & getTimeVector() const {return t;}
};

}