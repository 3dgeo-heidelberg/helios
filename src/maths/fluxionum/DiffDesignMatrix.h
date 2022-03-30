#pragma once

#include <fluxionum/FluxionumException.h>
#include <fluxionum/AbstractDesignMatrix.h>
#include <fluxionum/TemporalDesignMatrix.h>
#include <fluxionum/FluxionumTypes.h>

#include <armadillo>

#include <vector>
#include <string>
#include <sstream>

namespace fluxionum {

using std::vector;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief The heart of a differential design matrix is the idea that the
 *  columns are defining the values of variables differentiated over
 *  time. The \f$m-1\f$ rows are samples of the differential behavior of the
 *  function \f$x_j(t_i)\f$ for the \f$j\f$-th column as variable.
 *  For then, it is possible to interpolate the following vector of derivatives
 *  \f[
 *      \overrightarrow{\frac{dx}{dt}} =
 *      \left[\begin{array}{c}
 *          \frac{dx_1}{dt} \\
 *          \vdots \\
 *          \frac{dx_n}{dt}
 *      \end{array}\right]
 *  \f]
 *  using each \f$i\f$-th row to fit each \f$j\f$-th variable
 *
 * To explain in detail the fundamentals of a DiffDesignMatrix, assume a
 *  restricted time domain such that \f$t \in [t_a, t_b]\f$. The values
 *  defining the extremes of the interval correspond to the minimum and the
 *  maximum time values in the TemporalDesignMatrix that was used to generate
 *  the DiffDesignMatrix. There is also a time vector available, such that
 *  for the \f$m\f$ points (matrix rows) there are also \f$m\f$ associated time
 *  values \f$\vec{t} = \left(t_1, \ldots, t_m\right)\f$. Thus, for each
 *  \f$t_i\f$ there exists a vector \f$\vec{x_i'} \in \mathbb{R}^{n}\f$ which
 *  defines the values that the attributes took at time \f$t_i\f$. From now on,
 *  it will be assumed that \f$\forall i, t_{i+1} > t_{i}\f$. It is, the
 *  indices of time values and their associated vectors are sorted in time with
 *  no repetitions.
 *
 * Now, lets define the \f$\frac{DX}{DT} \in \mathbb{R}^{(m-1) \times n}\f$
 *  matrix as a forward finite differences based matrix:
 * \f[
 *  \frac{DX}{DT} = \frac{\Delta X}{\Delta T} = \left[\begin{array}{ccc}
 *      \frac{x_{21}' - x_{11}'}{t_2 - t_1} &
 *          \ldots &
 *          \frac{x_{2n}' - x_{1n}'}{t_2 - t_1} \\
 *      \vdots & \ddots & \vdots \\
 *      \frac{x_{m1}' - x_{m-1,1}'}{t_m - t_{m-1}} &
 *          \ldots &
 *          \frac{x_{mn}' - x_{m-1,n}'}{t_m - t_{m-1}}
 *  \end{array}\right]
 * \f]
 * The former matrix can also be defined as a central finite differences based
 *  matrix, as will be later explained. However, no matter how the matrix is
 *  defined, it will be noted as matrix \f$A \in \mathbb{R}^{l \times n}\f$
 *  from now on, for the sake of simplicity. This matrix is expected to define
 *  accurate samples of the derivative for each of the variables.
 * \f[
 *  A = \left[\begin{array}{ccc}
 *      a_{11} & \ldots & a_{1n} \\
 *      \vdots & \ddots & \vdots \\
 *      a_{m1} & \ldots & a_{mn}
 *  \end{array}\right]
 * \f]
 *
 * An <b>example of fitting</b> is developed based on the Moore-Penrose
 *  pseudoinverse of a matrix.
 *  For this purpose, assume \f$l\f$ time samples of \f$n\f$
 *  variables each, for which it is possible to build an approximation by means
 *  of a polynomial series. Then, it must be that there are \f$n\f$ functions,
 *  such that for any \f$j\f$-th variable there is a function
 *  \f$f_j(t_i) = \sum_{k=0}^{k_j} \omega_{jk}t_{i}^{k}\approx a_{ij}\f$ that
 *  approximates \f$\frac{dx_j}{dt}\f$.
 *  Since there are \f$n\f$ polynomial series of order \f$k_j\f$ each, it is
 *  known that there are \f$n\f$ coefficient vectors
 *  \f$\vec{\omega_{j}} \in \mathbb{R}^{k_j + 1}\f$. Also, there is an implicit
 *  matrix \f$T_j \in \mathbb{R}^{l \times (k_j+1)}\f$ for each variable
 *  as follows:
 * \f[
 *  T_j = \left[\begin{array}{ccc}
 *      t_1^0 & \dots & t_1^k \\
 *      \vdots & \ddots & \vdots \\
 *      t_l^0 & \dots & t_l^k
 *  \end{array}\right]
 * \f]
 *
 * Let \f$\vec{a_j} = \left(a_{1j}, \ldots, a_{lj}\right)\f$ be the \f$j\f$-th
 *  column vector of matrix \f$A_{l \times n}\f$ and thus the system
 *  \f$T_j \vec{\omega_j} = \vec{a_j}\f$ arises. The polynomial coefficients
 *  can then be found simply by solving them with Moore-Penrose pseudoinverse
 *  such that \f$\vec{\omega_j} =
 *      (T_j^T T_j)^{-1}T_j^T \vec{a_j}  =
 *      T_j^\dagger \vec{a_j}\f$. This way the derivatives for each of the
 *  \f$n\f$ variables are approximated each by its own function:
 * \f[
 *  \forall j, f_j(t) = \sum_{k=0}^{k_j}{\omega_{jk}t^k} \approx
 *      \frac{d}{dt}x_j(t)
 * \f]
 *
 *
 * It was stated before that the \f$A \in \mathbb{R}^{l \times n}\f$ matrix
 *  was generated from forward finite differences so \f$l=m-1\f$ and thus
 *  \f$A = \frac{DX}{DT} =
 *      \frac{\Delta X}{\Delta T} \in \mathbb{R}^{m-1 \times n}\f$.
 *  However, it is also possible to define \f$A = \frac{DX}{DT} =
 *      \frac{\delta X}{\delta T} \in \mathbb{R}^{m-2 \times n}\f$ so it comes
 *  from <b>central finite differences</b>. Central finite differences might
 *  lead to a more accurate approximation of the derivatives, at the expenses
 *  of losing \f$2\f$ data points instead of \f$1\f$. The first step to compute
 *  central finite differences is to define a new time vector such that
 *  \f$\vec{\tau} =
 *      \left(\frac{t_1+t_2}{2}, \ldots, \frac{t_{m-1}+t_m}{2}\right) =
 *      \left(\tau_1, \ldots, \tau_{m-1}\right)
 *      \in \mathbb{R}^{m-1}\f$. Thus, for central finite differences the
 *  \f$T \in \mathbb{R}^{(m-2) \times (k_{j}+1)}\f$ matrix has one less row and
 *  its values come from the first \f$m-2\f$ values of vector
 *  \f$\vec{\tau} \in \mathbb{R}^{m-1}\f$ instead of the first \f$m-1\f$ values
 *  of the time vector \f$\vec{t} \in \mathbb{R}^{m}\f$. The second step is to
 *  compute the matrix \f$\frac{\delta X}{\delta T}\f$ as:
 * \f[
 *  \frac{DX}{DT} = \frac{\delta X}{\delta T} = \left[\begin{array}{ccc}
 *      \frac{x_{31}' - x_{11}'}{2(\tau_2 - \tau_1)} &
 *          \ldots &
 *          \frac{x_{3n}' - x_{1n}'}{2(\tau_2 - \tau_1)} \\
 *      \vdots & \ddots & \vdots \\
 *      \frac{x_{m1}' - x_{m-2,1}'}{2(\tau_{m-1} - \tau_{m-2})} &
 *          \ldots &
 *          \frac{x_{mn}' - x_{m-2,n}'}{2(\tau_{m-1} - \tau_{m-2})}
 *  \end{array}\right] = \left[\begin{array}{ccc}
 *      \frac{x_{31}' - x_{11}'}{t_3 - t_1} &
 *          \ldots &
 *          \frac{x_{3n}' - x_{1n}'}{t_3 - t_1} \\
 *      \vdots & \ddots & \vdots \\
 *      \frac{x_{m1}' - x_{m-2,1}'}{t_m - t_{m-2}} &
 *          \ldots &
 *          \frac{x_{mn}' - x_{m-2,n}'}{t_m - t_{m-2}}
 *  \end{array}\right]
 * \f]
 *
 *
 * The fitting example for central finite differences would be exactly as
 *  described before, but now the approximation would be based on
 *  \f$\vec{\tau}\f$ and \f$A=\frac{\delta X}{\delta T}\f$ instead of
 *  \f$\vec{t}\f$ and \f$A=\frac{\Delta X}{\Delta T}\f$.
 *  It is, the function \f$f_j(t)\f$ is fitted from
 *  \f$f_j(\tau_{i}) =
 *      \sum_{k=0}^{k_j}{\omega_{jk}\tau_i^k} \approx
 *      a_{ij}\f$.
 *
 * @tparam TimeType The time's domain
 * @tparam VarType The non time's domain (basic DesignMatrix domain)
 * @see fluxionum::AbstractDesignMatrix
 * @see fluxionum::TemporalDesignMatrix
 * @see fluxionum::DiffDesignMatrixType
 */
template <typename TimeType, typename VarType>
class DiffDesignMatrix : AbstractDesignMatrix<VarType>{
public:
    // ***  USING  *** //
    // *************** //
    using AbstractDesignMatrix<VarType>::getColumnNames;

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Specify the type of differential of the DiffDesignMatrix
     * @see fluxionum::DiffDesignMatrixType
     */
    enum DiffDesignMatrixType diffType;
    /**
     * @brief The starting value for differential time interval, \f$t_a\f$
     */
    TimeType ta;
    /**
     * @brief The end value for the differential time interval, \f$t_b\f$
     */
    TimeType tb;
    /**
     * @brief The sorted series of time values so
     *  \f$\forall i, t_{i+1} > t_{i}\f$ is strictly satisfied and the
     *  \f$i\f$-th time value corresponds to the approximated
     *  \f$\overrightarrow{\frac{dx_i}{dt}}\f$ row vector in the
     *  \f$\frac{DX}{DT} \in \mathbb{R}^{(m-1) \times n}\f$ matrix. It is used
     *  to build the \f$T\f$ matrix.
     *
     * Notice the \f$\vec{t}\f$ vector is in fact the \f$\vec{\tau}\f$ vector
     *  when the DiffDesignMatrix is built from central finite differences
     *
     * <b><span color="red">WARNING</span>!</b>: The
     *  DiffDesignMatrix::t vector must not be confused with
     *  TemporalDesignMatrix::t vector. For the case of forward finite
     *  differences the DiffDesignMatrix::t vector is
     *  \f$\vec{t} = \left(t_1, \ldots, t_{m-1}\right) \in \mathbb{R}^{m-1}\f$.
     *  It is, all values from DiffDesignMatrix::t but the last one. In the
     *  case of central finite differences the DiffDesignMatrix::t vector is
     *  \f$\vec{t} =
     *      \left(\frac{t_1+t_2}{2}, \ldots, \frac{t_{m-2}-t_{m-1}}{2}\right)
     *      \in \mathbb{R}^{m-2}\f$.
     *  It is, all values from the \f$\vec{\tau}\f$ vector but the last one.
     *
     */
    arma::Col<TimeType> t;
    /**
     * @brief The \f$\frac{DX}{DT}\f$ matrix, whether it comes from forward
     *  finite differences or from central finite differences.
     */
    arma::Mat<VarType> A;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a DiffDesignMatrix with no data
     * @param columnNames Either the name for each column or an empty vector if
     *  there are no names
     */
    DiffDesignMatrix(
        vector<string> const &columnNames=vector<string>(0),
        DiffDesignMatrixType diffType=DiffDesignMatrixType::UNKNOWN
    ) :
        AbstractDesignMatrix<VarType>(columnNames),
        diffType(diffType)
    {}
    /**
     * @brief Build a DiffDesignMatrix from given armadillo column vector of
     *  time values and given armadillo matrix of attributes
     * @param t Armadillo column vector of time values to build
     *  DiffDesignMatrix from
     * @param A Armadillo matrix of attributes to build DiffDesignMatrix from
     * @param columnNames Either the name for each column or an empty vector if
     *  there are no names
     */
    DiffDesignMatrix(
        arma::Col<TimeType> const &t,
        arma::Mat<VarType> const &A,
        vector<string> const &columnNames=vector<string>(0),
        DiffDesignMatrixType diffType=DiffDesignMatrixType::UNKNOWN
    ) :
        AbstractDesignMatrix<VarType>(columnNames),
        diffType(diffType),
        ta(arma::min(t)),
        tb(arma::max(t)),
        t(t),
        A(A)
    {}
    /**
     * @brief Build a DiffDesignMatrix from data in file at given path
     * @param path Path to the file containing the data for the
     *  DiffDesignMatrix
     * @param columnNames The default column names to be used if no column
     *  names are read from file at given path
     */
    DiffDesignMatrix(
        string const &path,
        vector<string> const &columnNames=vector<string>(0),
        DiffDesignMatrixType const diffType=DiffDesignMatrixType::UNKNOWN
    ) :
        AbstractDesignMatrix<VarType>(columnNames),
        diffType(diffType)
    {}
    /**
     * @brief Build a DiffDesignMatrix from given TemporalDesignMatrix
     * @param tdm The TemporalDesignMatrix to be used to build the
     *  DiffDesignMatrix
     * @param diffType The type of differencing method
     */
    DiffDesignMatrix(
        TemporalDesignMatrix<TimeType, VarType> const &tdm,
        DiffDesignMatrixType const diffType
    ) :
        AbstractDesignMatrix<VarType>(tdm.getColumnNames())
    {
        switch (diffType) {
            case DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES:{
                arma::Col<TimeType> const &t = tdm.getTimeVector();
                arma::Col<TimeType> DT(t.subvec(0, t.n_elem-1));
                arma::Mat<VarType> DXDT = arma::diff(tdm.getX(), 1, 0);
                DXDT.each_col() /= arma::diff(t, 1);
                *this = DiffDesignMatrix(DT, DXDT, getColumnNames(), diffType);
                break;
            }
            case DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES:{
                arma::Mat<VarType> const &X = tdm.getX();
                arma::Col<TimeType> const &t = tdm.getTimeVector();
                arma::Col<TimeType> DT(
                    (t.subvec(0, t.n_elem-1) + t.subvec(1, t.n_elem))
                );
                arma::Mat<VarType> DXDT(
                    (X.rows(2, X.n_rows) - X.rows(0, X.n_rows-2))
                );
                DXDT.each_col() /= arma::diff(DT, 1);
                DT = DT.subvec(0, DT.n_elem-1) / 2.0;
                *this = DiffDesignMatrix(DT, DXDT, getColumnNames(), diffType);
                break;
            }
            default:{
                std::stringstream ss;
                ss  << "DiffDesignMatrix::DiffDesignMatrix("
                       "TemporalDesignMatrix const &, DiffType const) failed."
                       "\n\tUnexpected differential type";
                throw FluxionumException(ss.str());
            }
        }
    }
};

}