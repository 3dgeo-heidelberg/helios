#pragma once

#include <fluxionum/AbstractDesignMatrix.h>
#include <filems/read/core/DesignMatrixReader.h>

#include <armadillo>

#include <vector>
#include <string>

namespace fluxionum {

using namespace helios::filems;

using std::vector;
using std::string;

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
template <typename T>
class DesignMatrix : public AbstractDesignMatrix<T>{
protected:
    // ***  USING  *** //
    // *************** //
    using AbstractDesignMatrix<T>::translateColumnNameToIndex;

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
    DesignMatrix(
        vector<string> const &columnNames=vector<string>(0)
    ) :
        AbstractDesignMatrix<T>(columnNames)
    {}
    /**
     * @brief Build a DesignMatrix from given armadillo matrix
     * @param X Armadillo matrix to build DesignMatrix from
     * @param columnNames Either the name for each column or an empty vector
     *  if there are no names
     */
    DesignMatrix(
        arma::Mat<T> const &X,
        vector<string> const &columnNames=vector<string>(0)
    ) :
        AbstractDesignMatrix<T>(columnNames),
        X(X)
    {}
    /**
     * @brief Build a DesignMatrix from data in file at given path
     * @param path Path to the file containing the data for the DesignMatrix
     * @param columnNames The default column names to be used if no column
     *  names are read from file at given path
     */
    DesignMatrix(string const &path) :
        AbstractDesignMatrix<T>()
    {
        helios::filems::DesignMatrixReader<T> reader(path);
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
    inline T& operator() (size_t const i, size_t const j) override
    {return X.at(i, j);}


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain a constant/read reference to the \f$X\f$ matrix
     * @return Constant/read reference to the \f$X\f$ matrix
     * @see fluxionum::DesignMatrix::X
     */
    inline arma::Mat<T> const & getX() const {return X;}
    /**
     * @brief Obtain the number of rows of the DesignMatrix \f$X\f$
     * @return The number of rows of the DesignMatrix \f$X\f$
     * @see fluxionum::DesignMatrix::X
     * @see fluxionum::AbstractDesignMatrix::getNumRows
     */
    inline size_t getNumRows() const override {return X.n_rows;}
    /**
     * @brief Obtain the number of columns of the DesignMatrix \f$X\f$
     * @return The number of columns of the DesignMatrix \f$X\f$
     * @see fluxionum::DesignMatrix::X
     * @see fluxionum::AbstractDesignMatrix::getNumColumns
     */
    inline size_t getNumColumns() const override {return X.n_cols;}
    /**
     * @brief Obtain the number of elements of the DesignMatrix \f$X\f$
     * @return The number of elements of the DesignMatrix \f$X\f$
     * @see fluxionum::DesignMatrix::X
     * @see fluxionum::AbstractDesignMatrix::getNumElements
     */
    inline size_t getNumElements() const override {return X.n_elem;}
    /**
     * @brief Obtain the \f$i\f$-th row of the DesignMatrix \f$X\f$
     * @param i Index of the row to be obtained
     * @return The \f$i\f$-th row of the DesignMatrix \f$X\f$
     * @see fluxionum::DesignMatrix::X
     */
    inline arma::subview_row<T> const getRow(size_t const i) const
    {return X.row(i);}
    /**
     * @brief Like DesignMatrix::getRow(size_t const) but returning a copy by
     *  value instead of a view-like reference
     * @see fluxionum::DesignMatrix::getRow(size_t const)
     */
    inline arma::Row<T> getRowCopy(size_t const i) const {return getRow(i);}
    /**
     * @brief Obtain the \f$j\f$-th column of the DesignMatrix \f$X\f$
     * @param j Index of the column to be obtained
     * @return The \f$j\f$-th column of the DesignMatrix \f$X\f$X
     * @see fluxionum::DesignMatrix::X
     */
    inline arma::subview_col<T> const getColumn(size_t const j) const
    {return X.col(j);}
    /**
     * @brief Like DesignMatrix::getColumn(size_t const) but returning a copy
     *  by value instead of a view-like reference
     * @see fluxionum::DesignMatrix::getColumn(size_t const)
     */
    inline arma::Col<T> getColumnCopy(size_t const j) const
    {return getColumn(j);}
    /**
     * @brief Like the fluxionum::DesignMatrix::getColumn(size_t const) method
     *  but specifying the column by name
     * @param columnName The name of the column to be obtained
     * @see fluxionum::DesignMatrix::getColumn(size_t const)
     */
    inline arma::subview_col<T> const  getColumn(
        string const &columnName
    ) const
    {return getColumn(translateColumnNameToIndex(columnName));}
    /**
     * @brief Like DesignMatrix::getColumn(string const &) but returning a copy
     *  by value instead of a view-like reference
     * @see fluxionum::DesignMatrix::getColumn(string const &)
     */
    inline arma::Col<T> getColumnCopy(string const &columnName) const
    {return getColumn(columnName);}

};

}