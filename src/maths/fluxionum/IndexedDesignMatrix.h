#pragma once

#include <fluxionum/DesignMatrix.h>
#include <fluxionum/TemporalDesignMatrix.h>

#include <vector>
#include <memory>

namespace fluxionum{

using std::vector;
using std::shared_ptr;
using std::make_shared;

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
template <typename IndexType, typename VarType>
class IndexedDesignMatrix : public DesignMatrix<T> {
protected:
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
        arma::Mat<VarType> const &X, size_t const indicesColumnIndex
    ){
        arma::Col<VarType> inds = X.col(indicesColumnIndex);
        size_t const m = inds.n_rows;
        vector<IndexType> indices(inds.n_rows);
        for(size_t i = 0 ; i < m ; ++i) indices[i] = (IndexType) inds[i];
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
    IndexedDesignMatrix(
        DesignMatrix<VarType> const &designMatrix,
        size_t const indicesColumnIndex,
        string const indexName="index"
    ) :
        DesignMatrix<VarType>(
            TemporalDesignMatrix<VarType>::extractNonTimeMatrix(
                designMatrix.getX(), indicesColumnIndex
            )
        ),
        indices(extractIndices(designMatrix.getX(), indicesColumnIndex)),
        indexName(
            designMatrix.hasColumnNames() ?
                designMatrix.getColumnName(timeColumnIndex) :
                timeName
        )
    {}
    /**
     * @brief Build an IndexedDesignMatrix from given DesignMatrix and indices
     *  vector
     * @param designMatrix The design matrix (with no indices column)
     * @param indices The vector of indices
     * @param indexName The name for the index attribute
     */
    IndexedDesignMatrix(
        DesignMatrix<VarType> const &designMatrix,
        vector<IndexType> const &indices,
        string const indexName="index"
    ) :
        DesignMatrix(designMatrix),
        indices(indices),
        indexName(indexName)
    {}
    /**
     * @brief Build an IndexedDesignMatrix from given \f$X\f$ matrix and
     *  specified indices column
     * @param X The matrix containing both data points and its associated index
     * @param indicesColumnIndex Index of the column containing indices
     * @param indexName The name for the index attribute
     */
    IndexedDesignMatrix(
        arma::Mat<VarType> const &X,
        size_t const indicesColumnIndex,
        string const indexName="index"
    ) :
        DesignMatrix<VarType>(
            TemporalDesignMatrix<VarType>::extractNonTimeMatrix(
                X, indicesColumnIndex
            )
        ),
        t(extractIndices(X, indicesColumnIndex)),
        indexName(indexName)
    {}
    /**
     * @brief Build an IndexedDesignMatrix from given DesignMatrix \f$X\f$
     *  and indices
     * @param X The DesignMatrix (with no indices column)
     * @param indices The indices
     * @param indexName The name for the index attribute
     */
    IndexedDesignMatrix(
        arma::Mat<VarType> const &X,
        vector<IndexType> const &indices,
        string const indexName="index"
    ) :
        DesignMatrix<VarType>(X),
        indices(indices),
        indexName(indexName)
    {}
    /**
     * @brief Build an IndexedDesignMatrix from data in file at given path and
     *  specified index column
     * @param path Path to the file containing both the data and the indices
     * @param indicesColumnIndex Index of the column containing indices
     * @param indexName The default name for the index attribute to be used in
     *  case the read DesignMatrix does not specify a column name for its
     *  indices column
     */
    IndexedDesignMatrix(
        string const &path,
        size_t const indicesColumnIndex,
        string const indexName="index"
    ) :
        TemporalDesignMatrix(
            DesignMatrix<VarType>(path),
            indicesColumnIndex,
            indexName
        )
    {
        // TODO Rethink : Implement as read and swap
    }
    virtual ~IndexedDesignMatrix() = default;


    // ***  OPERATORS  *** //
    // ******************* //
    /**
     * @brief Access to the \f$i\f$-th index
     * @param i The index (in the indices vector) of the index being accessed
     * @return Reference to the \f$i\f$-th index
     */
    inline IndexType& operator[] (size_t const i) {return indices[i];}


    // ***  METHODS  *** //
    // ***************** //
    // TODO Rethink : Implement toDiffDesignMatrix methods like TemporalDesignMatrix does
    /**
     * @brief Build a DiffDesignMatrix from the IndexedDesignMatrix assuming
     *  that the index 0 lies at \f$t_a\f$ while the maximum index lies at
     *  \f$t_b\f$. All \f$m\f$ indices are expected to be linearly spaced with
     *  a step \f$\Delta t = \frac{t_b - t_a}{m}\f$
     *
     * @param ta The start time
     * @param tb The end time
     * @return DiffDesignMatrix built from IndexedDesignMatrix
     * @see toLinearTimeDiffDesignMatrixPointer(double const, double const)
     * @see fluxionum::DiffDesignMatrix
     */
    DiffDesignMatrix toLinearTimeDiffDesignMatrix(
        double const ta=0.0,
        double const tb=1.0
    ) const {
        return DiffDesignMatrix(
            TemporalDesignMatrix(
                getX(),
            )
        )
    }
    /**
     * @see Like toLinearTimeDiffDesignMatrix(double const, double const)
     * @return Pointer to the DiffDesignMatrix built from IndexedDesignMatrix
     * @see toLinearTimeDiffDesignMatrix(double const, double const)
     * @see fluxionum::DiffDesignMatrix
     */
    shared_ptr<DiffDesignMatrix> toLinearTimeDiffDesignMatrixPointer(
        double const ta=0.0,
        double const tb=1.0
    ) const;



    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain a constant/read reference to the vector of indices
     * @return Constant/read reference to the vector of indices
     * @see fluxionum::IndexedDesignMatrix::indices
     */
    inline vector<IndexType> const & getIndices() const {return indices;}
};

}