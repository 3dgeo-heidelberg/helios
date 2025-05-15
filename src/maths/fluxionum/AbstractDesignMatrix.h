#pragma once

#include <fluxionum/FluxionumException.h>

#include <sstream>
#include <string>
#include <vector>

namespace fluxionum {

using std::string;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief The abstract class which represents the fundamentals of any design
 *  matrix
 *
 * @tparam T Type \f$\mathcal{T}\f$ of element for the design matrix \f$X\f$
 *  such that \f$x_{ij} \in \mathcal{T}\f$.
 *  @see fluxionum::DesignMatrix
 */
template<typename T>
class AbstractDesignMatrix
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The column names for the DesignMatrix. It can be either an
   *  empty vector when no column names are specified or a vector with
   *  as many names as columns (in the same order)
   */
  vector<string> columnNames;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the AbstractDesignMatrix
   * @param columnNames Either the name for each column or an empty vector
   *  if there are no names
   */
  AbstractDesignMatrix(vector<string> const& columnNames = vector<string>(0))
    : columnNames(columnNames)
  {
  }
  virtual ~AbstractDesignMatrix() = default;

  // ***  INNER UTILS  *** //
  // ********************* //
  /**
   * @brief Find the corresponding column name for given index
   * @param columnName The name of the column which index must be obtained
   * @return The index of the column which names matches given one. If none,
   *  then a fluxionum::FluxionumException will be thrown
   */
  inline size_t translateColumnNameToIndex(string const& columnName) const
  {
    for (size_t i = 0; i < columnNames.size(); ++i) {
      if (columnNames[i] == columnName)
        return i;
    }
    std::stringstream ss;
    ss << "AbstractDesignMatrix::translateColumnNameToIndex failed to "
       << "find a column with name: \"" << columnName << "\"";
    throw FluxionumException(ss.str());
  }

public:
  // ***  OPERATORS  *** //
  // ******************* //
  /**
   * @brief Access operator for the element at \f$i\f$-th row and \f$j\f$-th
   *  column of the AbstractDesignMatrix
   * @param i The row of the element being accessed
   * @param j The column of the element being accessed
   * @return Reference to the element at \f$i\f$-th row and \f$j\f$-th
   *  column
   */
  virtual T& operator()(size_t const i, size_t const j) = 0;
  /**
   * @brief Like the
   *  AbstractDesignMatrix::operator()(size_t const, size_t const)
   *  method but specifying the column by its name instead of its index
   * @param columnName The name of the column of the element being accessed
   * @see AbstractDesignMatrix::operator()(size_t const, size_t const)
   */
  inline T& operator()(size_t const i, string const columnName)
  {
    return operator()(i, translateColumnNameToIndex(columnName));
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Check whether there are available column names for the
   *  AbstractDesignMatrix (true) or not (false)
   * @return True if there are available column names, false otherwise
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  inline bool hasColumnNames() const { return !columnNames.empty(); }
  /**
   * @brief Obtain the name of the \f$j\f$-th column
   * @param j Index of the column which name must be obtained
   * @return The name of the \f$j\f$-th column
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  inline string const& getColumnName(size_t const j) const
  {
    return columnNames[j];
  }
  /**
   * @brief Set the name of the \f$j\f$-th column
   * @param j Index of the column which name must be setted
   * @param columnName New name for the \f$j\f$-th column
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  inline void setColumnName(size_t const j, string const& columnName)
  {
    columnNames[j] = columnName;
  }
  /**
   * @brief Obtain a constant/read reference to the column names
   * @return Constant/read reference to the column names
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  inline vector<string> const& getColumnNames() const { return columnNames; }
  /**
   * @brief Obtain a constant/read reference to the column names
   * @param columnNames The new column names for the AbstractDesignMatrix
   * @see fluxionum::AbstractDesignMatrix::columnNames
   */
  inline void setColumnNames(vector<string> const& columnNames)
  {
    this->columnNames = columnNames;
  }
  /**
   * @brief Obtain the number of rows of the AbstractDesignMatrix
   * @return The number of rows of the AbstractDesignMatrix
   */
  virtual inline size_t getNumRows() const = 0;
  /**
   * @brief Obtain the number of columns of the AbstractDesignMatrix
   * @return The number of columns of the AbstractDesignMatrix
   */
  virtual inline size_t getNumColumns() const = 0;
  /**
   * @brief Obtain the number of elements of the AbstractDesignMatrix
   * @return The number of elements of the AbstractDesignMatrix
   */
  virtual inline size_t getNumElements() const = 0;
};

}
