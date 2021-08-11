#pragma once

#include <string>
#include <vector>
#include <armadillo>

#include <scene/primitives/Primitive.h>

using std::vector;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dynamic object base implementation
 *
 * A dynamic object is a set of primitives which together define a single
 *  object. This object is expected to have a dynamic behavior which changes
 *  over time.
 */
class DynObject {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Identifier for the dynamic object
     */
    string id;
    /**
     * @brief Primitives defining the dynamic object
     */
    vector<Primitive *> primitives;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic object default constructor
     */
    DynObject() = default;
    /**
     * @brief Dynamic object constructor with id as argument
     * @param id The id for the dynamic object
     * @see DynObject::id
     */
    DynObject(string const id) : id(id) {}
    /**
     * @brief Dynamic object constructor with primitives as argument
     * @param primitives The primitives defining the dynamic object
     * @see DynObject::primitives
     */
    DynObject(vector<Primitive *> const &primitives) : primitives(primitives){}
    /**
     * @brief Dynamic object constructor with id and primitives as arguments
     * @param id The id for the dynamic object
     * @param primitives The primitives defining the dynamic object
     * @see DynObject::id
     * @see DynObject::primitives
     */
    DynObject(string const id, vector<Primitive *> const &primitives) :
        id(id),
        primitives(primitives)
    {}
    virtual ~DynObject() = default;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Any dynamic object concrete implementation must override this
     *  method providing dynamic behavior to the object.
     *
     * This method must be invoked each time a dynamic object must be updated
     *  because the time step has changed. For the sake of understanding, let
     *  \f$\mathcal{D}\f$ be the dynamic object so \f$\mathcal{D}_0\f$
     *  represents its status at start time. Now, the doStep method can be
     *  understood as a function \f$f\f$ which receives a dynamic object
     *  on a given time step \f$t\f$ and update it to correspond with next
     *  time step \f$t+1\f$:
     *
     * \f[
     *  \mathcal{D}_{t+1} = f\left(\mathcal{D}_{t}\right)
     * \f]
     * @see DynObject::operator()()
     */
    virtual void doStep() = 0;
    /**
     * @brief Alias for DynObject::doStep method so it can be used as a functor
     * @see DynObject::doStep
     */
    inline void operator() () {doStep();}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the primitives of the dynamic object
     * @return Dynamic object primitives
     * @see DynObject::primitives
     */
    inline vector<Primitive *> const & getPrimitives() const
    {return primitives;}
    /**
     * @brief Set the primitives of the dynamic object
     * @param primitives Dynamic object primitives
     * @see DynObject::primitives
     */
    inline void setPrimitives(vector<Primitive *> const &primitives)
    {this->primitives = primitives;}

    /**
     * @brief Obtain the ID of the dynamic object
     * @return Dynamic object ID
     * @see DynObject::id
     */
    inline string const &getId() const {return id;}
    /**
     * @brief Set the ID of the dynamic object
     * @param id Dynamic object ID
     * @see DynObject::id
     */
    inline void setId(const string &id) {this->id = id;}

    // ***  U T I L  *** //
    // ***************** //
    /**
     * @brief Count how many vertices there are defining the dynamic object
     *
     * It is the summation of the number of vertices for each primitive
     *
     * @return How many vertices there are defining the dynamic object
     * @see primitives
     */
    size_t countVertices();
    /**
     * @brief Obtain the position matrix for primitives defining the dynamic
     *  object
     *
     * \f[
     *  X_{m \times 3} = \left[\begin{array}{lll}
     *      x_1 & \ldots & x_m \\
     *      y_1 & \ldots & y_m \\
     *      z_1 & \ldots & z_m
     *  \end{array}\right]
     * \f]
     *
     * @return Position matrix
     */
    arma::mat positionMatrixFromPrimitives();
    /**
     * @brief Like DynObject::positionMatrixFromPrimitives but receiving
     *  the total number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing as \f$m\f$. Otherwise, it will be computed for each
     *  call of positionMatrixFromPrimitives method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::positionMatrixFromPrimitives
     * @see DynObject::countVertices
     */
    arma::mat positionMatrixFromPrimitives(size_t const m);
    /**
     * @brief Obtain the normal matrix for primitives defining the dynamic
     *  object
     *
     * \f[
     *  X_{m \times 3} = \left[\begin{array}{lll}
     *      \hat{n}_{x1} & \ldots & \hat{n}_{xm} \\
     *      \hat{n}_{y1} & \ldots & \hat{n}_{ym} \\
     *      \hat{n}_{z1} & \ldots & \hat{n}_{zm}
     *  \end{array}\right]
     * \f]
     *
     * @return Normal matrix
     */
    arma::mat normalMatrixFromPrimitives();
    /**
     * @brief Like DynObject::normalMatrixFromPrimitives but receiving
     *  the total number of vertices beforehand.
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing as \f$m\f$. Otherwise, it will be computed for each
     *  call of normalMatrixFromPrimitives method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::normalMatrixFromPrimitives
     * @see DynObject::countVertices
     */
    arma::mat normalMatrixFromPrimitives(size_t const m);
    /**
     * @brief Update the position of each primitive with given matrix
     *
     * The first column of matrix \f$X\f$ contains the coordinates for the
     *  new position of the first primitive. The last column of matrix \f$X\f$
     *  contains the coordinates for the new position of the last primitive.
     *  The \f$j\f$-th column of matrix \f$X\f$ contains the coordinates for
     *  the new position of the \f$j\f$-th primitive.
     *
     * @param X The position matrix containing new coordinates
     */
    void updatePrimitivesPositionFromMatrix(arma::mat const &X);
    /**
     * @brief Like updatePrimitivesPositionFromMatrix(arma::mat const &) but
     *  receiving the total number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing as \f$m\f$. Otherwise, it will be computed for each
     *  call of updatePrimitivesPositionFromMatrix method which implies
     *  iterating through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see updatePrimitivesPositionFromMatrix(size_t const, arma::mat const &)
     */
    void updatePrimitivesPositionFromMatrix(
        size_t const m,
        arma::mat const &X
    );
    /**
     * @brief Update the normal of each primitive with given matrix
     *
     * The first column of matrix \f$X\f$ contains the components for the new
     *  normal of the first primitive. The last column of matrix \f$X\f$
     *  contains the components for the new normal of the last primitive. The
     *  \f$j\f$-th column of matrix \f$X\f$ contains the components for the
     *  new normal of the \f$j\f$-th primitive.
     *
     * @param X The normal matrix containing new components
     */
    void updatePrimitivesNormalFromMatrix(arma::mat const &X);
    /**
     * @brief Like updatePrimitivesNormalFromMatrix(arma::mat const &) but
     *  receiving the total number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing as \f$m\f$. Otherwise, it will be computed for each
     *  call of updatePrimitivesNormalFromMatrix method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see updatePrimitivesNormalFromMatrix(size_t const, arma::mat const &)
     */
    void updatePrimitivesNormalFromMatrix(size_t const m, arma::mat const &X);
protected:
    arma::mat matrixFromPrimitives(
        std::function<arma::colvec(Vertex const *)> get
    );
    arma::mat matrixFromPrimitives(
        size_t const m,
        std::function<arma::colvec(Vertex const *)> get
    );
    void matrixToPrimitives(
        std::function<arma::colvec(Vertex *, arma::colvec const &)> set,
        arma::mat const &X
    );
    void matrixToPrimitives(
        size_t const m,
        std::function<arma::colvec(Vertex *, arma::colvec const &)> set,
        arma::mat const &X
    );
};
