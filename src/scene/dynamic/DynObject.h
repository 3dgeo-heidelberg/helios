#pragma once

#include <string>
#include <vector>
#include <armadillo>

#include <assetloading/ScenePart.h>

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
class DynObject : public ScenePart{
private:
    // ***  SERALIZATION  *** //
    // ********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic object to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic object
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<DynObject, ScenePart>();
        ar &boost::serialization::base_object<ScenePart>(*this);
    }

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic object default constructor
     */
    DynObject() = default;
    /**
     * @brief Build the dynamic object from given scene part
     * @param sp Scene part as basis for dynamic object
     * @see ScenePart::ScenePart(ScenePart const &, bool const)
     */
    DynObject(ScenePart const &sp, bool const shallowPrimitives=false) :
        ScenePart(sp, shallowPrimitives)
    {}
    /**
     * @brief Dynamic object constructor with id as argument
     * @param id The id for the dynamic object
     * @see DynObject::id
     */
    DynObject(string const id) {setId(id);}
    /**
     * @brief Dynamic object constructor with primitives as argument
     * @param primitives The primitives defining the dynamic object
     * @see DynObject::primitives
     */
    DynObject(vector<Primitive *> const &primitives)
    {setPrimitives(primitives);}
    /**
     * @brief Dynamic object constructor with id and primitives as arguments
     * @param id The id for the dynamic object
     * @param primitives The primitives defining the dynamic object
     * @see DynObject::id
     * @see DynObject::primitives
     */
    DynObject(string const id, vector<Primitive *> const &primitives){
        setId(id);
        setPrimitives(primitives);
    }
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
     *
     * @return True if computing the step has modified the dynamic object.
     *  False if no changes have been performed.
     * @see DynObject::operator()()
     */
    virtual bool doStep() = 0;
    /**
     * @brief Alias for DynObject::doStep method so it can be used as a functor
     * @see DynObject::doStep
     */
    inline void operator() () {doStep();}

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
    size_t countVertices() const;
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
    arma::mat positionMatrixFromPrimitives() const;
    /**
     * @brief Like DynObject::positionMatrixFromPrimitives but receiving
     *  the total number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing it as \f$m\f$. Otherwise, it will be computed for
     *  each call of positionMatrixFromPrimitives method which implies
     *  iterating through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::positionMatrixFromPrimitives
     * @see DynObject::countVertices
     */
    arma::mat positionMatrixFromPrimitives(size_t const m) const;
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
    arma::mat normalMatrixFromPrimitives() const;
    /**
     * @brief Like DynObject::normalMatrixFromPrimitives but receiving
     *  the total number of vertices beforehand.
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing it as \f$m\f$. Otherwise, it will be computed for
     *  each call of normalMatrixFromPrimitives method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::normalMatrixFromPrimitives
     * @see DynObject::countVertices
     */
    arma::mat normalMatrixFromPrimitives(size_t const m) const;
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
     * @see updatePrimitivesPositionFromMatrix(size_t const, arma::mat const &)
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
     * @see updatePrimitivesPositionFromMatrix(arma::mat const &)
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
     * @see updatePrimitivesNormalFromMatrix(size_t const, arma::mat const &)
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
     * @see updatePrimitivesNormalFromMatrix(arma::mat const &)
     */
    void updatePrimitivesNormalFromMatrix(size_t const m, arma::mat const &X);
protected:
    /**
     * @brief Build a matrix from the set of primitives defining the dynamic
     *  object.
     *
     * Let \f$X\f$ be the built matrix:
     *
     * \f[
     *  X = \left[\begin{array}{lll}
     *      x_{11} &\ldots& x_{1n} \\
     *      \vdots &\ddots& \vdots \\
     *      x_{m1} &\ldots& x_{mn}
     *  \end{array}\right]
     * \f]
     *
     * Now, let \f$\vec{x}_{j}=\left(x_{1j}, \ldots, x_{mj}\right)\f$ be the
     *  \f$j\f$-th column vector of matrix \f$X\f$, \f$p_j\f$ be the
     *  \f$j\f$-th vertex defining the dynamic object and \f$f\f$ be the
     *  get function. Thus, the usage of the get function can be defined
     *  as follows:
     *
     * \f[
     *  \vec{x}_{j} = f\left(p_j\right)
     * \f]
     *
     * @param get Function to extract a column vector for a vertex which will
     *  be inserted in the matrix
     * @return Built matrix
     */
    arma::mat matrixFromPrimitives(
        std::function<arma::colvec(Vertex const *)> get
    ) const ;
    /**
     * @brief Like DynObject::matrixFromPrimitives but receiving the total
     *  number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing it as \f$m\f$. Otherwise, it will be computed for
     *  each call of matrixFromPrimitives method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::matrixFromPrimitives
     * @see DynObject::countVertices
     */
    arma::mat matrixFromPrimitives(
        size_t const m,
        std::function<arma::colvec(Vertex const *)> get
    ) const ;
    /**
     * @brief Update primitives defining the dynamic object from given matrix.
     *
     * Let \f$X\f$ be the input matrix:
     *
     * \f[
     *  X = \left[\begin{array}{lll}
     *      x_{11} &\ldots& x_{1n} \\
     *      \vdots &\ddots& \vdots \\
     *      x_{m1} &\ldots& x_{mn}
     *  \end{array}\right]
     * \f]
     *
     * Now, let \f$\vec{x}_{j}=\left(x_{1j}, \ldots, x_{mj}\right)\f$ be the
     *  \f$j\f$-th column vector of matrix \f$X\f$, \f$p_j\f$ be the
     *  \f$j\f$-th vertex defining the dynamic object and \f$f\f$ be the
     *  set function. Thus, the usage of the set function can be defined
     *  as follows:
     *
     * \f[
     *  p_j = f\left(\vec{x}_{j}\right)
     * \f]
     *
     * @param set Function to update a vertex from corresponding column vector
     *  in given matrix
     * @param X Matrix containing necessary data to update vertices. Each
     *  column contains the data for its corresponding vertex
     */
    void matrixToPrimitives(
        std::function<void(Vertex *, arma::colvec const &)> set,
        arma::mat const &X
    );
    /**
     * @brief Like DynObject::matrixToPrimitives but receiving the total
     *  number of vertices beforehand
     *
     * It can increase efficiency when the number of primitives defining the
     *  dynamic object remains the same by caching the countVertices method
     *  output and passing it as \f$m\f$. Otherwise, it will be computed for
     *  each call of matrixToPrimitives method which implies iterating
     *  through vertices of each primitive.
     *
     * @param m Total number of vertices
     * @see DynObject::matrixToPrimitives
     * @see DynObject::countVertices
     */
    void matrixToPrimitives(
        size_t const m,
        std::function<void(Vertex *, arma::colvec const &)> set,
        arma::mat const &X
    );

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see ScenePart::getType
     */
    ObjectType getType() const override {return ObjectType::DYN_OBJECT;}
};
