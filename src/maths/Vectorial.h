#pragma once

#include <glm/glm.hpp>


/**
 * @brief Class to perform util vector operations
 */
class Vectorial{
public:
    // ***  CLASS METHODS  *** //
    // *********************** //
    /**
     * @brief Translate the normalized director vector to the angle it
     * corresponds in range \f$[0, 2\pi)\f$ if positive is requested,
     * in range \f$[-pi, pi]\f$ otherwise.
     *
     * For this purpose, only X and Y components are considered.
     *
     * @param u Director to be translated to angle over XY
     * @return Angle in range over XY for given director vector
     */
    static double directionToAngleXY(glm::dvec3 const u, bool positive=true);

    /**
     * @brief Let theta be the difference between two angles, alpha and beta:
     * \f$\theta = \beta - \alpha\f$. The sign of shortest rotation to go
     * from beta to alpha is computed by this function.
     *
     * @param theta Angular difference \f$\beta - \alpha\f$
     * @return Sign of shortest rotation
     */
    static double shortestRotationSign(double theta);

    /**
     * @brief Obtain the sign which minimizes the rotation magnitude to go
     * from alpha to beta
     *
     * Angles are expected to be expressed in interval \f$[0, 2pi)\f$,
     * corresponding to the direction to angle translation from
     * Vectorial::directionToAngleXY function.
     * Negative angles will be translated and work adequately, as long as they
     * were obtained through aforementioned function too.
     *
     * @param alpha Start direction angle
     * @param beta End direction angle
     * @return Sign of shortest rotation
     * @see Vectorial::directionToAngleXY
     * @see Vectorial::shortestRotationSign(double)
     */
    static double shortestRotationSign(double alpha, double beta);

    /**
     * @brief Like the Vectorial::shortestRotationSign function but taking
     * the angle from given normalized director vectors over XY
     * @param u First normalized director vector
     * @param v Second normalized director vector
     * @see Vectorial::directionToAngleXY
     * @see Vectorial::shortestRotationSign(double, double)
     */
    static double shortestRotationSignXY(
        glm::dvec3 const u,
        glm::dvec3 const v
    );
};