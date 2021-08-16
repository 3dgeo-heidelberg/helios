#pragma once

#include "RotationOrder.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#if PYTHON_BINDING
#include <PythonDVec3.h>
using pyhelios::PythonDVec3;
#endif

/**
* This class implements rotations in a three-dimensional space.
*
* <p>Rotations can be represented by several different mathematical
* entities (matrices, axe and angle, Cardan or Euler angles,
* quaternions). This class presents an higher level abstraction, more
* user-oriented and hiding this implementation details. Well, for the
* curious, we use quaternions for the internal representation. The
* user can build a rotation from any of these representations, and
* any of these representations can be retrieved from a
* <code>Rotation</code> instance (see the various constructors and
* getters). In addition, a rotation can also be built implicitly
* from a set of vectors and their image.</p>
* <p>This implies that this class can be used to convert from one
* representation to another one. For example, converting a rotation
* matrix into a set of Cardan angles from can be done using the
* following single line of code:</p>
* <pre>
* double[] angles = new Rotation(matrix, 1.0e-10).getAngles(RotationOrder.XYZ);
* </pre>
* <p>Focus is oriented on what a rotation <em>do</em> rather than on its
* underlying representation. Once it has been built, and regardless of its
* internal representation, a rotation is an <em>operator</em> which basically
* transforms three dimensional {@link Vector3D vectors} into other three
* dimensional {@link Vector3D vectors}. Depending on the application, the
* meaning of these vectors may vary and the semantics of the rotation also.</p>
* <p>For example in an spacecraft attitude simulation tool, users will often
* consider the vectors are fixed (say the Earth direction for example) and the
* frames change. The rotation transforms the coordinates of the vector in inertial
* frame into the coordinates of the same vector in satellite frame. In this
* case, the rotation implicitly defines the relation between the two frames.</p>
* <p>Another example could be a telescope control application, where the rotation
* would transform the sighting direction at rest into the desired observing
* direction when the telescope is pointed towards an object of interest. In this
* case the rotation transforms the direction at rest in a topocentric frame
* into the sighting direction in the same topocentric frame. This implies in this
* case the frame is fixed and the vector moves.</p>
* <p>In many case, both approaches will be combined. In our telescope example,
* we will probably also need to transform the observing direction in the topocentric
* frame into the observing direction in inertial frame taking into account the observatory
* location and the Earth rotation, which would essentially be an application of the
* first approach.</p>
*
* <p>These examples show that a rotation is what the user wants it to be. This
* class does not push the user towards one specific definition and hence does not
* provide methods like <code>projectVectorIntoDestinationFrame</code> or
* <code>computeTransformedDirection</code>. It provides simpler and more generic
* methods: {@link #applyTo(Vector3D) applyTo(Vector3D)} and {@link
* #applyInverseTo(Vector3D) applyInverseTo(Vector3D)}.</p>
*
* <p>Since a rotation is basically a vectorial operator, several rotations can be
* composed together and the composite operation <code>r = r<sub>1</sub> o
* r<sub>2</sub></code> (which means that for each vector <code>u</code>,
* <code>r(u) = r<sub>1</sub>(r<sub>2</sub>(u))</code>) is also a rotation. Hence
* we can consider that in addition to vectors, a rotation can be applied to other
* rotations as well (or to itself). With our previous notations, we would say we
* can apply <code>r<sub>1</sub></code> to <code>r<sub>2</sub></code> and the result
* we get is <code>r = r<sub>1</sub> o r<sub>2</sub></code>. For this purpose, the
* class provides the methods: {@link #applyTo(Rotation) applyTo(Rotation)} and
* {@link #applyInverseTo(Rotation) applyInverseTo(Rotation)}.</p>
*
* <p>Rotations are guaranteed to be immutable objects.</p>
*
* @see Vector3D
* @see RotationOrder
* @since 1.2
*/
class Rotation {
    // ***  SERIALIZATION  *** //
    // *********************** //
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version) {
		ar & q0 & q1 & q2 & q3;
	}

	// ***  ATTRIBUTES  *** //
	// ******************** //
	/** Scalar coordinate of the quaternion. */
	double q0, q1, q2, q3;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    Rotation() = default;
    Rotation(double q0, double q1, double q2, double q3, bool needsNormalization);
    Rotation(glm::dvec3 axis, double angle);
    Rotation(glm::dvec3 u, glm::dvec3 v);
    Rotation(RotationOrder order, double alpha1, double alpha2, double alpha3);
    virtual ~Rotation() = default;



	/** Identity rotation. */
	//const Rotation *IDENTITY = new Rotation(1.0, 0.0, 0.0, 0.0, false);

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	/**
	 * @brief Get the scalar coordinate of the quaternion.
	 * @return scalar coordinate of the quaternion
	 */
	double getQ0() {return q0;}
	void setQ0(double q0) {this->q0 = q0;}

	/**
	 * @brief Get the first coordinate of the vectorial part of the quaternion.
	 * @return first coordinate of the vectorial part of the quaternion
	 */
	double getQ1() {return q1;}
	void setQ1(double q1) {this->q1 = q1;}

	/**
	 * @brief Get the second coordinate of the vectorial part of the
	 * quaternion.
	 * @return second coordinate of the vectorial part of the quaternion
	 */
	double getQ2() {return q2;}
	void setQ2(double q2) {this->q2 = q2;}

	/**
	 * @brief Get the third coordinate of the vectorial part of the quaternion.
	 * @return third coordinate of the vectorial part of the quaternion
	 */
	double getQ3() {return q3;}
	void setQ3(double q3) {this->q3 = q3;}

	// ***  M E T H O D S  *** //
	// *********************** //
	Rotation revert();
	glm::dvec3 getAxis();
	double getAngle();
	double** getMatrix();
	glm::dvec3 applyTo(glm::dvec3 u);
	void applyTo(double* in, double* out);
	glm::dvec3 applyInverseTo(glm::dvec3 u);
	void applyInverseTo(double* in, double* out);
	Rotation applyTo(Rotation r);
	Rotation applyInverseTo(Rotation r);
	glm::dvec3 operator*(glm::dvec3 u) {return applyTo(u);}
	Rotation operator*(Rotation r) {return applyTo(r);}

    /**
     * @brief Get the roll, pitch and yaw for the Rotation
     * @param order Rotation order to use
     * @param[out] roll Roll angle (standard roll for XYZ order)
     * @param[out] pitch Pitch angle (standard pitch for XYZ order)
     * @param[out] yaw Yaw angle (standard yaw for XYZ order)
     */
	void getAngles(
	    RotationOrder const *order,
	    double &roll,
	    double &pitch,
	    double &yaw
    );

	friend std::ostream& operator << (std::ostream& out, Rotation& r);

#ifdef PYTHON_BINDING
	PythonDVec3 * getAxisPython() {return new PythonDVec3(getAxis());}
#endif

};