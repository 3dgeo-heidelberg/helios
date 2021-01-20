/**
* This class is a utility representing a rotation order specification
* for Cardan or Euler angles specification.
*
* This class cannot be instanciated by the user. He can only use one
* of the twelve predefined supported orders as an argument to either
* the {@link Rotation#Rotation(RotationOrder,double,double,double)}
* constructor or the {@link Rotation#getAngles} method.
*
* @since 1.2
*/
#pragma once

#include <string>

#include <glm/glm.hpp>

class RotationOrder {

public:
	/** Null vector (coordinates: 0, 0, 0). */
	static const glm::dvec3 ZERO;

	/** First canonical vector (coordinates: 1, 0, 0). */
	static const glm::dvec3 PLUS_I;

	/** Opposite of the first canonical vector (coordinates: -1, 0, 0). */
	static const glm::dvec3 MINUS_I;

	/** Second canonical vector (coordinates: 0, 1, 0). */
	static const glm::dvec3 PLUS_J;

	/** Opposite of the second canonical vector (coordinates: 0, -1, 0). */
	static const glm::dvec3 MINUS_J;

	/** Third canonical vector (coordinates: 0, 0, 1). */
	static const glm::dvec3 PLUS_K;

	/** Opposite of the third canonical vector (coordinates: 0, 0, -1).  */
	static const glm::dvec3 MINUS_K;


	/** Set of Cardan angles.
	* this ordered set of rotations is around X, then around Y, then
	* around Z
	*/
public:
	static const RotationOrder XYZ;

	/** Set of Cardan angles.
	* this ordered set of rotations is around X, then around Z, then
	* around Y
	*/
	static const RotationOrder XZY;

	/** Set of Cardan angles.
	* this ordered set of rotations is around Y, then around X, then
	* around Z
	*/
	static const RotationOrder YXZ;

	/** Set of Cardan angles.
	* this ordered set of rotations is around Y, then around Z, then
	* around X
	*/
	static const RotationOrder YZX;

	/** Set of Cardan angles.
	* this ordered set of rotations is around Z, then around X, then
	* around Y
	*/
	static const RotationOrder ZXY;

	/** Set of Cardan angles.
	* this ordered set of rotations is around Z, then around Y, then
	* around X
	*/
	static const RotationOrder ZYX;

	/** Set of Euler angles.
	* this ordered set of rotations is around X, then around Y, then
	* around X
	*/
	static const RotationOrder XYX;

	/** Set of Euler angles.
	* this ordered set of rotations is around X, then around Z, then
	* around X
	*/
	static const RotationOrder XZX;

	/** Set of Euler angles.
	* this ordered set of rotations is around Y, then around X, then
	* around Y
	*/
	static const RotationOrder YXY;

	/** Set of Euler angles.
	* this ordered set of rotations is around Y, then around Z, then
	* around Y
	*/
	static const RotationOrder YZY;

	/** Set of Euler angles.
	* this ordered set of rotations is around Z, then around X, then
	* around Z
	*/
	static const RotationOrder ZXZ;

	/** Set of Euler angles.
	* this ordered set of rotations is around Z, then around Y, then
	* around Z
	*/
	static const RotationOrder ZYZ;

private:
	/** Name of the rotations order. */
	std::string name;

	/** Axis of the first rotation. */
	glm::dvec3 a1;

	/** Axis of the second rotation. */
	glm::dvec3 a2;

	/** Axis of the third rotation. */
	glm::dvec3 a3;

	/** Private constructor.
	* This is a utility class that cannot be instantiated by the user,
	* so its only constructor is private.
	* @param name name of the rotation order
	* @param a1 axis of the first rotation
	* @param a2 axis of the second rotation
	* @param a3 axis of the third rotation
	*/
	private: RotationOrder(const std::string _name,
		const glm::dvec3 a1, const glm::dvec3 a2, const glm::dvec3 a3) {
		name = _name;
		this->a1 = a1;
		this->a2 = a2;
		this->a3 = a3;
	}

	public:

	/** Get a string representation of the instance.
	* @return a string representation of the instance (in fact, its name)
	*/
	const std::string toString() {
		return name;
	}

	/** Get the axis of the first rotation.
	* @return axis of the first rotation
	*/
	const glm::dvec3 getA1() {
		return this->a1;
	}

	/** Get the axis of the second rotation.
	* @return axis of the second rotation
	*/
	const glm::dvec3 getA2() {
		return this->a2;
	}

	/** Get the axis of the second rotation.
	* @return axis of the second rotation
	*/
	const glm::dvec3 getA3() {
		return this->a3;
	}

};
