#include <helios/maths/RotationOrder.h>

const glm::dvec3 RotationOrder::ZERO = glm::dvec3(0, 0, 0);
const glm::dvec3 RotationOrder::PLUS_I = glm::dvec3(1, 0, 0);
const glm::dvec3 RotationOrder::MINUS_I = glm::dvec3(-1, 0, 0);
const glm::dvec3 RotationOrder::PLUS_J = glm::dvec3(0, 1, 0);
const glm::dvec3 RotationOrder::MINUS_J = glm::dvec3(0, -1, 0);
const glm::dvec3 RotationOrder::PLUS_K = glm::dvec3(0, 0, 1);
const glm::dvec3 RotationOrder::MINUS_K = glm::dvec3(0, 0, -1);

const RotationOrder RotationOrder::XYZ =
  RotationOrder("XYZ", PLUS_I, PLUS_J, PLUS_K);
const RotationOrder RotationOrder::XZY =
  RotationOrder("XZY", PLUS_I, PLUS_K, PLUS_J);
const RotationOrder RotationOrder::YXZ =
  RotationOrder("YXZ", PLUS_J, PLUS_I, PLUS_K);
const RotationOrder RotationOrder::YZX =
  RotationOrder("YZX", PLUS_J, PLUS_K, PLUS_I);
const RotationOrder RotationOrder::ZXY =
  RotationOrder("ZXY", PLUS_K, PLUS_I, PLUS_J);
const RotationOrder RotationOrder::ZYX =
  RotationOrder("ZYX", PLUS_K, PLUS_J, PLUS_I);
const RotationOrder RotationOrder::XYX =
  RotationOrder("XYX", PLUS_I, PLUS_J, PLUS_I);
const RotationOrder RotationOrder::XZX =
  RotationOrder("XZX", PLUS_I, PLUS_K, PLUS_I);
const RotationOrder RotationOrder::YXY =
  RotationOrder("YXY", PLUS_J, PLUS_I, PLUS_J);
const RotationOrder RotationOrder::YZY =
  RotationOrder("YZY", PLUS_J, PLUS_K, PLUS_J);
const RotationOrder RotationOrder::ZXZ =
  RotationOrder("ZXZ", PLUS_K, PLUS_I, PLUS_K);
const RotationOrder RotationOrder::ZYZ =
  RotationOrder("ZYZ", PLUS_K, PLUS_J, PLUS_K);
