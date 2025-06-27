#include "Vectorial.h"
#include "MathConstants.h"

double
Vectorial::directionToAngleXY(glm::dvec3 const u, bool positive)
{
  double angle = std::atan2(u.x, u.y);
  if (positive && angle < 0.0)
    angle += PI_2;
  return angle;
}

double
Vectorial::shortestRotationSign(double theta)
{
  double sign = -1.0;
  if (theta > M_PI)
    sign = 1.0;
  if (theta < 0.0 && theta > -M_PI)
    sign = 1.0;
  return sign;
}

double
Vectorial::shortestRotationSign(double alpha, double beta)
{
  if (alpha < 0.0)
    alpha += PI_2;
  if (beta < 0.0)
    beta += PI_2;
  return shortestRotationSign(beta - alpha);
}

double
Vectorial::shortestRotationSignXY(glm::dvec3 const u, glm::dvec3 const v)
{
  return shortestRotationSign(directionToAngleXY(u), directionToAngleXY(v));
}
