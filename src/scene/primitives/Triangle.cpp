#include "Triangle.h"

#define _USE_MATH_DEFINES
#include "math.h"

#include <algorithm>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>

using namespace glm;
using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Triangle::Triangle(Vertex v0, Vertex v1, Vertex v2) {
  verts[0] = v0;
  verts[1] = v1;
  verts[2] = v2;
  update();
}

Primitive *Triangle::clone() {
  Triangle *t = new Triangle(verts[0], verts[1], verts[2]);
  _clone(t);
  return t;
};

void Triangle::_clone(Primitive *p){
    Primitive::_clone(p);
    Triangle *t = (Triangle *) p;
    t->faceNormal = glm::dvec3(this->faceNormal);
    t->faceNormalSet = this->faceNormalSet;
    t->eps = this->eps;
    t->e1 = this->e1;
    t->e2 = this->e2;
    t->v0 = this->v0;
    delete t->aabb;
    if(aabb == nullptr) t->aabb = nullptr;
    else t->aabb = (AABB *) aabb->clone();
}

// ***  M E T H O D S  *** //
// *********************** //
AABB *Triangle::getAABB() {
  if (aabb == nullptr)
    buildAABB();
  return aabb;
}

dvec3 Triangle::getCentroid() {
  return (verts[0].pos + verts[1].pos + verts[2].pos) / 3.0;
}

dvec3 Triangle::getFaceNormal() {
  if (!faceNormalSet) {
    update();
    faceNormalSet = true; // TODO temp solution
  }
  return faceNormal;
}

double Triangle::getIncidenceAngle_rad(const glm::dvec3 &rayOrigin,
                                       const glm::dvec3 &rayDir,
                                       const glm::dvec3 &intersectionPoint) {
  return M_PI - glm::angle(faceNormal, rayDir);
}

// These naive methods are much faster than the built-in in Vector3D
inline double Triangle::dotProductNaive(const dvec3 &v1, const dvec3 &v3) {
  return v1.x * v3.x + v1.y * v3.y + v1.z * v3.z;
}

inline dvec3 Triangle::crossProductNaive(const dvec3 &v1, const dvec3 &v2) {
  return dvec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z,
               v1.x * v2.y - v1.y * v2.x);
}

/*
 * Fast, Minimum Storage Ray/Triangle Intersection (M�ller and Trumbore, 1997)
 * See http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
 */
std::vector<double> Triangle::getRayIntersection(const dvec3 &rayOrigin,
                                                 const dvec3 &rayDir) {
  dvec3 e1 = this->verts[1].pos - this->verts[0].pos;
  dvec3 e2 = this->verts[2].pos - this->verts[0].pos;
  dvec3 h = crossProductNaive(rayDir, e2);
  double a = dotProductNaive(e1, h);

  if (a > -eps && a < eps) {
    return std::vector<double>{-1};
  }

  double f = 1.0 / a;
  dvec3 s = rayOrigin - this->verts[0].pos;
  double u = f * dotProductNaive(s, h);

  if (u < 0.0 || u > 1.0) {
    return std::vector<double>{-1};
  }

  dvec3 q = crossProductNaive(s, e1);
  double v = f * dotProductNaive(rayDir, q);

  if (v < 0.0 || u + v > 1.0) {
    return std::vector<double>{-1};
  }

  double t = f * dotProductNaive(e2, q);

  if (t > eps) {
    return std::vector<double>{t};
  }

  return std::vector<double>{-1};
}

double Triangle::getRayIntersectionDistance(const glm::dvec3 &rayOrigin,
                                            const glm::dvec3 &rayDir) {
  // h = rayDir x e2 (cross product)
  const double hx = rayDir.y * e2.z - rayDir.z * e2.y;
  const double hy = rayDir.z * e2.x - rayDir.x * e2.z;
  const double hz = rayDir.x * e2.y - rayDir.y * e2.x;

  // a = <h, e1>
  const double a = hx * e1.x + hy * e1.y + hz * e1.z;
  if (a > -eps && a < eps)
    return -1.0;

  // s = rayOrigin - v0
  const double sx = rayOrigin.x - v0.x;
  const double sy = rayOrigin.y - v0.y;
  const double sz = rayOrigin.z - v0.z;

  // u = <s, h> / a
  const double u = (sx * hx + sy * hy + sz * hz) / a;
  if (u < 0.0 || u > 1.0)
    return -1.0;

  // q = s x e1 (cross product)
  const double qx = sy * e1.z - sz * e1.y;
  const double qy = sz * e1.x - sx * e1.z;
  const double qz = sx * e1.y - sy * e1.x;

  // v = <Rd, q> / a
  const double v = (rayDir.x * qx + rayDir.y * qy + rayDir.z * qz) / a;
  if (v < 0.0 || (u + v) > 1.0)
    return -1.0;

  // t = <e2, q> / a
  const double t = (e2.x * qx + e2.y * qy + e2.z * qz) / a;

  if (t > eps)
    return t;
  return -1.0;
}

Vertex *Triangle::getVertices() { return verts; }

void Triangle::update() {
  v0 = verts[0].pos;
  e1 = verts[1].pos - verts[0].pos;
  e2 = verts[2].pos - verts[0].pos;

  dvec3 normal_unnormalized =
      glm::cross(verts[1].pos - verts[0].pos, verts[2].pos - verts[0].pos);

  if (glm::l2Norm(normal_unnormalized) > 0) {
    this->faceNormal = glm::normalize(normal_unnormalized);
  }

  if (aabb != nullptr) {
    delete aabb;
  }
  buildAABB();
}

void Triangle::buildAABB() {
  double minX =
      std::min(std::min(verts[0].getX(), verts[1].getX()), verts[2].getX());
  double minY =
      std::min(std::min(verts[0].getY(), verts[1].getY()), verts[2].getY());
  double minZ =
      std::min(std::min(verts[0].getZ(), verts[1].getZ()), verts[2].getZ());
  dvec3 min = dvec3(minX, minY, minZ);

  double maxX =
      std::max(std::max(verts[0].getX(), verts[1].getX()), verts[2].getX());
  double maxY =
      std::max(std::max(verts[0].getY(), verts[1].getY()), verts[2].getY());
  double maxZ =
      std::max(std::max(verts[0].getZ(), verts[1].getZ()), verts[2].getZ());
  dvec3 max = dvec3(maxX, maxY, maxZ);

  aabb = new AABB(min, max);
}

string Triangle::toString() {
  stringstream ss;
  ss << verts[0].getX() << " " << verts[0].getY() << " " << verts[0].getZ()
     << endl;
  ss << verts[1].getX() << " " << verts[1].getY() << " " << verts[1].getZ()
     << endl;
  ss << verts[2].getX() << " " << verts[2].getY() << " " << verts[2].getZ()
     << endl;
  return ss.str();
}

void Triangle::setAllVertexColors(Color4f color) {
  verts[0].color = color;
  verts[1].color = color;
  verts[2].color = color;
}

void Triangle::setAllVertexNormalsFromFace() {
  verts[0].normal = faceNormal;
  verts[1].normal = faceNormal;
  verts[2].normal = faceNormal;
}

double Triangle::calcArea2D() {
  double det = verts[0].getX() * (verts[1].getY() - verts[2].getY()) +
               verts[1].getX() * (verts[2].getY() - verts[0].getY()) +
               verts[2].getX() * (verts[0].getY() - verts[1].getY());
  return 0.5 * std::fabs(det);
}

double Triangle::calcArea3D() {
  dvec3 ab = dvec3(verts[1].getX() - verts[0].getX(),
                   verts[1].getY() - verts[0].getY(),
                   verts[1].getZ() - verts[0].getZ());
  dvec3 ac = dvec3(verts[2].getX() - verts[0].getX(),
                   verts[2].getY() - verts[0].getY(),
                   verts[2].getZ() - verts[0].getZ());
  double cross = glm::l2Norm(crossProductNaive(ab, ac));
  return 0.5 * cross;
}

inline double Triangle::euclideanDistance2D(const dvec3 &v1, const dvec3 &v2) {
  double diffX = (v1.x - v2.x) * (v1.x - v2.x);
  double diffY = (v1.y - v2.y) * (v1.y - v2.y);
  return sqrt(diffX + diffY);
}

std::ostream &operator<<(std::ostream &out, Triangle *t) {
  out << t->getVertices() << " " << t->getVertices() + 1 << " "
      << t->getVertices() + 2;
  return out;
}
