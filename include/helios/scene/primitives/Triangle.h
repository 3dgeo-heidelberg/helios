#pragma once

#include <helios/scene/primitives/AABB.h>
#include <helios/scene/primitives/Primitive.h>
#include <helios/scene/primitives/Vertex.h>
#include <iostream>

#include <helios/util/Color4f.h>

/**
 * @brief Class representing triangle primitive
 */
class Triangle : public Primitive
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a Triangle to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the Triangle
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<Triangle, Primitive>();
    ar& boost::serialization::base_object<Primitive>(*this);
    ar & faceNormal;
    ar & e1 & e2 & v0;
    ar & faceNormalSet;
    ar & eps;
    // ar &aabb;  // Not needed because it is built on construction
    // ar &verts;  // Not needed because they are in save/load construct
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Triangle face normal vector
   */
  glm::dvec3 faceNormal;
  /**
   * @brief Precomputed vertex operations to speed-up ray intersection
   */
  glm::dvec3 e1, e2, v0;
  /**
   * @brief Flag to specify if face normal has been setted (true) or not
   *  (false)
   */
  bool faceNormalSet = false;
  /**
   * @brief Decimal precision threshold for triangle computations
   */
  double eps = 0.0000001;
  /**
   * @brief Axis aligned bounding box containing the triangle
   * @see AABB
   */
  AABB* aabb = nullptr;

public:
  /**
   * @brief The 3 vertices defining the triangle
   */
  Vertex verts[3];

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Triangle constructor
   * @param v0 First triangle vertex
   * @param v1 Second triangle vertex
   * @param v2 Third triangle vertex
   * @see Triangle::verts
   */
  Triangle(Vertex v0, Vertex v1, Vertex v2);
  ~Triangle() override { delete aabb; }
  /**
   * @see Primitive::clone
   */
  Primitive* clone() override;
  /**
   * @see Primitive::_clone
   */
  void _clone(Primitive* p) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  friend std::ostream& operator<<(std::ostream& out, Triangle* t);
  /**
   * @see Primitive::getAABB
   */
  AABB* getAABB() override;
  /**
   * @see Primitive::getCentroid
   */
  glm::dvec3 getCentroid() override;
  /**
   * @brief Obtain triangle face normal vector
   * @return Triangle face normal vector
   * @see Triangle::faceNormal
   */
  glm::dvec3 getFaceNormal();
  /**
   * @see Primitive::getNumVertices
   */
  size_t getNumVertices() override { return 3; }
  /**
   * @see Primitive::getVertices
   */
  Vertex* getVertices() override;
  /**
   * @see Primitive::getIncidenceAngle_rad
   */
  double getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                               const glm::dvec3& rayDir,
                               const glm::dvec3& intersectionPoint) override;
  /**
   * @see Primitive::getRayIntersection
   */
  std::vector<double> getRayIntersection(const glm::dvec3& rayOrigin,
                                         const glm::dvec3& rayDir) override;
  /**
   * @see Primitive::getRayIntersectionDistance
   */
  double getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                    const glm::dvec3& rayDir) override;
  /**
   * @brief Naive computation of dot product (faster due to the absence of
   *  integrity checks)
   */
  inline double dotProductNaive(const glm::dvec3& v1, const glm::dvec3& v3);
  /**
   * @brief Naive computation of cross product (faster due to the absence of
   *  integrity checks)
   */
  inline glm::dvec3 crossProductNaive(const glm::dvec3& v1,
                                      const glm::dvec3& v2);

  /**
   * @see Primitive::update
   */
  void update() override;

  /**
   * @brief Build the axis aligned bounding box containing the triangle
   */
  void buildAABB();

  /**
   * @brief Build a string representation of the triangle
   * @return String representation of the triangle
   */
  std::string toString();

  /**
   * @brief Set color for all triangle vertices
   * @param color Color for all triangel vertices
   */
  void setAllVertexColors(Color4f color);
  /**
   * @brief Set all triangle vertices normals to the normal vector of
   *  triangle face
   */
  void setAllVertexNormalsFromFace();

  /**
   * @brief Compute the 2D area of the triangle
   * @return Triangle 2D area
   */
  double calcArea2D();
  /**
   * @brief Compute the 3D area of the triangle
   * @return Triangle 3D area
   */
  double calcArea3D();

  /**
   * @brief Compute the 2D euclidean distance (XY) for given vertices
   * @param v1 First vertex/point
   * @param v2 Second vertex/point
   * @return Euclidean distance over XY (2D euclidean distance)
   */
  inline double euclideanDistance2D(const glm::dvec3& v1, const glm::dvec3& v2);
};
