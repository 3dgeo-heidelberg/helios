#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <ostream>

#include "Color4f.h"

/**
 * @brief Class representing a vertex
 */
class Vertex
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Vertex 3D position
   */
  glm::dvec3 pos;
  /**
   * @brief Vertex normal vector
   */
  glm::dvec3 normal = glm::dvec3(0, 0, 0);
  /**
   * @brief Vertex texture coordinates
   */
  glm::dvec2 texcoords;
  /**
   * @brief Vertex color (RGBA)
   * @see Color4f
   */
  Color4f color;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default vertex constructor
   */
  Vertex() = default;
  /**
   * @brief Build a vertex with given position coordinates
   * @param x \f$x\f$ position coordinate
   * @param y  \f$y\f$ position coordinate
   * @param z \f$z\f$ position coordinate
   */
  Vertex(double const x, double const y, double const z)
    : pos(x, y, z) {};
  Vertex(const Vertex& v);

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Copy the vertex
   * @return Copy of vertex
   */
  inline Vertex copy() const { return Vertex(*this); }
  /**
   * @brief Matrix x Vector multiplication
   * @param mat Matrix to multiply
   * @param vec Vector to multiply
   * @return Multiplication result (3 double components)
   */
  static double* matxvec(double** mat, double* vec);
  /**
   * @brief Build a vertes result of rotating another vertex (v)
   * @param v Vertex to rotate
   * @param rotationMatrix Matrix defining rotation to be applied
   * @return Rotated vertex
   */
  static Vertex rotateVertex(Vertex v, double** rotationMatrix);

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Get the X coordinate of vertex position
   * @return X coordinate of vertex position
   * @see Vertex::pos
   */
  inline double getX() const { return this->pos.x; }

  /**
   * @brief Get the Y coordinate of vertex position
   * @return Y coortainte of vertex position
   * @see Vertex::pos
   */
  inline double getY() const { return this->pos.y; }

  /**
   * @brief Get the Z coordinate of vertex position
   * @return Z coordinate of vertex position
   * @see Vertex::pos
   */
  inline double getZ() const { return this->pos.z; }

  /**
   * @brief Compare if two vertex are equal
   *
   * Two vertex are considered to be equal when their positions are exactly
   *  the same
   *
   * @param v Vertex to compare with
   * @return True if vertex are equla, false otherwise
   * @see Vertex::pos
   */
  inline bool operator==(const Vertex& v) const
  {
    return this->pos.x == v.pos.x && this->pos.y == v.pos.y &&
           this->pos.z == v.pos.z;
  }

  friend std::ostream& operator<<(std::ostream& out, Vertex* v);
};

/**
 * @brief Struct to compare vertex when using unordered set
 * @see Vertex
 * @see AABB::getForVertices
 * @see std::unordered_set
 */
struct VertexKeyEqual
{
  bool operator()(Vertex* const& a, Vertex*& b) const
  {
    return a->getX() == b->getX() && a->getY() == b->getY() &&
           a->getZ() == b->getZ();
  }
};

/**
 * @brief Struct to obtain vertex hash when using unordered set
 * @see Vertex
 * @see AABB::getForVertices
 * @see std::unordered_set
 * @see std::hash
 */
struct VertexKeyHash
{
  size_t operator()(Vertex* const& v) const
  {
    std::hash<double> doubleHasher;
    const size_t prime = 31;
    size_t hash = 1;
    hash = prime * hash + doubleHasher(v->getX());
    hash = prime * hash + doubleHasher(v->getY());
    hash = prime * hash + doubleHasher(v->getZ());
    return hash;
  }
};
