#pragma once

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <glm/glm.hpp>
#include <vector>

#include <helios/assetloading/ScenePart.h>
#include <helios/noise/NoiseSource.h>
#include <helios/scene/IntersectionHandlingResult.h>
#include <helios/scene/Material.h>
#include <helios/util/HeliosException.h>

#include <helios/scene/dynamic/DynMovingObject.h>
#include <helios/scene/dynamic/DynObject.h>
#include <helios/scene/dynamic/DynSequentiableMovingObject.h>

class AABB;
class Vertex;

/**
 * @brief Abstract class defining the common behavior for all primitives
 */
class Primitive
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a Primitive to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the Primitive
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // Register ScenePart derived classes
    ar.template register_type<DynMovingObject>();
    ar.template register_type<DynSequentiableMovingObject>();

    // Debugging purposes ---
    /*std::string partId = "#NULLID#";
    if(part!=nullptr) partId = part->getId();*/
    // --- Debugging purposes

    ar & part;
    ar & material;
  }

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Shared pointer to the scene part the primitive belongs to
   * @see ScenePart
   */
  std::shared_ptr<ScenePart> part = nullptr;
  /**
   * @brief Shared pointer to the material defining certain properties such
   * as reflectance, specularity, ...
   * @see Material
   */
  std::shared_ptr<Material> material = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~Primitive() {}
  virtual Primitive* clone() = 0;
  virtual void _clone(Primitive* p);

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Obtain the axis aligned bounding box containing the primitive
   * @return Axis aligned bounding box containing the primitive
   */
  virtual AABB* getAABB() = 0;
  /**
   * @brief Obtain the primitive centroid
   * @return Primitive centroid
   */
  virtual glm::dvec3 getCentroid() = 0;
  /**
   * @brief Obtain the incidence angle in radians
   * @param rayOrigin Ray origin coordinates
   * @param rayDir Ray director vector
   * @param intersectionPoint Intersection point between ray and primitive
   * @return Incidence angle (radians)
   */
  virtual double getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                                       const glm::dvec3& rayDir,
                                       const glm::dvec3& intersectionPoint) = 0;
  /**
   * @brief Obtain the intersection point between primitive and given ray
   * @param rayOrigin Ray origin coordinates
   * @param rayDir Ray director vector
   * @return Intersection point between primitive and given ray
   */
  virtual std::vector<double> getRayIntersection(const glm::dvec3& rayOrigin,
                                                 const glm::dvec3& rayDir) = 0;
  /**
   * @brief Obtain the intersection distance between primitive and given ray
   * @param rayOrigin Ray origin coordinates
   * @param rayDir Ray director vector
   * @return Intersection distance between primitive and given ray. If no
   * intersection is possible, then < 0 should be returned (generally -1).
   */
  virtual double getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                            const glm::dvec3& rayDir) = 0;
  /**
   * @brief Method to be triggered once all Primitives have been loaded.
   *
   * It can be used to prepare primitives after being loaded but before
   * starting the simulation. It is not necessary that this method does
   * nothing at all, but it can be used by primitives which need it.
   *
   * @param[in] uniformNoiseSource Uniform noise source in range [-1, 1]
   *
   * @see DetailedVoxel
   */
  virtual void onFinishLoading(NoiseSource<double>& uniformNoiseSource) {}

  /**
   * @brief Obtain the number of vertices returned by the getVertices
   * function
   * @return Number of vertices returned by the getVertices function
   * @see Primitive::getVertices
   */
  inline virtual size_t getNumVertices() { return 0; }
  /**
   * @brief Obtain basic vertices for the primitive
   * @return Basic vertices for the primitive
   * @see Primitive::getFullVertices
   */
  virtual Vertex* getVertices() = 0;

  /**
   * @brief Obtain the number of vertices returned by the getFullVertices
   * function
   * @return Number of vertices returned by the getFullVertices function
   * @see Primitive::getFullVertices
   */
  inline virtual size_t getNumFullVertices() { return getNumVertices(); };
  /**
   * @brief Obtain full vertices for the primitive
   *
   * FullVertices is a set containing basic vertices and, when necessary,
   * those extra vertices required to define all the space occupied by the
   * primitive.
   *
   * Thi is necessary, for instance, for correct voxels ray intersection.
   *
   * @return Full vertices for the primitive
   * @see Primitive::getVertices
   */
  virtual Vertex* getFullVertices() { return getVertices(); };

  /**
   * @brief Offset for ground point z coordinate
   *
   * When finding ground points it is necessary to consider the
   * offset. For triangles or similar plane-like primitives, the offset
   * is 0. However, when considering primitives with volume, the z offset
   * must be considered to compute the ground surface and not the first
   * intersected point.
   *
   * Imagine the ray with originWaypoint at o is going through the given voxel.
   * It will intersect at p first. However, the ground point must be q,
   * because the ground is understood as the voxel upper surface.
   * This offset is used to assist computing the ground point (see
   * Scene getGrountPointAt function)
   *  ___q____
   * |        |
   * |        |
   * |___p____|
   *
   *     |
   *     |
   *     |
   *     o
   *
   * @return Offset for ground point z coordinate
   * @see Scene::getGroundPointAt
   */
  inline virtual double getGroundZOffset() { return 0; }

  /**
   * @brief Necessary primitive updates after modification.
   * @see Scene::finalizeLoading
   */
  virtual void update() = 0;

  // ***  RAY INTERSECTION HANDLING  *** //
  // *********************************** //
  /**
   * @brief Specify if the primitive can handle intersections or not
   *
   * By default primitives cannot handle intersections.
   * Primitives which can handle intersections should override this method
   * and return true.
   * This method is used to avoid unnecessary computations for those
   * primitives which are not going to handle intersections.
   *
   * @return True if the primitive can handle intersections, False otherwise
   */
  virtual inline bool canHandleIntersections() { return false; }
  /**
   * @brief Handle ray intersections
   *
   * This function acts as a callback to apply desired behavior when
   * intersection events are notified.
   *
   * @param uniformNoiseSource The uniform noise source used to handle
   * randomness simmulation
   * @param rayDirection The direction of the ray. It is necessary to apply
   * randomness while avoiding the intersection point to go outside
   * voxel limits
   * @param insideIntersectionPoint The point where the ray enters the
   * primitive
   * @param outsideIntersectionPoint The point where the ray exits the
   * primitive
   * @param rayIntensity Intensity for the ray intersecting the primitive
   * @param extra An extra argument that can be used by handlers which
   * need it
   * @return Result of handling the intersection
   */
  virtual IntersectionHandlingResult onRayIntersection(
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity);

  // ***  TRANSFORMATIONS  *** //
  // ************************* //
  /**
   * @brief Performs rotation over primitive
   * @param r Rotation to perform
   */
  virtual void rotate(Rotation& r);
  /**
   * @brief Scale primitive by given factor
   * @param factor Scale factor
   */
  virtual void scale(double const factor);
  /**
   * @brief Translate primitive by given shift
   * @param shift Translation vector
   */
  virtual void translate(glm::dvec3 const& shift);

  // ***  S I G M A  *** //
  // ******************* //
  /**
   * @brief Check if primitive can compute sigma using LadLut or not
   * @return True if primitive can compute sigma using LadLut,
   * false otherwise
   * @see LadLut
   */
  virtual bool canComputeSigmaWithLadLut() { return false; }
  /**
   * @brief Compute sigma using LadLut
   *
   * This computation will only be possible when primitive is able to do
   * it. For this purpose, a LadLut is required. Also, the primitive must
   * be able to use the LadLut.
   *
   * @return sigma
   * @see Primitive::canComputeIntensityWithLadLut
   * @see LadLut
   */
  virtual double computeSigmaWithLadLut(glm::dvec3 const& direction)
  {
    throw HeliosException("Primitive cannot compute sigma with LadLut");
  }

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  /**
   * @brief Output stream operator << overloading
   */
  friend std::ostream& operator<<(std::ostream& out, Primitive& p);
};
