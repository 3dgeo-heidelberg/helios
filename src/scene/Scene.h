#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>

#include <glm/glm.hpp>

#include "serial.h"

#include "AABB.h"
#include "Asset.h"
#include "DetailedVoxel.h"
#include "Triangle.h"
#include "Vertex.h"
#include "Voxel.h"

#include "KDTreeNodeRoot.h"
#include <KDTreeFactory.h>
#include <SimpleKDTreeFactory.h>

#include "RaySceneIntersection.h"

/**
 * @brief Class representing a scene asset
 */
class Scene : public Asset {

private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a Scene to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the Scene
   */
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    // register primitive derivates
    ar.template register_type<AABB>();
    ar.template register_type<Triangle>();
    ar.template register_type<Vertex>();
    ar.template register_type<Voxel>();
    ar.template register_type<DetailedVoxel>();

    boost::serialization::void_cast_register<Scene, Asset>();
    ar &boost::serialization::base_object<Asset>(*this);
    ar &kdtf;
    ar &kdtree;
    ar &bbox;
    ar &bbox_crs;
    ar &primitives;
    ar &parts;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The KDTree factory used to build the scene KDTree
   * @see KDTreeFactory
   * @see Scene::kdtree
   */
  std::shared_ptr<KDTreeFactory> kdtf;
  /**
   * @brief KDTree splitting scene points/vertices to speed-up intersection
   *  computations
   */
  std::shared_ptr<KDTreeNodeRoot> kdtree;
  /**
   * @brief Axis aligned bounding box defining scene boundaries
   */
  std::shared_ptr<AABB> bbox;
  /**
   * @brief Original axis aligned bounding box defining scene boundaries
   *  before centering it
   */
  std::shared_ptr<AABB> bbox_crs;

public:
  /**
   * @brief Vector of primitives composing the scene
   */
  std::vector<Primitive *> primitives;
  /**
   * @brief Parts composing the scene with no repeats.
   *
   * Please avoid manually computing this vector. If it is necessary to
   *    initialize or update it, call registerParts instead.
   *
   * @see Scene::registerParts
   */
  std::vector<std::shared_ptr<ScenePart>> parts;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Scene default constructor
   */
  Scene() :
    kdtf(make_shared<SimpleKDTreeFactory>())
  {}
  ~Scene() override {
    for (Primitive *p : primitives)
      delete p;
  }
  Scene(Scene &s);

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Handle scene loading finish process
   *
   * Computations such as primitives update and centering the scene (all its
   *  vertices) so the axis aligned bounding box defining its boundaries
   *  start at (0, 0, 0) and also KDTree building are performed here
   *
   * @return True if scene loading was successfully finished, false otherwise
   */
  bool finalizeLoading();
  /**
   * @brief Register all scene parts composing the scene in the parts vector
   *    with no repetitions
   * @see Scene::parts
   */
  void registerParts();
  /**
   * @brief Obtain the axis aligned bounding box defining scene boundaries
   * @see Scene::bbox
   */
  std::shared_ptr<AABB> getAABB();
  /**
   * @brief Obtain the ground point at specified XY coordinates
   * @param point Point definint the XY coordinates for which the ground
   *  point shall be obtained
   * @return Intersected ground point
   */
  glm::dvec3 getGroundPointAt(glm::dvec3 point);
  /**
   * @brief Obtain the intersection between the ray and the scene, if any
   * @param rayOrigin Ray origin 3D coordinates
   * @param rayDir Ray 3D director vector
   * @param groundOnly Flag to specify if only ground points must be
   *  considered (true) or not (false)
   * @return Obtained intersection, nullptr if no intersection was detected
   * @see RaySceneIntersection
   * @see KDTreeRaycaster
   * @see KDTreeRaycaster::search
   */
  std::shared_ptr<RaySceneIntersection>
  getIntersection(glm::dvec3 &rayOrigin, glm::dvec3 &rayDir, bool groundOnly);
  /**
   * @brief Obtain all intersections between the ray and the scene, if any
   * @param rayOrigin Ray origin 3D coordinates
   * @param rayDir Ray 3D director vector
   * @param groundOnly Flag to specify if only ground points must be
   *  considered (true) or not (false)
   * @return Map of all primitives intersected by the ray, which key is the
   *  distance with respect to ray origin
   * @see RaySceneIntersection
   * @see KDTreeRaycaster
   * @see KDTreeRaycaster::searchAll
   */
  std::map<double, Primitive *>
  getIntersections(glm::dvec3 &rayOrigin, glm::dvec3 &rayDir, bool groundOnly);

  /**
   * @brief Obtain the minimum boundaries of the original axis aligned
   *  bounding box containing the scene, before it was centered so (0, 0, 0)
   *  became its new minimum boundaries
   * @return Minimum boundaries of the original axis aligned bounding box,
   *  before translating to (0, 0, 0)
   */
  glm::dvec3 getShift();

  /**
   * @brief Obtain all vertices (without repetitions) composing the scene
   * @return All vertices (without repetitions) composing the scene
   */
  std::vector<Vertex *> getAllVertices();

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the KDTree factory used by the scene
   * @return KDTree factory used by the scene
   * @see Scene::kdtf
   */
  std::shared_ptr<KDTreeFactory> getKDTreeFactory() const {return kdtf;}
  /**
   * @brief Set the KDTree factory to be used by the scene
   * @param kdtf New KDTree factory to be used by the scene
   * @see Scene::kdtf
   */
  void setKDTreeFactory(std::shared_ptr<KDTreeFactory> const kdtf)
  {this->kdtf = kdtf;}

  // ***   READ/WRITE  *** //
  // ********************* //
  /**
   * @brief Serialize the scene and write it to given output file
   * @param path Path to output file where serialized scene shall be stored
   */
  virtual void writeObject(std::string path);
  /**
   * @brief Read serialized scene from given file
   * @param path Path to file where a serialized scene is stored
   * @return Imported scene
   */
  static Scene *readObject(std::string path);
};