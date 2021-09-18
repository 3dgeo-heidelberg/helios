#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_member.hpp>

#include <glm/glm.hpp>
#include <serial.h>

#include "AABB.h"
#include "Asset.h"
#include "DetailedVoxel.h"
#include "Triangle.h"
#include "Vertex.h"
#include "Voxel.h"

#include "KDTreeNodeRoot.h"
#include <KDTreeFactory.h>
#include <SimpleKDTreeFactory.h>
#include <SAHKDTreeFactory.h>
#include <AxisSAHKDTreeFactory.h>
#include <FastSAHKDTreeFactory.h>

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
   * @brief Handle scene save operation
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param s Scene to be saved
   * @param version Version number for the Scene
   */
  template <class Archive>
  void save (
    Archive &ar,
    unsigned int const version
  ) const {
    // Register primitive derivates
    ar.template register_type<Vertex>();
    ar.template register_type<AABB>();
    ar.template register_type<Triangle>();
    ar.template register_type<Voxel>();
    ar.template register_type<DetailedVoxel>();

    // Register KDTree factories
    ar.template register_type<SimpleKDTreeFactory>();
    ar.template register_type<SAHKDTreeFactory>();
    ar.template register_type<AxisSAHKDTreeFactory>();
    ar.template register_type<FastSAHKDTreeFactory>();

    // Save the scene itself
    boost::serialization::void_cast_register<Scene, Asset>();
    ar &boost::serialization::base_object<Asset>(*this);
    ar &kdtf;
    //ar &kdtree; // KDTree not saved because it might be too deep
    ar &bbox;
    ar &bbox_crs;
    ar &primitives;
    ar &parts;
  }
  /**
   * @brief Handle scene load operation
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param s Scene to be loaded
   * @param version Version number for the Scene
   */
  template <class Archive>
  void load(
    Archive &ar,
    unsigned int const fileVersion
  ){
    // Register primitive derivates
    ar.template register_type<Vertex>();
    ar.template register_type<AABB>();
    ar.template register_type<Triangle>();
    ar.template register_type<Voxel>();
    ar.template register_type<DetailedVoxel>();

    // Register KDTree factories
    ar.template register_type<SimpleKDTreeFactory>();
    ar.template register_type<SAHKDTreeFactory>();
    ar.template register_type<AxisSAHKDTreeFactory>();
    ar.template register_type<FastSAHKDTreeFactory>();

    // Load the scene itself
    boost::serialization::void_cast_register<Scene, Asset>();
    ar &boost::serialization::base_object<Asset>(*this);
    ar &kdtf;
    //ar &kdtree; // KDTree not loaded because it might be too deep
    ar &bbox;
    ar &bbox_crs;
    ar &primitives;
    ar &parts;

    // Build KDTree from primitives
    if(kdtf != nullptr){
      kdtree = std::shared_ptr<KDTreeNodeRoot>(
        kdtf->makeFromPrimitivesUnsafe(primitives)
      );
    }
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER();

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
   * @param safe Enable safe mode (true) or disable it (false). Safe mode is
   *  mainly useful for debugging purposes because it prevents certain changes
   *  such as modification of primitives ordering. Whenever safe mode is not
   *  necessary, it is recommended to set it to false as enabling it might
   *  lead to a slowdown
   * @return True if scene loading was successfully finished, false otherwise
   */
  bool finalizeLoading(bool const safe=false);
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
  virtual inline std::shared_ptr<KDTreeFactory> getKDTreeFactory() const
  {return kdtf;}
  /**
   * @brief Set the KDTree factory to be used by the scene
   * @param kdtf New KDTree factory to be used by the scene
   * @see Scene::kdtf
   */
  virtual inline void setKDTreeFactory(
      std::shared_ptr<KDTreeFactory> const kdtf
  )
  {this->kdtf = kdtf;}
  /**
   * @brief Obtain the KDTree used by the scene
   * @return KDTree used by the scene
   * @see Scene::kdtree
   */
  virtual inline std::shared_ptr<KDTreeNodeRoot> getKDTree() const
  {return kdtree;}
  /**
   * @brief Set the KDTree to be used by the scene
   * @param kdtree New KDTree to be used by the scene
   * @see Scene::kdtree
   */
  virtual inline void setKDTree(std::shared_ptr<KDTreeNodeRoot> const kdtree)
  {this->kdtree = kdtree;}

  /**
   * @brief Obtain the scene's bounding box
   * @return Scene's bounding box
   * @see Scene::bbox
   */
  virtual inline std::shared_ptr<AABB> getBBox() const {return bbox;}
  /**
   * @brief Set the scene's bounding box
   * @param bbox New bounding box for the scene
   * @see Scene::bbox
   */
  virtual inline void setBBox(std::shared_ptr<AABB> const bbox)
  {this->bbox = bbox;}
  /**
   * @brief Obtain the scene's coordinate reference system bounding box
   * @return Scene's coordinate reference system bounding box
   * @see Scene::bbox
   */
  virtual inline std::shared_ptr<AABB> getBBoxCRS() const {return bbox_crs;}
  /**
   * @brief Set the scene's coordinate reference system bounding box
   * @param bbox New coordinate reference system bounding box for the scene
   * @see Scene::bbox
   */
  virtual inline void setBBoxCRS(std::shared_ptr<AABB> const bbox)
  {this->bbox_crs = bbox;}

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