#pragma once

#include <assetloading/SwapOnRepeatHandler.h>
class Primitive;
class AABB;
class WavefrontObj;

#include <LadLut.h>
#include <Vertex.h>
#include <maths/Rotation.h>

#include <armadillo>
#include <iostream>
#include <memory>
#include <ogr_geometry.h>
#include <vector>

/**
 * @brief Class representing a scene part
 */
class ScenePart
{
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a ScenePart to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the ScenePart
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & mPrimitives;
    ar & centroid;
    ar & bound;
    ar & mId;
    ar & subpartLimit;
    ar & onRayIntersectionMode;
    ar & onRayIntersectionArgument;
    ar & randomShift;
    ar & ladlut;
    ar & mOrigin;
    ar & mRotation;
    ar & mScale;
    ar & forceOnGround;
  }

public:
  // ***  TYPE ENUM  *** //
  // ******************** //
  /**
   * @brief Specify the type of scene part / object.
   * By default, the ScenePart is a static object.
   */
  enum ObjectType
  {
    STATIC_OBJECT,
    DYN_OBJECT,
    DYN_MOVING_OBJECT
  };
  /**
   * @brief Specify the type of primitive used to build the scene part
   * By default, the ScenePart is not build from primitives (None)
   * @see ScenePart::primitiveType
   */
  enum PrimitiveType
  {
    NONE,
    TRIANGLE,
    VOXEL
  };
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The type of primitive used to build the scene part
   * @see ScenePart::PrimitiveType
   */
  PrimitiveType primitiveType;
  /**
   * @brief Vector of pointers to primitives used by this scene part
   */
  std::vector<Primitive*> mPrimitives;
  /**
   * @brief The centroid of the scene part
   */
  arma::colvec centroid;
  /**
   * @brief The axis aligned bounding box defining scene part boundaries
   */
  std::shared_ptr<AABB> bound = nullptr;

  /**
   * @brief Identifier for the scene part
   */
  std::string mId = "";
  /**
   * @brief Vector specifying the limit of a subpart as the index of first
   *  element of next subpart.
   *
   * This vector makes sense when a scene part is built from multiple
   *  objects, so each one must be placed on its own scene part.
   *
   * The ith subpart is defined inside interval \f$[u[i-1], u[i])\f$, where
   *  \f$u\f$ is the subpartLimit vector. For the first case, the interval
   *  is defined as \f$[0, u[i])\f$
   * Having \f$|u| = 1\f$ means there is only one scene part
   */
  std::vector<size_t> subpartLimit;
  /**
   * @brief Specify the handling mode for ray intersections
   */
  std::string onRayIntersectionMode = "";
  /**
   * @brief Specify the extra value to be used for ray intersection handling
   * computation, when needed (depends on mode).
   */
  double onRayIntersectionArgument = 0.0;
  /**
   * @brief Specify if apply random shift to the scene part (true)
   * or not (false, by default)
   */
  bool randomShift = false;

  /**
   * @brief Look-up table for leaf angle distribution
   */
  std::shared_ptr<LadLut> ladlut = nullptr;

  /**
   * @brief Specify the origin for the scene part
   */
  glm::dvec3 mOrigin = glm::dvec3(0, 0, 0);
  /**
   * @brief Specify the rotation for the scene part
   */
  Rotation mRotation = Rotation(glm::dvec3(1, 0, 0), 0);
  /**
   * @brief Specify the scale for the scene part
   */
  double mScale = 1;
  /**
   * @brief Force the scene part to be on ground if \f$\neq 0\f$, do nothing
   *  if \f$= 0\f$.
   *
   * The force on ground process assumes the ground can be reduced to a
   *  plane and uses a search process:
   * <br/><b>-1</b> : Force on ground with optimum vertex, it is the one
   *  which is closest to the ground plane. Optimum solution with respect
   *  to ground plane is guaranteed but it is the most computationally
   *  expensive mode
   * <br/><b>0</b> : Dont force on ground at all
   * <br/><b>1</b> : Uses the minimum \f$z\f$ vertex to calculate to ground
   *  translation. It is the least computationally expensive mode
   * <br/><b>>1</b> : Uses a search process considering as many search steps
   *  as given. It allows to approximate the optimum vertex while maintaining
   *  computational cost inside desired boundaries
   *
   * NOTICE forceOnGround will not force the object to be on ground if it
   *  is a dynamic object which moves around the scene. It only assures the
   *  object will be INITIALLY placed at ground level
   */
  int forceOnGround = 0;

  OGRSpatialReference* mCrs = nullptr;
  OGREnvelope* mEnv = nullptr;

  /**
   * @brief The handler for scene parts that must swap its geometry for
   *  different simulation runs (repetitions).
   * @see SwapOnRepeatHandler
   * @see ScenePart::getSwapOnRepeatHandler
   * @see SimulationPlayer
   */
  std::shared_ptr<SwapOnRepeatHandler> sorh = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for a scene part
   * @param shallowPrimitives If true, primitives pointers will be the same
   *  for the copy and for the source. If false, then primitives for the
   *  copy will be cloned so they are at different memory regions. Notice
   *  that a shallow copy of the primitives implies that the primitives in
   *  the copy will have the source as their reference scene part. Use this
   *  mode (true) with caution.
   */
  ScenePart()
    : primitiveType(PrimitiveType::NONE)
  {
  }
  ScenePart(ScenePart const& sp, bool const shallowPrimitives = false);
  virtual ~ScenePart() {}

  // ***  COPY / MOVE OPERATORS  *** //
  // ******************************* //
  /**
   * @brief Copy assigment operator.
   */
  ScenePart& operator=(const ScenePart& rhs);

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Add the primitives of a WavefrontObj to the ScenePart
   * @param obj Pointer to a loaded OBJ
   */
  void addObj(WavefrontObj* obj);
  /**
   * @brief Obtain all vertices in the scene part
   * @return All vertices in the scene part
   */
  std::vector<Vertex*> getAllVertices() const;

  /**
   * @brief Smooth normals for each vertex computing the mean for each
   * triangle using it
   */
  void smoothVertexNormals();

  /**
   * @brief Split each subpart into a different scene part, with the first
   *  one corresponding to this scene part
   * @see subpartLimit
   * @return True when split was successfully performed, false otherwise
   */
  bool splitSubparts();

  /**
   * @brief Compute the default centroid for the scene part as the midrange
   *  point and its boundaries too
   *
   * Let \f$P = \left\{p_1, \ldots, p_m\right\}\f$ be the set of
   *  points/vertices defining the scene part, where
   *  \f$\forall p_i \in P,\, p_i=\left(p_{ix}, p_{iy}, p_{iz}\right)\f$. So
   *  \f$\vec{p}_{x}\f$ is the vector containing the \f$m\f$
   *  \f$x\f$-coordinates, \f$\vec{p}_{y}\f$ is the vector containing the
   *  \f$m\f$ \f$y\f$-coordinates and \f$\vec{p}_{z}\f$ is the vector
   *  containing the \f$m\f$ \f$z\f$-cordinates. Now, let \f$o\f$ be the
   *  centroid of the scene part, so it can be defined as
   *
   * \f[
   * \begin{array}{lll}
   *  o &=& \frac{1}{2} \left(
   *      \min\left(\vec{p}_x\right)+\max\left(\vec{p}_{x}\right),
   *      \min\left(\vec{p}_y\right)+\max\left(\vec{p}_{y}\right),
   *      \min\left(\vec{p}_z\right)+\max\left(\vec{p}_{z}\right)
   *  \right) \\
   *  &=& \frac{1}{2} \left(\frac{a+b}{2}\right)
   * \end{array}
   * \f]
   *
   * The boundaries of the scene part are defined as the set of the min and
   *  max vertices respectively \f$\{a, b\}\f$ where:
   *
   * \f[
   * \begin{array}{lll}
   *  a &=& \left(
   *      \min\left(\vec{p}_x\right),
   *      \min\left(\vec{p}_y\right),
   *      \min\left(\vec{p}_z\right)
   *  \right) \\
   *  b &=& \left(
   *      \max\left(\vec{p}_x\right),
   *      \max\left(\vec{p}_y\right),
   *      \max\left(\vec{p}_z\right)
   *  \right)
   * \end{array}
   * \f]
   *
   * @param computeBound True to specify boundaries must be computed, false
   *  to avoid it. They are already necessary to compute the centroid, so
   *  if they are needed, calling this function with computeBound=true will
   *  store them with no extra computations but the ones required to build
   *  the AABB object itself
   *
   * @see ScenePart::centroid
   * @see AABB
   */
  void computeCentroid(bool const computeBound = false);

  /*
   * @brief Release all the resources that belong to the scene part. Note
   *  that calling release implies that the scene part can no longer be
   *  used.
   */
  virtual void release();

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the primitives of the scene part
   * @return Scene part primitives
   * @see ScenePart::mPrimitives
   */
  inline std::vector<Primitive*> const& getPrimitives() const
  {
    return mPrimitives;
  }
  /**
   * @brief Set the primitives of the scene part
   * @param primitives Scene part primitives
   * @see ScenePart::mPrimitives
   */
  inline void setPrimitives(std::vector<Primitive*> const& primitives)
  {
    this->mPrimitives = primitives;
  }
  /**
   * @brief Obtain the centroid of the scene part
   * @return Scene part centroid
   * @see ScenePart::centroid
   */
  inline arma::colvec getCentroid() const { return centroid; }
  /**
   * @brief Set the centroid of the scene part
   * @param centroid New centroid for the scene part
   * @see ScenePart::centroid
   */
  inline void setCentroid(arma::colvec centroid) { this->centroid = centroid; }

  /**
   * @brief Obtain the ID of the scene part
   * @return Scene part ID
   * @see ScenePart::mId
   */
  inline std::string const& getId() const { return mId; }
  /**
   * @brief Set the ID of the scene part
   * @param id Scene part ID
   * @see ScenePart::id
   */
  inline void setId(const std::string& id) { this->mId = id; }
  /**
   * @brief Obtain the object type of the scene part
   * @return Object type of the scene part
   * @see ScenePart::ObjectType
   */
  virtual ObjectType getType() const { return ObjectType::STATIC_OBJECT; }
  virtual PrimitiveType getPrimitiveType() const { return primitiveType; }

  /**
   * @brief Obtain the swap on repeat handler of the scene part.
   * @return Swap on repeat handler of the scene part.
   * @see ScenePart::sorh
   */
  inline std::shared_ptr<SwapOnRepeatHandler> getSwapOnRepeatHandler()
  {
    return sorh;
  }

  /**
   * @brief Check whether the scene part is null, according to the underlying
   *  swap on repeat handler.
   * @return True if the scene part is null, false otherwise.
   * @see SwapOnRepeatHandler
   * @see SwapOnRepeatHandler::null
   * @see SwapOnRepeatHandler::isNull
   */
  inline bool isNull() const
  {
    return (sorh == nullptr) ? false : sorh->isNull();
  }

  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Compute the transformations for each primitive in the scene part.
   *  The transformations are typically specified through assetloading
   *  filters.
   * @param sp The ScenePart whose transformations must be computed.
   * @param holistic Whether to use an holistic approach for all the vertices
   *  (e.g., for points loaded from point clouds as input).
   * @see XmlSceneLoader
   * @see XmlSceneLoader::digestScenePart
   * @see SwapOnRepeatHandler
   * @see SimulationPlayer
   */
  static void computeTransformations(std::shared_ptr<ScenePart> sp,
                                     bool const holistic = false);
};
