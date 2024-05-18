#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_member.hpp>

#include <glm/glm.hpp>
#include <serial.h>

#include <AABB.h>
#include <Asset.h>
#include <DetailedVoxel.h>
#include <Triangle.h>
#include <Vertex.h>
#include <Voxel.h>

#include <KDTreeNodeRoot.h>
#include <KDGroveFactory.h>
#include <KDGrove.h>
#include <KDGroveRaycaster.h>

#include <RaySceneIntersection.h>

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

        // Save the scene itself
        boost::serialization::void_cast_register<Scene, Asset>();
        ar &boost::serialization::base_object<Asset>(*this);
        ar &kdgf;
        //ar &kdgrove; // KDGrove not saved because trees might be too deep
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

        // Load the scene itself
        boost::serialization::void_cast_register<Scene, Asset>();
        ar &boost::serialization::base_object<Asset>(*this);
        ar &kdgf;
        //ar &kdgrove; // KDTree not loaded because it might be too deep
        ar &bbox;
        ar &bbox_crs;
        ar &primitives;
        ar &parts;

        // Build KDTree from primitives
        if(kdgf != nullptr) buildKDGroveWithLog(true);
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER();

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
    /**
     * @brief The KDGrove factory used to build the scene KDGrove
     * @see KDGroveFactory
     * @see Scene::kdgrove
     */
    std::shared_ptr<KDGroveFactory> kdgf;
    /**
     * @brief KDGrove containing a KDTree for each scene part to speed-up
     *    ray-primitive intersection check computations
     */
    std::shared_ptr<KDGrove> kdgrove;
    /**
     * @brief Axis aligned bounding box defining scene boundaries
     */
    std::shared_ptr<AABB> bbox;
    /**
     * @brief Original axis aligned bounding box defining scene boundaries
     *  before centering it
     */
    std::shared_ptr<AABB> bbox_crs;
    /**
     * @brief The raycaster based on KDGrove
     * @see Scene::kdgrove
     * @see Scene::getIntersection
     * @see Scene::getIntersections
     */
    std::shared_ptr<KDGroveRaycaster> raycaster;
    /**
     * @brief The default reflectance that must be used for primitives with
     *  not reflectance (NaN). It is typically loaded through the spectral
     *  library and then assigned to the scene so it can be known when needed.
     * @see Scene::setDefaultReflectance
     * @see Scene::getDefaultReflectance
     * @see SpectralLibrary
     * @see SpectralLibrary::defaultReflectance
     * @see XmlSurveyLoader::createSurveyFromXml
     */
    double defaultReflectance = 50.0;

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
        kdgf(make_shared<KDGroveFactory>(make_shared<SimpleKDTreeFactory>()))
    {}
    ~Scene() override {
        for (Primitive *p : primitives) delete p;
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
     * @brief Prepare the scene to deal with the simulation.
     * @param simFrequency_hz Simulation frequency the scene will work with.
     */
    virtual void prepareSimulation(int const simFrequency_hz) {}
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
    std::shared_ptr<RaySceneIntersection> getIntersection(
        glm::dvec3 const &rayOrigin,
        glm::dvec3 const &rayDir,
        bool const groundOnly
    ) const;
    /**
     * @brief Like Scene::getIntersection(dvec3 const, dvec3 const, bool const)
     *  but receiving precomputed minimum and maximum intersection times
     *  with respect to the axis aligned bounding box that bounds the scene.
     *
     * @param tMinMax The minimum and maximum intersection times
     * @see Scene::getIntersection(dvec3 const, dvec3 const, bool const)
     */
    std::shared_ptr<RaySceneIntersection> getIntersection(
        vector<double> const &tMinMax,
        glm::dvec3 const &rayOrigin,
        glm::dvec3 const &rayDir,
        bool const groundOnly
    ) const ;
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
    std::map<double, Primitive *> getIntersections(
        glm::dvec3 &rayOrigin,
        glm::dvec3 &rayDir,
        bool const groundOnly
    );

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

    /**
     * @brief For each scene part which is flagged as forceOnGround, it will be
     *  vertically translated to closest ground scene part.
     *
     * This function requires that there is at least one scene part flagged as
     *  ground, otherwise it is not possible to apply it. The algorithm can be
     *  described in 5 steps:
     *
     * 1. For any ground scene part, find its min
     *    \f$a_i = (a_{ix}, a_{iy}, a_{iz})\f$ and max
     *    \f$b_i = (b_{ix}, b_{iy}, b_{iz})\f$ vertices.
     *
     * 2. Let the scene part to be placed on ground be defined by its \f$m\f$
     *    vertices, such that \f$Q = \left\{q_1, \ldots, q_m\right\}\f$.
     *    Also, let it be represented by its minimum \f$z\f$ vertex
     *    \f$q_* = (q_{*x}, q_{*y}, q_{*z}) \f$. Thus, the first found ground
     *    scene part such that \f$q_*\f$ is inside its \f$\mathbb{R}^{2}\f$
     *    boundaries is considered as the ground reference. More formally, it
     *    must be satisfied that:
     *
     * \f[
     *    q_{*x} \in [a_{ix}, b_{ix}] \land
     *    q_{*y} \in [a_{iy}, b_{iy}]
     * \f]
     *
     * 3. Find the best fitting plane \f$\pi\f$ with respect to vertices of
     *    ground reference, which orthonormal vector is noted as \f$\hat{v}\f$
     *
     * \f[
     *    \pi = \left\{
     *        p=(x, y, z) :
     *            \left\langle{\overrightarrow{op}, \hat{v}}\right\rangle = 0
     *    \right\}
     * \f]
     *
     * 4. Compute the vertical projection of \f$q\f$ on best fitting ground
     *    plane \f$\pi\f$ as follows:
     *
     * \f[
     * \begin{array}{lll}
     *    \mathcal{P}^{z}_{\pi}(q) &=&
     *    \left(
     *        q_{x}, q_{y},
     *        \frac{
     *            \hat{v}_x o_x + \hat{v}_y o_y + \hat{v}_z o_z -
     *            \hat{v}_x q_{x} - \hat{v}_y q_{y}
     *        }
     *        {\hat{v}_z}
     *    \right) \\ &=&
     *    \left(
     *        q_{x}, q_{y},
     *        \frac{
     *            \left\langle{\hat{v}, \vec{o}}\right\rangle -
     *            \hat{v}_x q_{x} - \hat{v}_y q_{y}
     *          }{\hat{v}_z}
     *    \right) \\ &=&
     *    (p_x, p_y, p_z) \\ &=&
     *    p
     * \end{array}
     * \f]
     *
     *    Where \f$q\f$ varies depending on search depth, as explained in
     *    parameter documentation at the Scene::findForceOnGroundQ function.
     *
     * 5. Let \f$\Delta_z = q_{*z} - p_z\f$ be the magnitude of vertical
     *    translation for the entire scene part. Now, each vertex defining the
     *    scene part must be translated so:
     *
     * \f[
     *    \forall q_i \in Q,\, q'_i = (q_{ix}, q_{iy}, q_{iz} - \Delta_z)
     * \f]
     *
     * @see Scene::findForceOnGroundQ
     */
    void doForceOnGround();
    /**
     * @brief Assist doForceOnGround function to find an adequate \f$q\f$ for
     *    step 4
     * @param searchDepth The search strategy specification. This function should
     *    never be called with searchDepth 0, that case must be skipped at
     *    Scene::doForceOnGround directly. Search depth corresponds with the
     *    scene part's forceOnGround attribute
     *
     *    If it is 0 the force on ground process will be skipped.
     *    If it is 1, a specific algorithm based on \f$q=q_*\f$ as min \f$z\f$
     *    point will be used. If it is \f$>1\f$, then it specifies the number
     *    of search steps to be performed to find \f$q\f$ on a discrete linear
     *    search with searchDepth steps. If it is -1, then
     *    \f$
     *        q = \mathrm{argmin}_{q \in Q} \;\;\; q_z -
     *            \frac{\left\langle{\hat{v}, \vec{o}}\right\rangle - \left(
     *                \hat{v}_x q_x + \hat{v}_y q_y
     *            \right)}{\hat{v}_z}
     *    \f$
     * @param minzv The \f$q_*\f$ that is used as \f$q\f$ if searchDepth is 1
     * @param vertices All vertices defining the scene part to be placed on
     *    ground
     * @param o A point contained in the best fitting ground plane
     * @param v The normal vector of the best fitting ground plane
     * @see Scene::doForceOnGround
     */
    glm::dvec3 findForceOnGroundQ(
        int const searchDepth,
        glm::dvec3 const minzv,
        vector<Vertex *> &vertices,
        vector<double> const &o,
        vector<double> const &v
    );

    /**
     * @brief Build the KDGrove for the scene, overwriting previous one if any.
     * @param safe The same safe as the one received by finalizeSceneLoading
     * @see Scene::finalizeSceneLoading
     * @see Scene::buildKDGroveWithLog
     */
    void buildKDGrove(bool const safe=false);
    /**
     * @brief Call buildKDGrove exporting building information through logging
     *  system
     * @see Scene::buildKDGrove
     */
    void buildKDGroveWithLog(bool const safe=false);
    /**
     * @brief Shutdown the scene whe simulation has finished.
     */
    virtual void shutdown();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the axis aligned bounding box defining scene boundaries
     * @see Scene::bbox
     */
    std::shared_ptr<AABB> getAABB();

    virtual inline std::shared_ptr<KDGroveRaycaster> const &getRaycaster()
    const {return raycaster;}
    /**
     * @brief Obtain the KDGrove factory used by the scene
     * @return KDGrove factory used by the scene
     * @see Scene::kdgf
     */
    virtual inline std::shared_ptr<KDGroveFactory> getKDGroveFactory() const
    {return kdgf;}
    /**
     * @brief Set the KDGrove factory to be used by the scene
     * @param kdgf New KDGrove factory to be used by the scene
     * @see Scene::kdgf
     */
    virtual inline void setKDGroveFactory(
        std::shared_ptr<KDGroveFactory> const kdgf
    )
    {this->kdgf = kdgf;}
    /**
     * @brief Obtain the KDGrove used by the scene
     * @return KDGrove used by the scene
     * @see Scene::kdgrove
     */
    virtual inline std::shared_ptr<KDGrove> getKDGrove() const
    {return kdgrove;}
    /**
     * @brief Set the KDGrove to be used by the scene
     * @param kdgrove New KDGrove to be used by the scene
     * @see Scene::kdtree
     */
    virtual inline void setKDGrove(std::shared_ptr<KDGrove> const kdgrove)
    {this->kdgrove = kdgrove;}

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

    /**
     * @brief Checke whether the scene contains at least one moving object
     *  (true) or not (false)
     * @return True if scene hast at least one moving objects, false otherwise
     */
    virtual inline bool hasMovingObjects() const{
        for(std::shared_ptr<ScenePart> part : parts){
            if(part->getType()==ScenePart::ObjectType::DYN_MOVING_OBJECT)
            return true;
        }
        return false;
    }
    /**
     * @brief Get all the scene parts in the scene that need a swap between
     *  consecutive simulation replays.
     * @return Vector of scene parts with a handle for swap on repeat
     *  operations.
     * @see ScenePart::getSwapOnRepeatHandler
     * @see ScenePart::sorh
     */
    std::vector<std::shared_ptr<ScenePart>> getSwapOnRepeatObjects();
    /**
     * @brief Set the default reflectance that must be used for primitives
     *  that have no assigned reflectance.
     * @param defaultReflectance The new default reflectance.
     * @see Scene::defaultReflectance
     * @see Scene::getDefaultReflectance
     * @see SpectralLibrary
     * @see SpectralLibrary::defaultReflectance
     * @see XmlSurveyLoader::createSurveyFromXml
     */
    inline void setDefaultReflectance(double const defaultReflectance)
    {this->defaultReflectance = defaultReflectance;}
    /**
     * @brief Get the default reflectance that must be used for primitives that
     *  have not got an assigned reflectance.
     * @return The default reflectance for primitives that have not got an
     *  assigned reflectance.
     * @see Scene::setDefaultReflectance
     * @see SpectralLibrary
     * @see SpectralLibrary::defaultReflectance
     * @see XmlSurveyLoader::createSurveyFromXml
     */
    inline double getDefaultReflectance() const {return defaultReflectance;}



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

    // ***  SIMULATION STEP  *** //
    // ************************* //
    /**
     * @brief Support simulation step handling from scene side.
     *
     * For basic scenes and static scenes, there is nothing to do. However,
     *   dynamic scenes are expected to override this method to provide
     *   simulation level dynamism.
     *
     * @return True if scene was updated, false otherwise
     * @see DynScene::doSimStep
     */
    virtual bool doSimStep() {return false;}
};