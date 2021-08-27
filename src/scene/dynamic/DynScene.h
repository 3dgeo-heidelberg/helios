#pragma once

#include <vector>
#include <memory>

#include <scene/Scene.h>
#include <scene/StaticScene.h>
#include <scene/dynamic/DynObject.h>

using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dynamic scene base implementation
 *
 * A dynamic scene extends the functionalities of a static scene considering
 *  dynamic objects. Thus, the dynamic scene implements a doSimStep method
 *  as Platform or Scanner do. In consequence, using a dynamic scene means the
 *  scene will change as the simulation advances. This differs from basic
 *  scenes which are totally static.
 *
 * @see StaticScene
 * @see Scene
 */
class DynScene : public StaticScene{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a DynScene to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the DynScene
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version){
        std::cout << "Exporting DynScene (0)..." << std::endl; // TODO Remove
        boost::serialization::void_cast_register<DynScene, StaticScene>();
        std::cout << "Exporting DynScene (1)..." << std::endl; // TODO Remove
        ar &boost::serialization::base_object<StaticScene>(*this);
        std::cout << "Exporting DynScene (2)..." << std::endl; // TODO Remove
        ar &dynObjs;
        std::cout << "Exporting DynScene (3)..." << std::endl; // TODO Remove
        ar &updated;
        std::cout << "Exporting DynScene (4)..." << std::endl; // TODO Remove
        ar &dynamicSpaceInterval;
        std::cout << "Exporting DynScene (5)..." << std::endl; // TODO Remove
        ar &currentStep;
        std::cout << "DynScene exported!" << std::endl; // TODO Remove
    }
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Dynamic objects composing the scene
     */
    vector<shared_ptr<DynObject>> dynObjs;
    /**
     * @brief Vector of flags controlling whether a dynamic object has been
     *  updated after last step (true) or not.
     *
     * If updated[i] is true it means the i-th dynamic object has been updated
     *  on last step, if it is false then it means the i-th dynamic object
     *  has NOT been updated on last step.
     */
    vector<bool> updated;
    /**
     * @brief Specify how many simulation steps must elapse between each
     *  simulation step computation for the dynamic scene
     */
    int dynamicSpaceInterval = 1;
    /**
     * @brief Stores the current simulation step.
     *
     * Let \f$\delta\f$ be the dynamic space frequency and \f$s_t\f$ the step
     *  at \f$t\f$ instant. Thus, the current step update behavior can be
     *  defined as follows:
     *
     * \f[
     *   s_{t+1} = \left(s_{t} + 1\right) \mod \delta
     * \f]
     * @see DynScene::dynamicSpaceInterval
     */
    int currentStep = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic scene default constructor
     */
    DynScene() = default;
    ~DynScene() override {}
    /**
     * @brief Copy constructor for dynamic scene
     * @param ds Dynamic scene to be copied
     */
    DynScene(DynScene &ds);
    /**
     * @brief Build a dynamic scene using given scene as basis
     * @param s Basis scene for dynamic scene
     */
    DynScene(Scene &s) : StaticScene(s) {}
    /**
     * @brief Build a dynamic scene using given static scene as basis
     * @param ss Basis static scene for dynamic scene
     */
    DynScene(StaticScene &ss) : StaticScene(ss) {}

    // ***  SIMULATION STEP  *** //
    // ************************* //
    /**
     * @brief Do corresponding computations for the dynamic scene at current
     *  simulation step if proceeds.
     *
     * Computations only occur for simulation steps which satisfy:
     * \f[
     *  s_{t} \equiv 0 \mod \delta
     * \f]
     *
     * Where \f$\delta\f$ is the dynamic space frequency and \f$s_{t}\f$ is the
     *  current step at instant \f$t\f$.
     * @return True if any dynamic object was udpated, false otherwise
     * @see DynScene::dynamicSpaceInterval
     * @see DynScene::currentStep
     * @see DynScene::doStep
     */
    bool doSimStep();
    /**
     * @brief Dynamic behavior computation itself.
     *
     * This function is invoked by doSimStep when necessary.
     *
     * @return True if any dynamic object was udpated, false otherwise
     * @see DynScene::doSimStep
     */
    bool doStep();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Append given dynamic object to the scene
     * @param dynobj Dynamic object to be appended to the scene
     * @see DynObject
     */
    inline void appendDynObject(shared_ptr<DynObject> dynobj) {
        dynObjs.push_back(dynobj);
        updated.push_back(true);
    }
    /**
     * @brief Obtain dynamic object at given index
     * @param index Index of dynamic object to be obtained
     * @return Dynamic object at given index
     */
    inline shared_ptr<DynObject> getDynObject(size_t const index)
    {return dynObjs[index];}
    /**
     * @brief Set dynamic object at given index
     * @param index Index of dynamic object to be setted
     * @param dynobj New dynamic object
     */
    inline void setDynObject(size_t const index, shared_ptr<DynObject> dynobj){
        dynObjs[index] = dynobj;
        updated[index] = true;
    }
    /**
     * @brief Remove dynamic object at given index
     * @param index Index of dynamic object to be removed
     */
    inline void removeDynObject(size_t const index){
        dynObjs.erase(dynObjs.begin()+index);
        updated.erase(updated.begin()+index);
    }
    /**
     * @brief Remove all dynamic objects from the dynamic scene
     */
    inline void clearDynObjects(){dynObjs.clear();}
    /**
     * @brief Obtain the number of dynamic objects in the scene
     * @return Number of dynamic objects in the scene
     */
    inline size_t numDynObjects() {return dynObjs.size();}
    /**
     * @brief Check whether the dynamic object at given index has been updated
     *  on last step (true) or not (false)
     * @param index Index of dynamic object to be checked
     * @return True if dynamic object at given index has been updated on last
     *  step, false otherwise
     */
    inline bool isDynObjectUpdated(size_t const index) const
    {return updated[index];}


    // ***   READ/WRITE  *** //
    // ********************* //
    /**
     * @brief Serialize the dynamic scene and write it to given output file
     * @param path Path to output file where serialized dynamic scene shall be
     *  stored
     */
    void writeObject(std::string path) override;
    /**
     * @brief Read serialized dynamic scene from given file
     * @param path Path to file where a serialized dynamic scene is stored
     * @return Imported dynamic scene
     */
    static DynScene *readObject(std::string path);
};