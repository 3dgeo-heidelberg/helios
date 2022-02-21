#pragma once

#include <vector>
#include <memory>

#include <scene/Scene.h>
#include <scene/StaticScene.h>
#include <scene/dynamic/DynObject.h>
#include <sim/tools/NonVoidStepLoop.h>

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
        boost::serialization::void_cast_register<DynScene, StaticScene>();
        ar &boost::serialization::base_object<StaticScene>(*this);
        ar &dynObjs;
        ar &updated;
        //ar &stepLoop; // TODO Rethink : Handle through save/load construct
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
     * @brief The step loop for the dynamic scene
     * @see StepLoop
     */
    NonVoidStepLoop<bool> stepLoop;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic scene default constructor
     */
    DynScene(int const stepInterval=1) :
        stepLoop(stepInterval, [&] () -> bool{return doStep();})
    {}
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
    DynScene(Scene &s, int const stepInterval=1) :
        StaticScene(s),
        stepLoop(stepInterval, [&] () -> bool{return doStep();})
    {}
    /**
     * @brief Build a dynamic scene using given static scene as basis
     * @param ss Basis static scene for dynamic scene
     */
    DynScene(StaticScene &ss, int const stepInterval=1) :
        StaticScene(ss),
        stepLoop(stepInterval, [&] () -> bool{return doStep();})
    {}

    // ***  SIMULATION STEP  *** //
    // ************************* //
    /**
     * @brief Do corresponding computations for the dynamic scene at current
     *  simulation step, if any.
     *
     * Computations only occur for simulation steps which satisfy:
     * \f[
     *  s_{t} \equiv 0 \mod \Delta
     * \f]
     *
     * Where \f$\Delta\f$ is the step interval and \f$s_{t}\f$ is the
     *  current step at instant \f$t\f$.
     *
     * Notice handling of this loop is done through StepLoop class
     *
     * @return True if any dynamic object was udpated, false otherwise
     * @see DynScene::stepInterval
     * @see DynScene::currentStep
     * @see DynScene::doStep
     * @see Scene::doSimStep
     * @see StepLoop
     */
    bool doSimStep() override;
    /**
     * @brief Dynamic behavior computation itself.
     *
     * This function is invoked by doSimStep when necessary.
     *
     * @return True if any dynamic object was udpated, false otherwise
     * @see DynScene::doSimStep
     */
    bool doStep();
    /**
     * @brief Build the step loop with given step interval for the dynamic
     *  scene
     * @param stepInterval Step interval for the step loop to be built
     * @see StepLoop
     */
    virtual void makeStepLoop(int const stepInterval);

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
    /**
     * @brief Obtain the current step interval for the dynamic scene
     * @return Current step interval for the dynamic scene
     * @see DynScene::stepLoop
     */
    inline int getStepInterval() const
    {return stepLoop.getStepInterval();}
    /**
     * @brief Set the step interval for the dynamic scene
     * @param stepInterval The new step interval for the dynamic scene
     * @see DynScene::stepLoop
     */
    inline void setStepInterval(int const stepInterval)
    {stepLoop.setStepInterval(stepInterval);}




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