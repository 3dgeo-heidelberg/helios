#pragma once

#include <memory>
#include <vector>

#include <helios/scene/Scene.h>
#include <helios/scene/StaticScene.h>
#include <helios/scene/dynamic/DynObject.h>
#include <helios/sim/tools/NonVoidStepLoop.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dynamic scene base implementation.
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
class DynScene : public StaticScene
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a DynScene to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the DynScene
   * @see DynScene::save(Archive &, const unsigned int)
   * @see DynScene::load(Archive &, const unsigned int)
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::split_member(ar, *this, version);
  }
  /**
   * @brief Save a serialized DynScene to a stream of bytes
   * @see DynScene::serialize(Archive &, const unsigned int)
   * @see DynScene::load(Archive &, const unsigned int)
   */
  template<class Archive>
  void save(Archive& ar, const unsigned int version) const
  {
    boost::serialization::void_cast_register<DynScene, StaticScene>();
    ar& boost::serialization::base_object<StaticScene>(*this);
    ar & dynObjs;
    ar & updated;
    ar & stepLoop.getStepInterval();
  }
  /**
   * @brief Load a serialized DynScene from a stream of bytes
   * @see DynScene::serialize(Archive &, const unsigned int)
   * @see DynScene::save(Archive &, const unsigned int)
   */
  template<class Archive>
  void load(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<DynScene, StaticScene>();
    ar& boost::serialization::base_object<StaticScene>(*this);
    ar & dynObjs;
    ar & updated;
    int stepInterval;
    ar & stepInterval;
    stepLoop.setStepInterval(stepInterval);
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Dynamic objects composing the scene
   */
  std::vector<std::shared_ptr<DynObject>> dynObjs;
  /**
   * @brief Vector of flags controlling whether a dynamic object has been
   *  updated after last step (true) or not.
   *
   * If updated[i] is true it means the i-th dynamic object has been updated
   *  on last step, if it is false then it means the i-th dynamic object
   *  has NOT been updated on last step.
   */
  std::vector<bool> updated;
  /**
   * @brief The step loop for the dynamic scene
   * @see StepLoop
   */
  NonVoidStepLoop<bool> stepLoop;

  /**
   * @brief The dynamic time step. It will be NaN when not given.
   * @see DynObject::dynTimeStep
   * @see DynMovingObject::observerDynTimeStep
   * @see DynScene::prepareSimulation
   */
  double dynTimeStep = std::numeric_limits<double>::quiet_NaN();

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Dynamic scene default constructor
   */
  DynScene(int const stepInterval = 1)
    : stepLoop(stepInterval, [&]() -> bool { return doStep(); })
    , dynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
  ~DynScene() override {}
  /**
   * @brief Copy constructor for dynamic scene
   * @param ds Dynamic scene to be copied
   */
  DynScene(DynScene& ds);
  /**
   * @brief Build a dynamic scene using given scene as basis
   * @param s Basis scene for dynamic scene
   */
  DynScene(Scene& s, int const stepInterval = 1)
    : StaticScene(s)
    , stepLoop(stepInterval, [&]() -> bool { return doStep(); })
  {
  }
  /**
   * @brief Build a dynamic scene using given static scene as basis
   * @param ss Basis static scene for dynamic scene
   */
  DynScene(StaticScene& ss, int const stepInterval = 1)
    : StaticScene(ss)
    , stepLoop(stepInterval, [&]() -> bool { return doStep(); })
  {
  }

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Prepare the dynamic scene to deal with the simulation.
   *
   * A dynamic scene must check the coherence of the many dynamic loops at
   * potentially different frequencies, i.e., the step of the dynamic scene,
   * the step of each scene part, and the step for KDT updates.
   *
   * Let \f$F_{\mathrm{sim}}\f$ be the simulation frequency,
   *  \f$\Delta F_s\f$ the discrete step of the dynamic scene (i.e., the
   *  dynStep parameter), \f$\Delta F_p\f$ the discrete step of a given scene
   *  part (e.g., dynamic moving object), \f$\Delta F_k\f$ the discrete step
   *  of a given KDTree, \f$\Delta t_s\f$ the continuous time step of the
   *  dynamic scene, \f$\Delta t_p\f$ the continuous time step of a given
   *  scene part, and \f$\Delta t_k\f$ the continuous time step of a given
   *  KDTree.
   *
   * First, the scene dynamic step divides the simulation frequency to obtain
   *  the iterations for scene updates, that must be less than or equal to
   *  the iterations per second of the simulation loop. Thus, both the
   *  discrete (number of iterations) and continuous (time) steps for the
   *  dynamic scene are given by the following expressions:
   *
   * \f[
   *  \Delta t_s = \frac{\Delta F_s}{F_{\mathrm{sim}}} \iff
   *  \Delta F_s = F_{\mathrm{sim}} \Delta t_s
   * \f]
   *
   * Second, the dynamic step of a moving scene part scales the dynamic step
   * of the scene. Consequently, the discrete and continuous steps for a
   * moving scene part are given by the following expressions:
   *
   * \f[
   *  \Delta t_p = \frac{\Delta F_s \Delta F_p}{F_{\mathrm{sim}}} \iff
   *  \Delta F_p = \frac{F_{\mathrm{sim}} \Delta t_p}{\Delta F_s}
   *  = \frac{F_{\mathrm{sim}} \Delta t_p}{F_{\mathrm{sim}} \Delta t_s}
   *  = \frac{\Delta t_p}{\Delta t_s}
   * \f]
   *
   * Finally, the dynamic step of a dynamic KDT scales the dynamic step of
   * its associated moving scene part. Consequently, the discrete and
   * continuous steps for a dynamic KDT are given by the following
   * expressions:
   *
   * \f[
   *  \Delta t_k = \frac{\Delta F_k \Delta F_s \Delta F_p}{F_{\mathrm{sim}}}
   *  \iff
   *  \Delta F_k =\frac{F_{\mathrm{sim}}\Delta t_k}{\Delta F_s \Delta F_p}
   *  = \frac{F_{\mathrm{sim}} \Delta t_k}{F_{\mathrm{sim}} \Delta t_s
   *      \frac{\Delta t_p}{\Delta t_s}
   *  }
   *  = \frac{\Delta t_k}{\Delta t_p}
   * \f]
   *
   * Note the continuous time steps are expected to be inside \f$[0, 1]\f$.
   * Negative time and $t>1$ are not supported.
   *
   * @param simFrequency_hz Simulation frequency the scene will work with.
   * @see Simulation::prepareSimulation
   */
  void prepareSimulation(int const simFrequency_hz) override;

  /**
   * @see Scene::shutdown
   */
  void shutdown() override;

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
  inline void appendDynObject(std::shared_ptr<DynObject> dynobj)
  {
    dynObjs.push_back(dynobj);
    updated.push_back(true);
  }
  /**
   * @brief Obtain dynamic object at given index
   * @param index Index of dynamic object to be obtained
   * @return Dynamic object at given index
   */
  inline std::shared_ptr<DynObject> getDynObject(std::size_t const index)
  {
    return dynObjs[index];
  }
  /**
   * @brief Set dynamic object at given index
   * @param index Index of dynamic object to be setted
   * @param dynobj New dynamic object
   */
  inline void setDynObject(std::size_t const index,
                           std::shared_ptr<DynObject> dynobj)
  {
    dynObjs[index] = dynobj;
    updated[index] = true;
  }
  /**
   * @brief Remove dynamic object at given index
   * @param index Index of dynamic object to be removed
   */
  inline void removeDynObject(std::size_t const index)
  {
    dynObjs.erase(dynObjs.begin() + index);
    updated.erase(updated.begin() + index);
  }
  /**
   * @brief Remove all dynamic objects from the dynamic scene
   */
  inline void clearDynObjects() { dynObjs.clear(); }
  /**
   * @brief Obtain the number of dynamic objects in the scene
   * @return Number of dynamic objects in the scene
   */
  inline std::size_t numDynObjects() { return dynObjs.size(); }
  /**
   * @brief Check whether the dynamic object at given index has been updated
   *  on last step (true) or not (false)
   * @param index Index of dynamic object to be checked
   * @return True if dynamic object at given index has been updated on last
   *  step, false otherwise
   */
  inline bool isDynObjectUpdated(std::size_t const index) const
  {
    return updated[index];
  }
  /**
   * @brief Obtain the current step interval for the dynamic scene
   * @return Current step interval for the dynamic scene
   * @see DynScene::stepLoop
   */
  inline int getStepInterval() const { return stepLoop.getStepInterval(); }
  /**
   * @brief Set the step interval for the dynamic scene
   * @param stepInterval The new step interval for the dynamic scene
   * @see DynScene::stepLoop
   */
  inline void setStepInterval(int const stepInterval)
  {
    stepLoop.setStepInterval(stepInterval);
  }
  /**
   * @brief Get the dynamic time step of the dynamic scene. Note it is not
   *  taken from the step loop, instead it represents a user-given parameter
   *  that must be used to configure the stepLoop.
   * @return The dynamic time step of the dynamic scene.
   * @see DynScene::dynTimeStep
   * @see DynScene::setDynTimeStep
   * @see DynScene::stepLoop
   * @see DynScene::prepareSimulation
   */
  inline double getDynTimeStep() const { return dynTimeStep; }
  /**
   * @brief Set the dynamic time step of the dynamic scene. Note it does not
   *  modify the dynamic scene's step loop. The dynTimeStep attribute simply
   *  represents a user-given parameter. Setting it will not automatically
   *  update the stepLoop.
   * @param dynTimeStep The new dynamic time step for the dynamic scene.
   * @see DynScene::dynTimeStep
   * @see DynScene::getDynTimeStep
   * @see DynScene::stepLoop
   * @see DynScene::prepareSimulation
   */
  inline void setDynTimeStep(double const dynTimeStep)
  {
    this->dynTimeStep = dynTimeStep;
  }

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
  static DynScene* readObject(std::string path);
};
