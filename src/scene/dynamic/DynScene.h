#pragma once

#include <vector>
#include <memory>

#include <scene/Scene.h>
#include <scene/dynamic/DynObject.h>

using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dynamic scene base implementation
 *
 * A dynamic scene extends the functionalities of a basic scene considering
 *  dynamic objects. Thus, the dynamic scene implements a doSimStep method
 *  as Platform or Scanner do. In consequence, using a dynamic scene means the
 *  scene will change as the simulation advances. This differs from basic
 *  scenes which are totally static.
 */
class DynScene : public Scene{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Dynamic objects composing the scene
     */
    vector<shared_ptr<DynObject>> dynObjs;
    /**
     * @brief Specify how many simulation steps must elapse between each
     *  simulation step computation for the dynamic scene
     */
    int dynamicSpaceFrequency = 1;
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
     * @see DynScene::dynamicSpaceFrequency
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
    DynScene(DynScene &ds);

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
     * @see DynScene::dynamicSpaceFrequency
     * @see DynScene::currentStep
     * @see DynScene::doStep
     */
    void doSimStep();
    /**
     * @brief Dynamic behavior computation itself.
     *
     * This function is invoked by doSimStep when necessary.
     *
     * @see DynScene::doSimStep
     */
    void doStep();

};