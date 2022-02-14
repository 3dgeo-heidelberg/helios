#pragma once

#include <VoidStepLoop.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending VoidStepLoop to support a continous linear step loop
 *  without cyclic behavior.
 *
 * @see VoidStepLoop
 */
template <typename ... StepInput>
class LinearVoidStepLoop : public StepLoop<StepInput ...>{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Linear void step loop constructor receiving interval and
     *  computation region function
     */
    LinearVoidStepLoop(
        std::function<void(StepInput ...)> f
    ) :
        VoidStepLoop<StepInput ...>(0),
        f(f)
    {}
    virtual ~VoidStepLoop() = default;

    // ***  LOOP METHODS  *** //
    // ********************** //
    /**
     * @brief Advances to current loop iteration without cyclic behavior
     * @see StepLoop::nextStep
     */
    void nextStep() override{++currentStep;}
};