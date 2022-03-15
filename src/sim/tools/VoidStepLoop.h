#pragma once

#include <StepLoop.h>

#include <functional>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending StepLoop to support functions with no return
 *
 * @see StepLoop
 */
template <typename ... StepInput>
class VoidStepLoop : public StepLoop<StepInput ...>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The function to be invoked when computation region function has
     *  been reached
     */
    std::function<void(StepInput ...)> f;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Void step loop constructor receiving interval and computation
     *  region function
     */
    VoidStepLoop(
        int const stepInterval,
        std::function<void(StepInput ...)> f
    ) :
        StepLoop<StepInput ...>(stepInterval),
        f(f)
    {}
    virtual ~VoidStepLoop() = default;

    // ***  LOOP METHODS  *** //
    // ********************** //
    /**
     * @brief Implementation of StepLoop::handleStep for computation regions
     *  with void return
     * @see StepLoop::handleStep
     */
    void handleStep(StepInput ... input) override {f(input ...);}
};