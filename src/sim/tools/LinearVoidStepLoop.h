#pragma once

#include <VoidStepLoop.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending VoidStepLoop to support a continuous linear step loop
 *  without cyclic behavior.
 *
 * @see VoidStepLoop
 */
template<typename... StepInput>
class LinearVoidStepLoop : public VoidStepLoop<StepInput...>
{
  using StepLoop<StepInput...>::currentStep;
  using StepLoop<StepInput...>::handleStep;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Linear void step loop constructor receiving interval and
   *  computation region function
   */
  LinearVoidStepLoop(std::function<void(StepInput...)> f)
    : VoidStepLoop<StepInput...>(0, f)
  {
  }
  virtual ~LinearVoidStepLoop() = default;

  // ***  LOOP METHODS  *** //
  // ********************** //
  /**
   * @brief Handle current loop iteration and advances to next one without
   *  cyclic behavior
   * @return True always, because it always compute
   * @see StepLoop::doStep
   */
  bool doStep(StepInput... input) override
  {
    handleStep(input...);
    nextStep();
    return true;
  }
  /**
   * @brief Advances to current loop iteration without cyclic behavior
   * @see StepLoop::nextStep
   */
  void nextStep() override { ++currentStep; }
};
