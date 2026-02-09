#pragma once

#include <StepLoop.h>

#include <functional>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending StepLoop to support functions with non void return
 *
 * @tparam StepOutput The output type of function to be invoked when
 *  computation region is reached
 * @see StepLoop
 */
template<typename StepOutput, typename... StepInput>
class NonVoidStepLoop : public StepLoop<StepInput...>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The function to be invoked when computation region has been
   *  reached
   */
  std::function<StepOutput(StepInput...)> f;
  /**
   * @brief The output obtained the last time that computation region was
   *  entered
   */
  StepOutput output;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Non void step loop constructor receiving interval and computation
   *  region function
   */
  NonVoidStepLoop(int const stepInterval,
                  std::function<StepOutput(StepInput...)> f)
    : StepLoop<StepInput...>(stepInterval)
    , f(f)
  {
  }
  virtual ~NonVoidStepLoop() = default;

  // ***  LOOP METHODS  *** //
  // ********************** //
  /**
   * @brief Implementation of StepLoop::handleStep for computation regions
   *  which return something
   * @see StepLoop::handleStep
   */
  void handleStep(StepInput... input) override { output = f(input...); }
  /**
   * @brief Obtain the output corresponding to the last time computation
   *  region was entered
   * @return The output that was obtained the last time that computation
   *  region was entered
   * @see NonVoidStepLoop::output
   */
  inline StepOutput retrieveOutput() const { return output; }
};
