#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class providing the basics to handle loops inside simulation
 *  time, which is discrete because it is based on simulation steps.
 *
 * Let \f$\Delta\f$ be the step interval and \f$s_t\f$ be the current step.
 * Considering that \f$s_t\f$ is calculated as explained in
 *  StepLoop::currentStep. Always that \f$s_t \equiv 0 \mod \Delta\f$, the
 *  computation region of the step loop will be reached.
 *
 * @tparam StepInput The input arguments of function to be invoked when
 *  computation region is reached
 * @see StepLoop::stepInterval
 * @see StepLoop::currentStep
 */
template <typename ... StepInput>
class StepLoop{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Specify how many simulation steps must elapse so the step loop
     *  enters its computation region.
     *
     * If the simulation frequency means there are \f$n\f$ iterations per
     *  second. Then, the step interval \f$\Delta\f$ means there are
     *  \f$\left\frac{n}{\Delta}\right\f$ steps per
     *  second. It must be satisfied that \f$\Delta \leq n\f$
     * @see StepLoop::currentStep
     */
    int stepInterval;
    /**
     * @brief Stores the current step.
     *
     * Let \f$Delta\f$ be the step interval and \f$s_t\f$ the step at
     *  \f$t\f$ instant. Thus, the current step update behavior can be defined
     *  as follows:
     *
     * \f[
     *  s_{t+1} = \left(s_{t} + 1\right) \mod \Delta
     * \f]
     *
     * @see StepLoop::stepInterval
     */
    int currentStep;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Step loop constructor receiving interval and computation region
     *  function
     */
    StepLoop(int const stepInterval) :
        stepInterval(stepInterval),
        currentStep(0)
    {}
    virtual ~StepLoop() = default;

    // ***  LOOP METHODS  *** //
    // ********************** //
    /**
     * @brief Handle current loop iteration and advances to next one by calling
     *  StepLoop::nextStep
     * @return True if the computation region was entered, false otherwise.
     */
    virtual bool doStep(StepInput ... input){
        bool computed = false;
        if(currentStep == 0){
            handleStep(input ...);
            computed = true;
        }
        nextStep();
        return computed;
    }

    /**
     * @brief Handle the current step itself, before proceeding to next one.
     *
     * This function must be overridden by any concrete implementation of
     *  StepLoop
     *
     * @param input Input for current step
     */
    virtual void handleStep(StepInput ... input) = 0;

    /**
     * @brief Advances to current loop iteration, restarting the loop when
     *  as many steps as step interval have been done
     */
    virtual void nextStep(){
        ++currentStep;
        if(currentStep == stepInterval) currentStep = 0;
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the step interval
     * @return The step interval
     * @see StepLoop::stepInterval
     */
    virtual int getStepInterval() const {return stepInterval;}
    /**
     * @brief Set the step interval
     * @param stepInterval New step interval
     * @see StepLoop::stepInterval
     */
    virtual void setStepInterval(int const stepInterval)
    {this->stepInterval = stepInterval;}
    /**
     * @brief Obtain the current step of the step loop
     * @return The current step of the step loop
     * @see StepLoop::currentStep
     */
    virtual int getCurrentStep() const {return currentStep;}
    /**
     * @brief Set the current step for the step loop
     * @param currentStep The new current step for the step loop
     * @see StepLoop::currentStep
     */
    virtual void setCurrentStep(int const currentStep)
    {this->currentStep = currentStep;}

};