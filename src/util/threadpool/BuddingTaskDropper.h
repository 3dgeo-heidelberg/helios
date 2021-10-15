#pragma once

#include <util/threadpool/TaskDropper.h>

/**
 * @brief TODO Rethink : Comment if works
 * @tparam BudType
 * @tparam TaskType
 * @tparam ThreadPoolType
 * @tparam TaskArgs
 */
template <
    typename BudType,
    typename TaskType,
    typename ThreadPoolType,
    typename ... TaskArgs
>
class BuddingTaskDropper :
    public TaskDropper<TaskType, ThreadPoolType, TaskArgs...>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Magnitude of increase/decrease for max tasks
     *
     * \f[
     *  \Delta_1
     * \f]
     *
     * @see BuddingTaskDropper::delta2
     */
    int delta1;
    /**
     * @brief The initial value of \f$\Delta_1\f$
     */
    int initDelta1;
    /**
     * @brief Magnitude of increase/decrease for \f$\Delta_1\f$
     *
     * \f[
     *  \Delta_2
     * \f]
     *
     * @see BuddingTaskDropper:delta1
     */
    int delta2;
    /**
     * @brief Must be \f$0\f$ at initial instance but then it should be either
     *  \f$1\f$ or \f$-1\f$
     */
    char lastSign;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the budding task dropper
     * @param stepSize Magnitude of increase/decrease in max tasks
     * @param stepFactor Scale ratio for consecutive step sizes
     * @param lastSign Value of last sign. For the first instance of a chain of
     *  buds, its last sign will also be the first sign of the chain
     * @see TaskDropper::maxTasks
     * @see TaskDropper
     */
    BuddingTaskDropper(
        int const maxTasks=32,
        int const delta1=8,
        int const initDelta1=8,
        int const delta2=1,
        char const lastSign=0
    ):
        TaskDropper<TaskType, ThreadPoolType, TaskArgs...> (maxTasks),
        delta1(delta1),
        initDelta1(initDelta1),
        delta2(delta2),
        lastSign(lastSign)
    {}
    virtual ~BuddingTaskDropper() = default;

    // ***  BUDDING METHODS  *** //
    // ************************* //
    /**
     * @brief Do the budding reproduction of current task dropper
     * @tparam BudType Type of child
     * @param sign Sign defining the evolutionary sense of the reproduction
     *  operation
     * @return Bud child of current budding task dropper
     */
    virtual BudType reproduce(char const sign){
        // Null step size means bud will be a clone
        if(delta1 == 0){
            return BudType(
                this->maxTasks,
                delta1,
                initDelta1,
                delta2,
                lastSign
            );
        }

        // Configure bud
        int delta1Bud = delta1;
        if(lastSign!=0){
            if(lastSign==sign){
                delta1Bud += delta2;
            }
            else{
                delta1Bud = initDelta1;
            }
        }

        // Build and return bud
        return BudType(
            std::max<int>(1, this->maxTasks + sign * delta1Bud),
            delta1Bud,
            initDelta1,
            delta2,
            sign
        );
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the last sign of the budding task dropper
     * @return Last sign of budding task dropper
     * @see BuddingTaskDropper::lastSign
     */
    virtual inline char getLastSign() const {return lastSign;}
    /**
     * @brief Obtain the \f$\Delta_1\f$ of the budding task dropper
     * @return \f$\Delta_1\f$ of budding task dropper
     * @see BuddingTaskDropper::delta1
     */
    virtual inline int getDelta1() const {return delta1;}
    /**
     * @brief Obtain the init value of \f$Delta_1\f$ of the budding task
     *  dropper
     * @return Init value of \f$Delta_1\f$ of budding task dropper
     * @see BuddingTaskDropper::initDelta1
     */
    virtual inline int getInitDelta1() const {return initDelta1;}
    /**
     * @brief Obtain the \f$\Delta_2\f$ of the budding task dropper
     * @return \f$\Delta_2\f$ of budding task dropper
     * @see BuddingTaskDropper::delta2
     */
    virtual inline int getDelta2() const {return delta2;}
};