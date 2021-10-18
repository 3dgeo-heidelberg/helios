#pragma once

#include <util/threadpool/TaskDropper.h>

/**
 * @brief The budding task dropper is a task dropper which implements the
 *  logic for its own reproduction so buds (children) are modified depending
 *  on the difference between last idle time length and current idle time
 *  length.
 *
 * The responsibility of measuring idle time lengths lies outside this class
 *  scope. A budding task dropper implements the logic of how to work with idle
 *  times, but it does not provide a mechanism to do such measurements.
 *
 * To understand how budding task dropper works, let \f$\epsilon\f$ be the
 *  minimum magnitude of a measurement so any measurement \f$<\epsilon\f$ will
 *  be discarded. Let \f$\tau\f$ be the threshold defining the significance of
 *  a measurement and let \f$\Delta_1\f$ and \f$\Delta_2\f$ be the parameters
 *  defining the behavior of a budding task dropper. Notice the \f$\Delta_1\f$
 *  of the source budding task dropper (the first one, which has no father)
 *  will be noted as \f$\Delta_0\f$. Also, let \f$t_a\f$ be the last idle time
 *  length and \f$t_b\f$ be the current idle time length. Thus, if \f$B_a\f$
 *  is the parent budding task dropper and \f$B_b\f$ is the child budding task
 *  dropper, the last can be defined as:
 *
 * \f[
 * B_b \approx \left\{\begin{array}{lll}
 *   B_a &,&  t_b < \tau \\
 *   r\left(B_a, {\mathrm{sgn}(t_a-t_b)s}\right) &,&
 *      t_b \geq \tau \land \left\vert{t_a-t_b}\right\vert > \epsilon \\
 *   r\left(B_a, s\right) &,&
 *      t_b \geq \tau \land \left\vert{t_a-t_b}\right\vert \leq \epsilon \\
 * \end{array}\right.
 * \f]
 *
 * Where \f$B_b \approx B_a\f$ means \f$B_b\f$ is exactly equal to \f$B_a\f$
 *  except for the vector of tasks. On the other hand, let \f$m\f$ be the chunk
 *  size (max tasks) and \f$s\f$ be the sign of \f$B_a\f$ so the reproduce
 *  function \f$r\f$ can be defined as:
 *
 * \f[
 * r\left(B_a, s^{'}\right) = r\left(\left[\begin{array}{c}
 *  m \\ \Delta_1 \\ \Delta_2 \\ s
 * \end{array}\right], s^{'}\right) =
 * B_b = \left[\begin{array}{l}
 *  m^{'} \\ \Delta_1^{'} \\ \Delta_2^{'} \\ s^{'}
 * \end{array}\right]
 * = \left[\begin{array}{c}
 *  m + s^{'} \delta\left({\Delta_1}, s, s^{'}\right) \\
 *  \delta\left({\Delta_1}, s, s^{'}\right) \\
 *  \Delta_2, \\
 *  s{'}
 * \end{array}\right]
 * \f]
 *
 * Finally, \f$\delta(\Delta_1, s, s^{'})\f$ works as follows:
 * \f[
 * \delta\left({\Delta_1, s, s^{'}}\right) = \left\{\begin{array}{lll}
 *  \Delta_1 + \Delta_2 &,& s=s^{'} \\
 *  \Delta_0 &,& s \neq s^{'}
 * \end{array}\right.
 * \f]
 *
 * @tparam BudType Type of bud. Classes extending budding task dropper should
 *  specify themselves as the bud type.
 * @see TaskDropper
 * @see BuddingTaskDropper::evolve
 * @see BuddingTaskDropper::reproduce
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
    // ***  TASK DROPPER METHODS *** //
    // ***************************** //
    /**
     * @brief Do an empty clone of this budding task dropper. It is, an exact
     *  clone but with an empty tasks vector
     * @return Empty clone of this budding task dropper
     */
    inline BudType emptyClone() const {
        return BudType(
            this->maxTasks,
            delta1,
            initDelta1,
            delta2,
            lastSign
        );
    }

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

    /**
     * @brief Create the bud of current budding task dropper according to the
     *  defined evolutive criteria
     * @param lastIdle The last idle time length. It will be modified if a
     *  significant change occurs so the new last idle time is the one
     *  given as current idle time.
     * @param idle Current idle time length, it is the one which comes after
     *  last idle time length
     * @param idleTh Threshold defining used to define whether a measurement
     *  is significant or not
     * @param idleEps Minimum magnitude of a measurement so those below it
     *  will be discarded
     * @return Bud satisfying evolution criteria
     */
    virtual BudType evolve(
#ifdef BUDDING_METRICS
        bool &debugBuddingMetrics,
#endif
        long &lastIdle,
        long const idle,
        long const idleTh=100000,
        long const idleEps=100000
    ){
        if(idle < idleTh){ // No significant idle time
#ifdef BUDDING_METRICS
            debugBuddingMetrics=false;
#endif
            return emptyClone(); // No mutating budding
        }
#ifdef BUDDING_METRICS
        debugBuddingMetrics=true;
#endif

        // Significant idle time, determine evolving sense (sign)
        bool const sigChange = std::abs(idle-lastIdle) > idleEps;
        if(sigChange){ // If significant change
            char sign = (idle > lastIdle) ? -lastSign : lastSign;
            if(sign == 0) sign = 1;
            lastIdle = idle;
            return reproduce(sign); // Evolve through budding
        }
        else{ // If no significant change
            return reproduce(lastSign);
        }
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