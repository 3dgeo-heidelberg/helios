#ifndef _ITERATIVEMETHODHANDLER_H_
#define _ITERATIVEMETHODHANDLER_H_

#include <functional>

namespace fluxionum {

using std::function;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base implementation of an iterative method.
 *
 * It supports control by maximum number of iterations, a good enough
 *  convergence criterion and a patience based early stopping mechanism.
 *
 * @tparam IT Input type
 * @tparam ET Error type
 */
template<typename IT, typename ET>
class IterativeMethodHandler
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize the iterative method handler to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the iterative method handler
   */
  template<typename Archive>
  void serialize(Archive& ar, unsigned int const version)
  {
    ar & maxIters;
    ar & currentIter;
    ar & enableConvergenceCriterion;
    ar & convergenceEps;
    ar & patience;
    ar & patienceCount;
    ar & patienceEps;
    ar & patiencePreserveBest;
    ar & patienceBestError;
    ar & patienceBestInput;
  }

public:
  // ***  ENUMERATION  *** //
  // ********************* //
  /**
   * @brief Enumeration with different iterative method status
   * <ul>
   *  <li><b>PATIENCE_EARLY_STOPPING</b> :
   *      Iterative method must stop because of patience early stopping
   *      mechanism
   *  </li>
   *  <li><b>MAX_ITERATIONS</b> :
   *      Iterative method must stop because max iterations have been
   *      reached
   *  </li>
   *  <li><b>CONVERGENCE</b> :
   *      Iterative method must stop because convergence have been
   *      reached
   *  </li>
   *  <li><b>CONTINUE</b> : Iterative method must continue</li>
   * </ul>
   * @see fluxionum::IterativeMethodHandler::handleEndOfIteration
   */
  enum IterativeMethodStatus
  {
    PATIENCE_EARLY_STOPPING,
    MAX_ITERATIONS,
    CONVERGENCE,
    CONTINUE
  };

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Maximum number of iterations for the iterative method.
   *
   * While usage of this attribute may vary depending on the class which
   *  extends the implementation, the default behavior expects that \f$0\f$
   *  means no maximum number of iterations while \f$>0\f$ sets a concrete
   *  maximum number of iterations for the iterative method
   */
  size_t maxIters;
  /**
   * @brief The current iteration of the iterative method
   */
  size_t currentIter;
  /**
   * @brief True to enable the usage of convergence criterion
   */
  bool enableConvergenceCriterion;
  /**
   * @brief The convergence threshold \f$\epsilon\f$.
   *
   * The iterative method will stop, even if the maximum number of iterations
   *  has not been reached, when \f$\xi \leq \epsilon \f$ where \f$\xi\f$ is
   *  the convergence error.
   */
  ET convergenceEps;
  /**
   * @brief How many consecutive iterations with no improvement will be
   *  tolerated until aborting.
   *
   * While usage of this attribute may vary depending on the class which
   *  extends the implementation, the default behavior expects that \f$0\f$
   *  means no patience mechanism will be used while \f$>0\f$ sets a concrete
   *  maximum number of iterations without improvement for the patience based
   *  early stopping mechanism
   */
  size_t patience;
  /**
   * @brief Count of how many consecutive iterations without improvement have
   *  elapsed
   */
  size_t patienceCount;
  /**
   * @brief The tolerance criterion \f$\epsilon\f$ to determine whether there
   *  have been an improvement or not
   *
   * Let \f$\xi_*\f$ be the lowest (best) patience error and \f$\xi\f$ the
   *  error at current iteration. Thus, there is an improvement if and only
   *  if \f$\xi - \xi_* \leq \epsilon\f$.
   */
  ET patienceEps;
  /**
   * @brief True to enable preserving best known input when early stopping
   *  because of patience, false to disable it
   */
  bool patiencePreserveBest;
  /**
   * @brief Error of best known case for patience based early stopping
   *  mechanism
   */
  ET patienceBestError;
  /**
   * @brief Input of best known case for patience based early stopping
   *  mechanism
   */
  IT patienceBestInput;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Iterative method constructor
   * @see fluxionum::IterativeMethodHandler::maxIters
   * @see fluxionum::IterativeMethodHandler::patience
   * @see fluxionum::IterativeMethodHandler::patienceEps
   * @see fluxionum::IterativeMethodHandler::patiencePreserveBest
   */
  IterativeMethodHandler(size_t maxIters,
                         bool enableConvergenceCriterion = false,
                         ET convergenceEps = 0.000000001,
                         size_t patience = 0,
                         ET patienceEps = 0.000000001,
                         bool patiencePreserveBest = true)
    : maxIters(maxIters)
    , currentIter(0)
    , enableConvergenceCriterion(enableConvergenceCriterion)
    , convergenceEps(convergenceEps)
    , patience(patience)
    , patienceCount(0)
    , patienceEps(patienceEps)
    , patiencePreserveBest(patiencePreserveBest)
  {
  }
  virtual ~IterativeMethodHandler() = default;

  // ***  ITERATIVE METHOD  *** //
  // ************************** //
  /**
   * @brief Handle end of iteration
   * @param error Error at given iteration
   * @return True if the iterative method is considered to have finished,
   *  thus no more iterations are needed. False otherwise.
   * @see IterativeMethodHandler::IterativeMethodStatus
   */
  virtual IterativeMethodStatus handleEndOfIteration(IT const& input,
                                                     ET const& error);

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the maximum number of iterations
   * @return Maximum number of iterations
   * @see fluxionum::IterativeMethodHandler::maxIters
   */
  virtual inline size_t getMaxIters() const { return maxIters; }
  /**
   * @brief Set the maximum number of iterations
   * @param maxIters New maximum number of iterations
   * @see fluxionum::IterativeMethodHandler::maxIters
   */
  virtual inline void setMaxIters(size_t const maxIters)
  {
    this->maxIters = maxIters;
  }
  /**
   * @brief Obtain the number of current iteration
   * @return Number of current iteration
   * @see fluxionum::IterativeMethodHandler::currentIter
   */
  virtual inline size_t getCurrentIter() const { return currentIter; }
  /**
   * @brief Set the number of current iteration
   * @param iter New number of current iteration
   * @see fluxionum::IterativeMethodHandler::currentIter
   */
  virtual inline void setCurrentIter(size_t const iter) { currentIter = iter; }

  /**
   * @brief Obtain the patience of the iterative method
   * @return Patience of the iterative method
   * @see fluxionum::IterativeMethodHandler::patience
   */
  virtual inline size_t getPatience() const { return patience; }
  /**
   * @brief Set the patience of the iterative method
   * @param patience New patience of the iterative method
   * @see fluxionum::IterativeMethodHandler::patience
   */
  virtual inline void setPatience(size_t const patience)
  {
    this->patience = patience;
  }
  /**
   * @brief Obtain number of consecutive iterations without improvement
   * @return Number of consecutive iterations without improvement
   * @see fluxionum::IterativeMethodHandler::patienceCount
   */
  virtual inline size_t getPatienceCount() const { return patienceCount; }
  /**
   * @brief Set the number of consecutive iterations without improvement
   * @param count New number of consecutive iterations without improvement
   * @see fluxionum::IterativeMethodHandler::patienceCount
   */
  virtual inline void setPatienceCount(size_t const count)
  {
    patienceCount = count;
  }
  /**
   * @brief Obtain the tolerance criterion for patience based early stopping
   * @return Tolerance criterion for patience based early stopping
   * @see fluxionum::IterativeMethodHandler::patienceEps
   */
  virtual inline ET getPatienceEps() const { return patienceEps; }
  /**
   * @brief Set the tolerance criterion for patience based early stopping
   * @param eps New tolerance criterion for patience based early stopping
   * @see fluxionum::IterativeMethodHandler::patienceEps
   */
  virtual inline void setPatienceEps(ET const eps) { patienceEps = eps; }
  /**
   * @brief Check whether the patience based early stopping mechanism is
   *  preserving best (true) or not (false)
   * @return True if patience based early stopping mechanism is preserving
   *  best, false otherwise
   * @see fluxionum::IterativeMethodHandler::patiencePreserveBest
   */
  virtual inline bool isPatiencePreservingBest() const
  {
    return patiencePreserveBest;
  }
  /**
   * @brief Set the patience based early stopping mechanism preserve best
   *  mode
   * @param preserveBest True to enabled preserve best mode, false to disable
   *  it
   * @see fluxionum::IterativeMethodHandler::patiencePreserveBest
   */
  virtual inline void setPatiencePreserveBest(bool const preserveBest)
  {
    patiencePreserveBest = preserveBest;
  }
  /**
   * @brief Get the patience based early stopping mechanism best known error
   * @return Patience based early stopping mechanism best known error
   * @see fluxionum::IterativeMethodHandler::patienceBestError
   */
  virtual inline ET getPatienceBestError() const { return patienceBestError; }
  /**
   * @brief Set the patience based early stopping mechanism best known error
   * @param bestError New best known error
   * @see fluxionum::IterativeMethodHandler::patienceBestError
   */
  virtual inline void setPatienceBestError(ET bestError)
  {
    patienceBestError = bestError;
  }
  /**
   * @brief Obtain the input of best known case for patience based early
   *  stopping mechanism
   * @return Input of best known case for patience based early stopping
   *  mechanism
   * @see fluxionum::IterativeMethodHandler::patienceBestInput
   */
  virtual inline IT getPatienceBestInput() const { return patienceBestInput; }
  /**
   * @brief Set the input of best known case for patience based early
   *  stopping mechanism
   * @param bestInput New best input for patience based early stopping
   *  mechanism
   * @see fluxionum::IterativeMethodHandler::patienceBestInput
   */
  virtual inline void setPatienceBestInput(IT bestInput)
  {
    patienceBestInput = bestInput;
  }
};

}
#endif

#include <IterativeMethodHandler.tpp>
