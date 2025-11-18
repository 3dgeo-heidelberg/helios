template<typename IT, typename ET>
typename fluxionum::IterativeMethodHandler<IT, ET>::IterativeMethodStatus
fluxionum::IterativeMethodHandler<IT, ET>::handleEndOfIteration(IT const& input,
                                                                ET const& error)
{
  // Handle patience based early stopping
  ET patienceDelta = error - patienceBestError; // Error evolution
  ++patienceCount;                              // Increase patience count
  if (currentIter == 0 || patienceDelta <= patienceEps) { // If improvement
    if (patiencePreserveBest)
      setPatienceBestInput(input);
    setPatienceBestError(error);
    patienceCount = 0;
  }
  if (patienceCount >= patience)
    return IterativeMethodStatus::PATIENCE_EARLY_STOPPING;

  // Handle max iterations
  if (maxIters > 0 && currentIter > maxIters)
    return IterativeMethodStatus::MAX_ITERATIONS;
  ++currentIter;

  // Handle convergence
  if (enableConvergenceCriterion && error <= convergenceEps)
    return IterativeMethodStatus::CONVERGENCE;

  // Return iterative method not finished
  return IterativeMethodStatus::CONTINUE;
}
