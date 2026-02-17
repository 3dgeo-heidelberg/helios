template<typename IT, typename OT>
IT
fluxionum::UnivariateNewtonRaphsonMinimizer<IT, OT>::argmin(IT x)
{
  // Prepare iterative method
  fluxionum::IterativeMethodHandler<IT, OT> imh(this->imh);
  bool iterate = true;
  std::function<OT(IT)> const& df = this->df[0];
  std::function<OT(IT)> const& d2f = this->df[1];

  // Compute iterative pmethod
  while (iterate) {
    // Compute the Newton-Rahpson minimization itself
    x = x - df(x) / d2f(x);

    // End of iteration handling
    switch (imh.handleEndOfIteration(x, x)) {
      case fluxionum::IterativeMethodHandler<IT, OT>::IterativeMethodStatus::
        PATIENCE_EARLY_STOPPING:
        if (imh.isPatiencePreservingBest())
          return imh.getPatienceBestInput();
      case fluxionum::IterativeMethodHandler<IT, OT>::IterativeMethodStatus::
        MAX_ITERATIONS:
      case fluxionum::IterativeMethodHandler<IT, OT>::IterativeMethodStatus::
        CONVERGENCE:
        iterate = false;
        break;
      case fluxionum::IterativeMethodHandler<IT, OT>::IterativeMethodStatus::
        CONTINUE:
        break;
    }
  }
  return x;
}
