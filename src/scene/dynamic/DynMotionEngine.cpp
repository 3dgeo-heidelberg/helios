#include <DynMotionEngine.h>

// ***  DYNAMIC MOTION ENGINE METHODS  *** //
// *************************************** //
arma::mat
DynMotionEngine::apply(DynMotion const& f,
                       arma::mat const& X,
                       DynObject& dynObj)
{
  // Handle self mode dynamic motion for non normals
  if (f.isSelfMode() && !f.isNormalMode()) {
    rigidmotion::RigidMotion g = rm3f.makeTranslation(-dynObj.getCentroid());
    g = rme.compose(f, g);
    g = rme.compose(rm3f.makeTranslation(dynObj.getCentroid()), g);
    dynObj.setCentroid(dynObj.getCentroid() + f.getC());
    return rme.apply(g, X);
  }

  // Handle plain dynamic motion (base rigid motion)
  if (!f.isNormalMode()) { // Position update
    dynObj.setCentroid(rme.apply(f, dynObj.getCentroid()));
    return rme.apply(f, X);
  }
  // Normal update
  return rme.apply(
    rigidmotion::RigidMotion(arma::zeros(f.getDimensionality()), f.getA()), X);
}

DynMotion
DynMotionEngine::compose(DynMotion const& f,
                         DynMotion const& g,
                         DynObject const& dynObj)
{
  // Compose the underlying rigid motion
  DynMotion dm = _compose(f, g, dynObj);

  // Configure dynamic motion
  if (f.isNormalMode() || g.isNormalMode()) // Normal mode transitivity
    dm.setNormalMode(true);

  // Return
  return dm;
}

DynMotion
DynMotionEngine::_compose(DynMotion const& f,
                          DynMotion const& g,
                          DynObject const& dynObj)
{
  if (f.isSelfMode()) {
    arma::colvec const go = rme.apply(g, dynObj.getCentroid()); // g(O)
    if (g.isSelfMode()) {

      // Composition: f' o g'
      rigidmotion::RigidMotion h =
        rm3f.makeTranslation(-dynObj.getCentroid()); // -O
      h = rme.compose(g, h);                         // g(X-O)
      h = rme.compose(                               // O + g(X-O) - g(O)
        rme.compose(rm3f.makeTranslation(-go),
                    rm3f.makeTranslation(dynObj.getCentroid())),
        h);
      h = rme.compose(f, h); // f[O+g(X-O)-g(O)]
      // Return: g(O)+f[O+g(X-O)-g(O)]
      return { rme.compose(rm3f.makeTranslation(go), h) };
    }

    // Composition: f' o g
    rigidmotion::RigidMotion h =
      rme.compose(rm3f.makeTranslation(-go), g); // g(X)-g(O)
    h = rme.compose(f, h);                       // f[g(X)-g(O)]
    // Return: g(O) + f[g(X)-g(O)]
    return { rme.compose(rm3f.makeTranslation(go), h) };

  } else if (g.isSelfMode()) {

    // Composition: f o g'
    rigidmotion::RigidMotion h = rme.compose( // g(X-O)
      g,
      rm3f.makeTranslation(-dynObj.getCentroid()));
    h = rme.compose( // O + g(X-O)
      rm3f.makeTranslation(dynObj.getCentroid()),
      h);
    // Return: f[O + g(X-O)]
    return { rme.compose(f, h) };
  }

  // Composition: f o g
  return { rme.compose(f, g) };
}
