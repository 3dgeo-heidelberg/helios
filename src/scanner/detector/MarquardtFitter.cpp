#include <helios/scanner/detector/MarquardtFitter.h>

#include <helios/util/logger/logging.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace std;

double
MarquardtFitter::evaluate(const double x, const vector<double>& params)
{
  double const A = params[0];
  double const B = params[1];
  double const C = params[2];
  double const D = params[3];
  double const u = (x - C) / D;
  return A + B * std::exp(-u * u);
}

/*
 *      Sets the values of the original data points that are going to be fit.
 */
void
MarquardtFitter::setData(const vector<double>& zvalues)
{
  X.resize(zvalues.size());
  for (size_t i = 0; i < zvalues.size(); i++) {
    X[i] = i;
  }

  Z = zvalues;
}

void
MarquardtFitter::setParameters(const vector<double>& parameters)
{
  A = parameters;
}
/*
 *     returns the cumulative error for current values
 */
double
MarquardtFitter::calculateErrors()
{
  double new_error = 0;
  for (size_t i = 0; i < Z.size(); i++) {
    double const v = evaluate(X[i], A);
    ERR[i] = Z[i] - v;
    new_error += ERR[i] * ERR[i];
  }
  return new_error;
}

double
MarquardtFitter::calculateDerivative(int const k,
                                     const double x,
                                     vector<double>& params)
{
  // Hybrid derivative
  if (k == 0) { // Use analytical derivative as it is faster for k[0], a
    return 1;
  } else if (k == 1) { // Use analytical derivative as it is faster for k[1], b
    double const u = (x - params[2]) / params[3];
    return std::exp(-u * u);
  }

  // Use numerical derivative as it is faster for other cases
  params[k] -= DELTA;
  double const b = evaluate(x, params);
  params[k] += 2 * DELTA;
  double const a = evaluate(x, params);
  params[k] -= DELTA;
  return (a - b) / (2 * DELTA);
}

void
MarquardtFitter::calculateDerivativeFast(double const x,
                                         double const c,
                                         double const d,
                                         double const coefficient,
                                         std::vector<double>& dvec)
{
  double const x_c = (x - c);
  double const u = (x - c) / d;
  double const db = std::exp(-u * u);
  double const dc = db * coefficient * x_c;

  dvec[0] = 1.0;
  dvec[1] = db;
  dvec[2] = dc;
  dvec[3] = dc * x_c / d;
}

/*
 *  Creates an array of derivatives since each one is used 3x's
 */
void
MarquardtFitter::calculateDerivatives()
{
  vector<double> working = A;
  for (size_t j = 0; j < A.size(); j++) {
    for (size_t i = 0; i < Z.size(); i++) {
      DERIVATIVES[i][j] = calculateDerivative(j, X[i], working);
    }
  }
}

void
MarquardtFitter::calculateDerivativesFast()
{
  // Precompute params
  double const c = A[2];
  double const d = A[3];
  double const coefficient = 2 * A[1] / (d * d);

  // Compute derivative using precomputed params
  for (size_t i = 0; i < Z.size(); i++) {
    calculateDerivativeFast(X[i], c, d, coefficient, DERIVATIVES[i]);
  }
}

void
MarquardtFitter::createBetaMatrix()
{
  BETA = vector<double>(A.size(), 0.0);
  for (size_t k = 0; k < BETA.size(); k++) {
    for (size_t i = 0; i < X.size(); i++) {
      BETA[k] += ERR[i] * DERIVATIVES[i][k];
    }
  }
}

void
MarquardtFitter::createAlphaPrimeMatrix()
{
  size_t n = A.size();
  alphaPrimeN = n;
  if (ALPHA_PRIME.size() < n) {
    ALPHA_PRIME.resize(n, std::vector<double>(n));
  }

  double v;
  for (size_t k = 0; k < n; k++) {
    for (size_t l = 0; l < n; l++) {
      v = 0.0;
      for (size_t i = 0; i < X.size(); i++) {
        v += DERIVATIVES[i][k] * DERIVATIVES[i][l];
      }
      if (k == l) {
        v *= (1 + LAMBDA[k]);
      }
      ALPHA_PRIME[l][k] = v;
    }
  }
}

/*
 *  Takes the current error, and the current parameter set and calculates the
 *  changes, then returns the maximum changed value
 */
void
MarquardtFitter::iterateValues()
{

  // calculateDerivatives(); // Uses hybrid derivative
  calculateDerivativesFast(); // Uses analyitical derivative with cache

  createBetaMatrix();
  createAlphaPrimeMatrix();

  int m = BETA.size();
  int n = 1;
  vector<vector<double>> BETA_MAT =
    vector<vector<double>>(m, std::vector<double>(n, 0.0));
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      BETA_MAT[i][j] = this->BETA[i + j * m];
    }
  }

  std::vector<std::vector<double>>& LU = this->ALPHA_PRIME;
  m = alphaPrimeN;
  n = alphaPrimeN;

  vector<int> piv(m);
  for (int i = 0; i < m; i++) {
    piv[i] = i;
  }
  int pivsign = 1;
  vector<double> LUcolj(m, 0.0);

  // Outer loop.

  for (int j = 0; j < n; j++) {

    // Make a copy of the j-th column to localize references.

    for (int i = 0; i < m; i++) {
      LUcolj[i] = LU[i][j];
    }

    // Apply previous transformations.

    for (int i = 0; i < m; i++) {

      vector<double>& LUrowi = LU[i];

      // Most of the time is spent in the following dot product.

      int kmax = ::min(i, j);
      double s = 0.0;
      for (int k = 0; k < kmax; k++) {
        s += LUrowi[k] * LUcolj[k];
      }
      LUrowi[j] = LUcolj[i] -= s;
    }

    // Find pivot and exchange if necessary.

    int p = j;
    for (int i = j + 1; i < m; i++) {
      if (::fabs(LUcolj[i]) > ::fabs(LUcolj[p])) {
        p = i;
      }
    }
    if (p != j) {
      for (int k = 0; k < n; k++) {
        double t = LU[p][k];
        LU[p][k] = LU[j][k];
        LU[j][k] = t;
      }
      int k = piv[p];
      piv[p] = piv[j];
      piv[j] = k;
      pivsign = -pivsign;
    }

    // Compute multipliers.
    if (j < m && LU[j][j] != 0.0) {
      for (int i = j + 1; i < m; i++) {
        LU[i][j] /= LU[j][j];
      }
    }
  }

  n = alphaPrimeN;

  int nx = BETA_MAT[0].size();
  vector<vector<double>> rhs = getMatrix(BETA_MAT, piv, 0, nx - 1);

  // Solve L*Y = B(piv,:)
  for (int k = 0; k < n; k++) {
    for (int i = k + 1; i < n; i++) {
      for (int j = 0; j < nx; j++) {
        rhs[i][j] -= rhs[k][j] * LU[i][k];
      }
    }
  }
  // Solve U*X = Y;
  for (int k = n - 1; k >= 0; k--) {
    for (int j = 0; j < nx; j++) {
      if (LU[k][k] == 0.0) {
        throw std::overflow_error("Divide by zero exception");
      }
      rhs[k][j] /= LU[k][k];
    }
    for (int i = 0; i < k; i++) {
      for (int j = 0; j < nx; j++) {
        rhs[i][j] -= rhs[k][j] * LU[i][k];
      }
    }
  }

  for (size_t i = 0; i < A.size(); i++) {
    this->A[i] += rhs[i][0];
  }
}

/* Get a submatrix.
@param r    Array of row indices.
@param j0   Initial column index
@param j1   Final column index
@return     A(r(:),j0:j1)
*/
vector<vector<double>>
MarquardtFitter::getMatrix(vector<vector<double>>& M,
                           vector<int>& row_i,
                           int j0,
                           int j1)
{
  int m = row_i.size();
  int n = j1 - j0 + 1;
  vector<vector<double>> SUB;
  SUB.resize(m, std::vector<double>(n, 0.0));

  for (int i = 0; i < m; i++) {
    for (int j = j0; j <= j1; j++) {
      SUB[i][j - j0] = M[row_i[i]][j];
    }
  }
  return SUB;
}

void
MarquardtFitter::initializeWorkspace()
{
  ERR = vector<double>(Z.size());
  DERIVATIVES =
    vector<vector<double>>(Z.size(), std::vector<double>(A.size(), 0.0));
  LAMBDA = vector<double>(A.size(), 0.01);
}

/*
 *  Main routine, call this and the Parameters are iterated until it is
 * finished.
 */
void
MarquardtFitter::fitData()
{
  initializeWorkspace();
  double error = calculateErrors();
  double nerror, value;

  vector<double> acopy(A.size());

  for (int i = 0; i < 10000; i++) {

    try {
      acopy = this->A;
      iterateValues();
    } catch (...) {
      ostringstream s;
      s << "Broke after " << i << " iterations" /*<< printMatrix()*/;
      logging::INFO(s.str());
      return;
    }

    nerror = calculateErrors();

    value = error - nerror;

    if (value < 0) {
      // reject changes
      this->A = acopy;
      updateLambda(value);
      nerror = calculateErrors();
    } else {
      if (value < 0.0001) {
        break;
      }
    }

    error = nerror;
  }
}

void
MarquardtFitter::updateLambda(double value)
{
  size_t n = LAMBDA.size();
  if (value < 0) {
    for (size_t i = 0; i < n; i++)
      LAMBDA[i] = LAMBDA[i] * 10;
  } else {
    for (size_t i = 0; i < n; i++)
      LAMBDA[i] = LAMBDA[i] * 0.1;
  }
}

/*
 *      Gets the current set of parameters values.
 */
vector<double>
MarquardtFitter::getParameters()
{
  return this->A;
}

/*
 * for stack traces
 */
string
MarquardtFitter::printMatrix()
{
  stringstream message;
  for (size_t i = 0; i < alphaPrimeN; i++) {
    for (size_t j = 0; j < alphaPrimeN; j++) {
      message << ALPHA_PRIME[i][j] << "\t";
    }
    message << "| " << BETA[i] << "\n";
  }
  return message.str();
}

string
MarquardtFitter::printMatrix(vector<vector<double>>& matrix)
{
  stringstream message;
  for (size_t i = 0; i < matrix.size(); i++) {
    for (size_t j = 0; j < matrix[0].size(); j++) {
      message << matrix[i][j] << "\t";
    }
  }
  return message.str();
}
