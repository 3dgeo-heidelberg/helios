#pragma once

#include <string>
#include <vector>

/**
 * @brief Class to compute Marquardt fitter
 */
class MarquardtFitter {
    // ***  ATTRIBUTES  *** //
    // ******************** //
	/*
	 * for solving non-linear least squares fit of f(x:A) = z with sets of x,z data points.
	 */
	/**
	 * @brief X input vector
	 */
	std::vector<std::vector<double>> X; //values
	/**
	 * @brief Z output vecctor
	 */
	std::vector<double> Z; //output values
	/**
	 * @brief A arguments vector
	 */
	std::vector<double> A; //Parameter Set

	/**
	 * @brief Alpha prime vector dimensionality
	 */
	size_t alphaPrimeN = 0;
	/**
	 * @brief Reference to alpha prime vector
	 */
	std::vector<std::vector<double>>& ALPHA_PRIME;
	/**
	 * @brief Vector containing derivatives (jacobian vector)
	 */
	std::vector<std::vector<double>> DERIVATIVES;
	/**
	 * @brief Beta vector
	 */
	std::vector<double> BETA;
	/**
	 * @brief Lambda vector
	 */
	std::vector<double> LAMBDA;
	/**
	 * @brief Error vector
	 */
	std::vector<double> ERR;

	//double DELTA = 0.000001;	//used for calculating derivatives
	/**
	 * @brief Delta for numerical derivatives
	 */
	double DELTA = 1e-6;
	/**
	 * @brief Mnimum error for evaluating. It is no longer used and it might be
	 * removed in the future
	 */
	double MINERROR = 1e-9;		//for evaluating
	/**
	 * @brief Minimum change. It is no longer used and it might be removed in
	 * the future
	 */
	double MINCHANGE = 1e-3;	//minimumchanges


public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base constructor for MarquardtFitter
     * @param apMatrix Reference to the alpha prime matrix. It is passed this
     * way to reduce the number of memory management operations, mainly due to
     * using multiple marquardt fitters
     */
    explicit MarquardtFitter(std::vector<std::vector<double>>& apMatrix)
    : ALPHA_PRIME(apMatrix){}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Set the values of the original data points that are going to be
     * fit
     * @param zvalues Values of the original data points
     */
	void setData(const std::vector<double> & zvalues);
	/**
	 * @brief Obtain marquardt fitter current arguments/parameters
	 * @return Marquardt fitter current arguments/parameters
	 */
    std::vector<double> getParameters();
	/**
	 * @brief Set the arguments/parameters for the fitter
	 * @param parameters Parameters for the fitter
	 */
	void setParameters(const std::vector<double> & parameters);
	/**
	 * @brief Get a submatrix
	 * @param A Matrix where values are taken from
	 * @param r Array of row indices
	 * @param j0 Initial column index
	 * @param j1 Final column index
	 * @return
	 */
    std::vector<std::vector<double>> getMatrix(
        std::vector<std::vector<double>> & A,
        std::vector<int> & r,
        int j0,
        int j1
    );

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Evaluate the marquardt fitter
     * @param values Values to evaluate over. At the moment values[0] is the
     * only considered one.
     * @param params Arguments/parameters which define the evaluation
     * @return Marquardt fitter evaluation
     */
	double evaluate(const std::vector<double> & values, const std::vector<double> & params);
	/**
	 * @brief Compute the cumulative error for current values
	 * @return Cumulative error for current values
	 */
	double calculateErrors();
    /**
    * @brief Given a set of parameters, and inputs, calculates the derivative
    *  of the k'th parameter
    *
    * \f[
    *  \frac{\partial{f}}{\partial{a_{k}}}
    * \f]
    *
    * @param k Index of the parameter that the derivative is being taken of
    * @param params Array that will be used to calculate derivatives, note params will be modified.
    * @param x Set of values to use.
    * @return Computed derivative
     * @see MarquardtFitter::calculateDerivativeFast(
     *  double, double, double, double, std::vector<double>&)
    **/
	double calculateDerivative(int k, std::vector<double> & values, std::vector<double> & params);
	/**
	 * @brief Alternative method for faster derivative computation
	 * @param x Input for the Marquardt fitter
	 * @param c Third argument (index 2)
	 * @param d Fourth argument (index 3)
	 * @param coefficient Coefficient calculated with respect to second argument
	 * (index 1)
	 * @param[out] dvec Vector of partial derivatives (where the output is
	 * stored)
	 * @see MarquardtFitter::calculateDerivative(int, std::vector<double>&,
	 *  std::vector<double>&)
	 */
	void calculateDerivativeFast(
	    double x,
	    double c,
	    double d,
	    double coefficient,
	    std::vector<double>& dvec
    );
	/**
	 * @brief Compute matrix of partial derivatives
	 * @see calculateDerivativesFast
	 */
	void calculateDerivatives();
	/**
	 * @brief Alternative method for faster computation of partial derivatives
	 * matrix
	 * @see calculateDerivatives
	 */
	void calculateDerivativesFast();
	/**
	 * @brief Create the beta matrix
	 */
	void createBetaMatrix();
	/**
	 * @brief Create the alpha prime matrix
	 */
	void createAlphaPrimeMatrix();
	/**
	 * @brief Iterate over values considering the current error and
	 *  arguments/parameters and compute the changes
	 */
	void iterateValues();
	/**
	 * @brief Initializes the workspace for the Marquardt fitter
	 */
	void initializeWorkspace();
	/**
	 * @brief Iterate until fitting is finished
	 */
	void fitData();
	/**
	 * @brief Update lambda
	 *
	 * When value is less than $0$, then lambda is multiplied by $10$.
	 * Otherwise it is multiplied by $0.1$.
	 *
	 * @param value Update value
	 */
	void updateLambda(double value);
	/**
	 * @brief Build a string representing the alpha prime matrix together with
	 * beta vector for debugging purposes
	 * @return String representing the alpha prime matrix together with
	 * beta vector
	 */
	std::string printMatrix();
	/**
	 * @brief Build a string representing given matrix for debugging purposes
	 * @param matrix Matrix to be debugged
	 * @return String representing given matrix
	 */
	std::string printMatrix(std::vector<std::vector<double>> & matrix);
};