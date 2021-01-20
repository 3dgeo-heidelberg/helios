#pragma once

#ifdef PYTHON_BINDING

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for RandomnessGenerator class
 */
class PyRandomnessGeneratorWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    RandomnessGenerator<double> &rg;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PyRandomnessGeneratorWrapper(RandomnessGenerator<double> &rg) : rg(rg) {};
    virtual ~PyRandomnessGeneratorWrapper(){}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    void computeUniformRealDistribution(double lowerBound, double upperBound)
        {rg.computeUniformRealDistribution(lowerBound, upperBound);}
    double uniformRealDistributionNext()
        {return rg.uniformRealDistributionNext();}
    void computeNormalDistribution(double mean, double stdev)
        {return rg.computeNormalDistribution(mean, stdev);}
    double normalDistributionNext()
        {return rg.normalDistributionNext();}
};

#endif