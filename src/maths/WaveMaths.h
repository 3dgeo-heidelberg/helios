#pragma once

#include <cmath>
#include <maths/MathConstants.h>

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Some common mathematical operatiosn concerning the full waveform.
 */
class WaveMaths{
private:
    // ***  STATIC CLASS  *** //
    // ********************** //
    WaveMaths() {};
    virtual ~WaveMaths() = 0;

public:
    // ***   T I M E   *** //
    // ******************* //
    /**
     * @brief Compute propagation time, thus obtaining the intensity peak
     *  index and populating the time wave vector adequately.
     *
     * The legacy algorithm for the estimation of propagation time can be
     *  described as forward iterating in the integer interval \f$[0, n)\f$
     *  such that at each \f$i\f$-th step the \f$i\f$-th bin of the
     *  discrete time vector \f$w_i\f$ is populated, where \f$h\f$ is the
     *  bin size, \f$t_i = ih\f$, and \f$\tau = \lambda / d\f$ with
     *  \f$\lambda\f$ being the pulse length in nanoseconds and
     *  \f$d\f$ a user given divisor:
     *
     * \f[
     * \begin{split}
     *  w_i =& \frac{(ih)^2}{\tau^2} \exp{-\frac{ih}{\tau}} \\
     *      =& \frac{t_i^2}{\tau^2} \exp{-\frac{t_i}{\tau}}
     * \end{split}
     * \f]
     *
     * The peak intensity index is simply obtained by taking the \f$i\f$-th
     *  index of the greatest \f$w_i\f$.
     *
     * @param timeWave The time discretization vector
     * @param numBins The number of bins for time discretization
     * @param binSize The size of each bin (in nanoseconds)
     * @param pulseLength The pulse length (in nanoseconds)
     * @param pulseLengthDivisor The number dividing the pulse length
     * @return The index of the intensity peak in the discrete time vector
     *
     * @see ScanningDevice::numTimeBins
     * @see ScanningDevice::time_wave
     * @see ScanningDevice::peakIntensityIndex
     */
    static int calcPropagationTimeLegacy(
        std::vector<double> &timeWave,
        int const numBins,
        double const binSize,
        double const pulseLength,
        double const pulseLengthDivisor
    );
};