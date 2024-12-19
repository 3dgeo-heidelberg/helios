#include <pybind11/pybind11.h>
#include <glm/glm.hpp>

namespace pybind11 {
size_t handlePythonIndex(long _index, size_t n) {
    size_t index = static_cast<size_t>(_index);
    if (_index < 0) {
        index = static_cast<size_t>(n + _index);
    }
    if (index >= n) {
        std::stringstream ss;
        ss << "Index " << _index << " out of range";
        throw pybind11::index_error(ss.str());
    }
    return index;
}
}

int calcTimePropagation(std::vector<double> &timeWave, int numBins, Scanner &scanner) {
    return WaveMaths::calcPropagationTimeLegacy(
        timeWave,
        numBins,
        scanner.getFWFSettings(0).binSize_ns,
        scanner.getPulseLength_ns(0),
        7.0  // 3.5 too many ops., 7.0 just one op.
    );
}

template<typename T, size_t N>
py::array_t<T> create_numpy_array(T (&arr)[N]) {
    return py::array_t<T>(N, arr);
}

template<size_t N>
void from_numpy_array(py::array_t<double> arr, double (&out)[N]) {
    if (arr.size() != N) {
        throw std::runtime_error("Input array size does not match expected size.");
    }
    
    // Create a buffer info object to access the data
    auto buf = arr.unchecked<1>(); // Unchecked for 1D access
    
    for (size_t i = 0; i < N; ++i) {
        out[i] = buf(i); // Copy each element into the output array
    }
}