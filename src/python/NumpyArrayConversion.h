#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <glm/glm.hpp>
#include <array>
#include <memory>
#include <vector>
#include <string>
#include <cstring>    
#include <stdexcept>
#include <map>
#include <mutex>
#include <algorithm>
#include <codecvt>
#include <locale>
#include <Measurement.h>
#include <Trajectory.h>

namespace py = pybind11;
namespace detail {

    void convert_utf8_to_fixed_utf32(const std::string &src, std::array<char32_t, 50>& dest);
    std::string convert_fixed_utf32_to_utf8(const char32_t* src);

    const py::object& get_measurement_dtype();
    const py::object& get_trajectory_dtype();

    template <typename T>
    void write_field(char* row, size_t offset, T&& value);

    template <typename T>
    T read_field(const char* row, size_t offset);

    py::array measurements_to_numpy(const std::vector<Measurement>& measurements);
    std::vector<Measurement> numpy_to_measurements(py::array arr);

    py::array trajectories_to_numpy(const std::vector<Trajectory>& trajectories);
    std::vector<Trajectory> numpy_to_trajectories(py::array arr);

    template <typename T>
    inline auto write_field_impl(char* row, size_t offset, T&& value)
        -> decltype(*reinterpret_cast<std::decay_t<T>*>(row + offset) = std::forward<T>(value), void())
    {
        *reinterpret_cast<std::decay_t<T>*>(row + offset) = std::forward<T>(value);
    }

    template <typename F>
    inline auto write_field_impl(char* row, size_t offset, F&& fn)
        -> decltype(std::forward<F>(fn)(row + offset), void())
    {
        std::forward<F>(fn)(row + offset);
    }

    template <typename T>
    inline void write_field(char* row, size_t offset, T&& value) {
        write_field_impl(row, offset, std::forward<T>(value));
    }

    template <typename T>
    inline T read_field(const char* row, size_t offset) {
        return *reinterpret_cast<const T*>(row + offset);
    }

    template <typename F>
    inline auto read_field(const char* row, size_t offset, F&& fn)
        -> decltype(std::forward<F>(fn)(row + offset))
    {
        return std::forward<F>(fn)(row + offset);
    }

}
