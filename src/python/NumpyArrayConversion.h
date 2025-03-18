#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <string>
#include <cstring>  
#include <cstddef>  
#include <stdexcept>
#include <map>
#include <mutex>
#include <algorithm>
#include <codecvt>
#include <locale>
#include <Measurement.h>
#include <Trajectory.h>

#ifdef _MSC_VER
using ssize_t = std::ptrdiff_t;
#endif

namespace py = pybind11;
namespace detail {

    void convert_utf8_to_fixed_utf32(const std::string &src, std::array<char32_t, 50>& dest) {
        std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
        std::u32string u32 = conv.from_bytes(src);
        dest.fill(U'\0');
        size_t len = std::min(u32.size(), size_t(dest.size()));
        std::copy(u32.begin(), u32.begin() + len, dest.begin());
    }
    
    std::string convert_fixed_utf32_to_utf8(const char32_t* src) {
        size_t len = 0;
        while (len < 50 && src[len] != U'\0') {
            ++len;
        }
        std::u32string u32(src, src + len);
        std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
        return conv.to_bytes(u32);
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

    inline void copy_dvec3(double dest[3], const glm::dvec3 &vec) {
        dest[0] = vec.x;
        dest[1] = vec.y;
        dest[2] = vec.z;
    }
    
    py::array measurements_to_numpy(const std::vector<Measurement>& measurements) {
        py::gil_scoped_acquire gil;
    
        py::object np = py::module_::import("numpy");
        py::list meas_fields;
    
        meas_fields.append(py::make_tuple("dev_id", "U50"));
        meas_fields.append(py::make_tuple("dev_idx", "u8"));
        meas_fields.append(py::make_tuple("hit_object_id", "U50"));
        meas_fields.append(py::make_tuple("position", "3f8"));
        meas_fields.append(py::make_tuple("beam_direction", "3f8"));
        meas_fields.append(py::make_tuple("beam_origin", "3f8"));
        meas_fields.append(py::make_tuple("distance", "f8"));
        meas_fields.append(py::make_tuple("intensity", "f8"));
        meas_fields.append(py::make_tuple("echo_width", "f8"));
        meas_fields.append(py::make_tuple("return_number", "i4"));
        meas_fields.append(py::make_tuple("pulse_return_number", "i4"));
        meas_fields.append(py::make_tuple("fullwave_index", "i4"));
        meas_fields.append(py::make_tuple("classification", "i4"));
        meas_fields.append(py::make_tuple("gps_time", "f8"));
    
        py::object dtype = np.attr("dtype")(meas_fields);
        py::dict fields = dtype.attr("fields");
    
        std::map<std::string, size_t> offsets;
        for (auto item : fields) {
            std::string key = py::str(item.first);
            py::tuple info = item.second.cast<py::tuple>();
            offsets[key] = info[1].cast<size_t>();
        }
    
        size_t n = measurements.size();
        py::array result = py::array(dtype, std::vector<ssize_t>{static_cast<ssize_t>(n)});
        py::buffer_info buf = result.request();
        char* base_ptr = static_cast<char*>(buf.ptr);
        size_t row_size = buf.itemsize;
        
        size_t off_dev_id = offsets["dev_id"];
        size_t off_dev_idx = offsets["dev_idx"];
        size_t off_hit_id = offsets["hit_object_id"];
        size_t off_pos = offsets["position"];
        size_t off_beam_dir = offsets["beam_direction"];
        size_t off_beam_orig = offsets["beam_origin"];
        size_t off_dist = offsets["distance"];
        size_t off_intens = offsets["intensity"];
        size_t off_width = offsets["echo_width"];
        size_t off_ret = offsets["return_number"];
        size_t off_pulse_ret = offsets["pulse_return_number"];
        size_t off_full = offsets["fullwave_index"];
        size_t off_class = offsets["classification"];
        size_t off_time = offsets["gps_time"];
    
        std::array<char32_t, 50> utf32_buffer;
    
        for (size_t i = 0; i < n; ++i) {
            const Measurement& m = measurements[i];
            char* row = base_ptr + i * row_size;
    
            convert_utf8_to_fixed_utf32(m.devId, utf32_buffer);
            std::memcpy(row + off_dev_id, utf32_buffer.data(), utf32_buffer.size() * sizeof(char32_t));
    
            *reinterpret_cast<uint64_t*>(row + off_dev_idx) = m.devIdx;
    
            convert_utf8_to_fixed_utf32(m.hitObjectId, utf32_buffer);
            std::memcpy(row + off_hit_id, utf32_buffer.data(), utf32_buffer.size() * sizeof(char32_t));
    
            copy_dvec3(reinterpret_cast<double*>(row + off_pos), m.position);
            copy_dvec3(reinterpret_cast<double*>(row + off_beam_dir), m.beamDirection);
            copy_dvec3(reinterpret_cast<double*>(row + off_beam_orig), m.beamOrigin);
    
            *reinterpret_cast<double*>(row + off_dist) = m.distance;
            *reinterpret_cast<double*>(row + off_intens) = m.intensity;
            *reinterpret_cast<double*>(row + off_width) = m.echo_width;
            *reinterpret_cast<int32_t*>(row + off_ret) = m.returnNumber;
            *reinterpret_cast<int32_t*>(row + off_pulse_ret) = m.pulseReturnNumber;
            *reinterpret_cast<int32_t*>(row + off_full) = m.fullwaveIndex;
            *reinterpret_cast<int32_t*>(row + off_class) = m.classification;
            *reinterpret_cast<double*>(row + off_time) = m.gpsTime / 1e9;
        }
    
        return result;
    }
    
    std::vector<Measurement> numpy_to_measurements(py::array arr) {
        py::gil_scoped_acquire gil;
        py::buffer_info buf = arr.request();
        if (buf.ndim != 1)
            throw std::runtime_error("Measurement array must be 1-dimensional");
    
        py::object dtype = arr.attr("dtype");
        py::dict fields = dtype.attr("fields");
        size_t row_size = buf.itemsize;
        char* base_ptr = static_cast<char*>(buf.ptr);

        std::map<std::string, size_t> offsets;
        for (auto item : fields) {
            std::string key = py::str(item.first);
            py::tuple info = item.second.cast<py::tuple>();
            offsets[key] = info[1].cast<size_t>();
        }
    
        size_t n = buf.shape[0];
        std::vector<Measurement> vec;
        vec.reserve(n);
    
        for (size_t i = 0; i < n; ++i) {
            Measurement m;
            char* row = base_ptr + i * row_size;
            
            m.devId = read_field(row, offsets.at("dev_id"), [](const char* ptr) {
                return convert_fixed_utf32_to_utf8(reinterpret_cast<const char32_t*>(ptr));
            });
            m.devIdx = read_field<uint64_t>(row, offsets.at("dev_idx"));
            m.hitObjectId = read_field(row, offsets.at("hit_object_id"), [](const char* ptr) {
                return convert_fixed_utf32_to_utf8(reinterpret_cast<const char32_t*>(ptr));
            });
            m.position = read_field(row, offsets.at("position"), [](const char* ptr) {
                const double* src = reinterpret_cast<const double*>(ptr);
                return glm::dvec3(src[0], src[1], src[2]);
            });
            m.beamDirection = read_field(row, offsets.at("beam_direction"), [](const char* ptr) {
                const double* src = reinterpret_cast<const double*>(ptr);
                return glm::dvec3(src[0], src[1], src[2]);
            });
            m.beamOrigin = read_field(row, offsets.at("beam_origin"), [](const char* ptr) {
                const double* src = reinterpret_cast<const double*>(ptr);
                return glm::dvec3(src[0], src[1], src[2]);
            });
            m.distance = read_field<double>(row, offsets.at("distance"));
            m.intensity = read_field<double>(row, offsets.at("intensity"));
            m.echo_width = read_field<double>(row, offsets.at("echo_width"));
            m.returnNumber = read_field<int32_t>(row, offsets.at("return_number"));
            m.pulseReturnNumber = read_field<int32_t>(row, offsets.at("pulse_return_number"));
            m.fullwaveIndex = read_field<int32_t>(row, offsets.at("fullwave_index"));
            m.classification = read_field<int32_t>(row, offsets.at("classification"));
            m.gpsTime = read_field<double>(row, offsets.at("gps_time")) * 1e9;
            
            vec.push_back(std::move(m));
        }
        return vec;
    }
    
    py::array trajectories_to_numpy(const std::vector<Trajectory>& trajectories) {
        py::gil_scoped_acquire gil;
    
        py::object np = py::module_::import("numpy");
        py::list traj_fields;
    
        traj_fields.append(py::make_tuple("gps_time", "f8"));
        traj_fields.append(py::make_tuple("position", "3f8"));
        traj_fields.append(py::make_tuple("roll", "f8"));
        traj_fields.append(py::make_tuple("pitch", "f8"));
        traj_fields.append(py::make_tuple("yaw", "f8"));
    
        py::object dtype = np.attr("dtype")(traj_fields);
        py::dict fields = dtype.attr("fields");
    
        std::map<std::string, size_t> offsets;
        for (auto item : fields) {
            std::string key = py::str(item.first);
            py::tuple info = item.second.cast<py::tuple>();
            offsets[key] = info[1].cast<size_t>();
        }
    
        size_t n = trajectories.size();
        py::array result = py::array(dtype, std::vector<ssize_t>{static_cast<ssize_t>(n)});
        py::buffer_info buf = result.request();
        char* base_ptr = static_cast<char*>(buf.ptr);
        size_t row_size = buf.itemsize;

        size_t off_time = offsets["gps_time"];
        size_t off_pos = offsets["position"];
        size_t off_roll = offsets["roll"];
        size_t off_pitch = offsets["pitch"];
        size_t off_yaw = offsets["yaw"];

        for (size_t i = 0; i < n; ++i) {
            const Trajectory &t = trajectories[i];
            char* row = base_ptr + i * row_size;
    
            *reinterpret_cast<double*>(row + off_time) = t.gpsTime / 1e9;
            copy_dvec3(reinterpret_cast<double*>(row + off_pos), t.position);
            *reinterpret_cast<double*>(row + off_roll) = t.roll;
            *reinterpret_cast<double*>(row + off_pitch) = t.pitch;
            *reinterpret_cast<double*>(row + off_yaw) = t.yaw;
        }
        return result;
    }
    
    std::vector<Trajectory> numpy_to_trajectories(py::array arr) {
        py::gil_scoped_acquire gil;
    
        py::buffer_info buf = arr.request();
        if (buf.ndim != 1)
            throw std::runtime_error("Trajectory array must be 1-dimensional");
    
        py::object dtype = arr.attr("dtype");
        py::dict fields = dtype.attr("fields");
        size_t row_size = buf.itemsize;
        char* base_ptr = static_cast<char*>(buf.ptr);
    
        std::map<std::string, size_t> offsets;
        for (auto item : fields) {
            std::string key = py::str(item.first);
            py::tuple info = item.second.cast<py::tuple>();
            offsets[key] = info[1].cast<size_t>();
        }
    
        size_t n = buf.shape[0];
        std::vector<Trajectory> vec;
        vec.reserve(n);
    
        for (size_t i = 0; i < n; ++i) {
            Trajectory t;
            char* row = base_ptr + i * row_size;
            t.gpsTime = read_field<double>(row, offsets.at("gps_time")) * 1e9;
            t.position = read_field(row, offsets.at("position"), [](const char* ptr) {
                const double* src = reinterpret_cast<const double*>(ptr);
                return glm::dvec3(src[0], src[1], src[2]);
            });
            t.roll = read_field<double>(row, offsets.at("roll"));
            t.pitch = read_field<double>(row, offsets.at("pitch"));
            t.yaw = read_field<double>(row, offsets.at("yaw"));
            vec.push_back(std::move(t));
        }
        return vec;
    }
    
    } 
