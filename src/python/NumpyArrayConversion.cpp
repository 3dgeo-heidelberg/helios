#include<NumpyArrayConversion.h>

namespace detail {
    std::mutex dtype_mutex;

    struct DTypeCache {
        py::object measurement_dtype;
        std::map<std::string, size_t> meas_offsets;
        py::object trajectory_dtype;
        std::map<std::string, size_t> traj_offsets;
    };

    static DTypeCache& get_dtype_cache() {
        static DTypeCache cache;
        return cache;
    }

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

    const py::object& get_measurement_dtype() {
        auto& cache = get_dtype_cache();
        std::lock_guard<std::mutex> lock(dtype_mutex);
        if (!cache.measurement_dtype) {
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

            cache.measurement_dtype = np.attr("dtype")(meas_fields);

            py::dict fields = cache.measurement_dtype.attr("fields");
            for (auto item : fields) {
                std::string key = py::str(item.first);
                py::tuple info = item.second.cast<py::tuple>();
                cache.meas_offsets[key] = info[1].cast<size_t>();
            }
        }
        return cache.measurement_dtype;
    }

    const py::object& get_trajectory_dtype() {
        auto& cache = get_dtype_cache();
        std::lock_guard<std::mutex> lock(dtype_mutex);
        if (!cache.trajectory_dtype) {
            py::object np = py::module_::import("numpy");
            py::list traj_fields;
            traj_fields.append(py::make_tuple("gps_time", "f8"));
            traj_fields.append(py::make_tuple("position", "3f8"));
            traj_fields.append(py::make_tuple("roll", "f8"));
            traj_fields.append(py::make_tuple("pitch", "f8"));
            traj_fields.append(py::make_tuple("yaw", "f8"));

            cache.trajectory_dtype = np.attr("dtype")(traj_fields);

            py::dict fields = cache.trajectory_dtype.attr("fields");
            for (auto item : fields) {
                std::string key = py::str(item.first);
                py::tuple info = item.second.cast<py::tuple>();
                cache.traj_offsets[key] = info[1].cast<size_t>();
            }
        }
        return cache.trajectory_dtype;
    }

    inline void copy_dvec3(double dest[3], const glm::dvec3 &vec) {
        dest[0] = vec.x;
        dest[1] = vec.y;
        dest[2] = vec.z;
    }

    py::array measurements_to_numpy(const std::vector<Measurement>& measurements) {
        const py::object& dtype = get_measurement_dtype();
        auto& offsets = get_dtype_cache().meas_offsets;
        size_t n = measurements.size();
        py::array result = py::array(dtype, std::vector<size_t>{static_cast<size_t>(n)});
        py::buffer_info buf = result.request();
        char* base_ptr = static_cast<char*>(buf.ptr);
        size_t row_size = buf.itemsize;

        std::array<char32_t, 50> utf32_buffer;
        for (size_t i = 0; i < n; ++i) {
            const Measurement &m = measurements[i];
            char* row = base_ptr + i * row_size;
            {
                size_t off = offsets.at("dev_id");
                char32_t* dest = reinterpret_cast<char32_t*>(row + off);
                convert_utf8_to_fixed_utf32(m.devId, utf32_buffer);
                memcpy(dest, utf32_buffer.data(), utf32_buffer.size() * sizeof(char32_t));
            }
            write_field(row, offsets.at("dev_idx"), m.devIdx);
            {
                size_t off = offsets.at("hit_object_id");
                char32_t* dest = reinterpret_cast<char32_t*>(row + off);
                convert_utf8_to_fixed_utf32(m.hitObjectId, utf32_buffer);
                memcpy(dest, utf32_buffer.data(), utf32_buffer.size() * sizeof(char32_t));
            }
            write_field(row, offsets.at("position"), [&m](char* dest) { copy_dvec3(reinterpret_cast<double*>(dest), m.position); });
            write_field(row, offsets.at("beam_direction"), [&m](char* dest) { copy_dvec3(reinterpret_cast<double*>(dest), m.beamDirection); });
            write_field(row, offsets.at("beam_origin"), [&m](char* dest) { copy_dvec3(reinterpret_cast<double*>(dest), m.beamOrigin); });
            write_field(row, offsets.at("distance"), m.distance);
            write_field(row, offsets.at("intensity"), m.intensity);
            write_field(row, offsets.at("echo_width"), m.echo_width);
            write_field(row, offsets.at("return_number"), m.returnNumber);
            write_field(row, offsets.at("pulse_return_number"), m.pulseReturnNumber);
            write_field(row, offsets.at("fullwave_index"), m.fullwaveIndex);
            write_field(row, offsets.at("classification"), m.classification);
            write_field(row, offsets.at("gps_time"), m.gpsTime);
        }
        return result;
    }

    std::vector<Measurement> numpy_to_measurements(py::array arr) {
        py::buffer_info buf = arr.request();
        if (buf.ndim != 1)
            throw std::runtime_error("Measurement array must be 1-dimensional");
        size_t n = buf.shape[0];
        auto& offsets = get_dtype_cache().meas_offsets;
        size_t row_size = buf.itemsize;
        char* base_ptr = static_cast<char*>(buf.ptr);

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
            m.gpsTime = read_field<double>(row, offsets.at("gps_time"));
            
            vec.push_back(std::move(m));
        }
        return vec;
    }

    py::array trajectories_to_numpy(const std::vector<Trajectory>& trajectories) {
        const py::object& dtype = get_trajectory_dtype();
        auto& offsets = get_dtype_cache().traj_offsets;
        size_t n = trajectories.size();
        py::array result = py::array(dtype, std::vector<size_t>{static_cast<size_t>(n)});
        py::buffer_info buf = result.request();
        char* base_ptr = static_cast<char*>(buf.ptr);
        size_t row_size = buf.itemsize;
        for (size_t i = 0; i < n; ++i) {
            const Trajectory &t = trajectories[i];
            char* row = base_ptr + i * row_size;
            write_field(row, offsets.at("gps_time"), t.gpsTime);
            {
                size_t off = offsets.at("position");
                double* dest = reinterpret_cast<double*>(row + off);
                copy_dvec3(dest, t.position);
            }
            write_field(row, offsets.at("roll"), t.roll);
            write_field(row, offsets.at("pitch"), t.pitch);
            write_field(row, offsets.at("yaw"), t.yaw);
        }
        return result;
    }

    std::vector<Trajectory> numpy_to_trajectories(py::array arr) {
        py::buffer_info buf = arr.request();
        if (buf.ndim != 1)
            throw std::runtime_error("Trajectory array must be 1-dimensional");
        size_t n = buf.shape[0];
        auto& offsets = get_dtype_cache().traj_offsets;
        size_t row_size = buf.itemsize;
        char* base_ptr = static_cast<char*>(buf.ptr);
        std::vector<Trajectory> vec;
        vec.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            Trajectory t;
            char* row = base_ptr + i * row_size;
            t.gpsTime = read_field<double>(row, offsets.at("gps_time"));
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