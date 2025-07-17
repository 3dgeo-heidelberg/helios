#include <python/InterpolatedPlatformPreparation.h>


// std::shared_ptr<InterpolatedMovingPlatformEgg>
std::shared_ptr<Platform>
load_interpolated_platform(std::shared_ptr<LinearPathPlatform> basePlatform,
                py::array trajectory,
                std::string rotspec,
                bool syncGPSTime
            ) {
    std::shared_ptr<InterpolatedMovingPlatformEgg> platform =
    std::make_shared<InterpolatedMovingPlatformEgg>();

    // Acquire buffer
    auto buf = trajectory.request();
    if (buf.ndim != 1)
        throw std::runtime_error("Trajectory array must be 1-dimensional");

    // shape = (m,), each item is a struct of n doubles
    size_t m = buf.shape[0];
    size_t rec_size = buf.itemsize;
    size_t stride  = buf.strides[0];
    char* base = static_cast<char*>(buf.ptr);

    // Get the dtype.fields dict and the names tuple
    py::object dtype  = trajectory.attr("dtype");
    py::dict fields  = dtype.attr("fields").cast<py::dict>();
    py::tuple names   = dtype.attr("names").cast<py::tuple>();
    size_t n_fields = names.size();

    // Precompute offsets and collect column‐names
    std::vector<size_t> offsets; offsets.reserve(n_fields);
    std::vector<std::string> colnames; colnames.reserve(n_fields);
    for (size_t j = 0; j < n_fields; ++j) {
        std::string name = names[j].cast<std::string>();
        // dtype.fields[name] -> (dtype, offset[, title])
        py::tuple meta = fields[name.c_str()].cast<py::tuple>();
        size_t offset = meta[1].cast<size_t>();
        offsets.push_back(offset);
        colnames.push_back(name);
    }

    // Fill an Armadillo matrix of size m x n_fields
    arma::Mat<double> fullX(m, n_fields);
    for (size_t i = 0; i < m; ++i) {
        char* row_ptr = base + i*stride;
        for (size_t j = 0; j < n_fields; ++j) {
            double* cell = reinterpret_cast<double*>(row_ptr + offsets[j]);
            fullX(i,j) = *cell;
        }
    }

    // Wrap into a DesignMatrix (with column‐names!), then into TemporalDesignMatrix
    DesignMatrix<double> dm(fullX, colnames);
    // time column is assumed to be the first field (j==0)
    platform->tdm = std::make_shared< TemporalDesignMatrix<double,double> >(dm, 0); 

    double startTime = 0.0;
    startTime = arma::min(platform->tdm->getTimeVector());
    platform->startTime = startTime;
    platform->tdm->shiftTime(-startTime);
    platform->syncGPSTime = syncGPSTime;

    platform->ddm = platform->tdm->toDiffDesignMatrixPointer(
        fluxionum::DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES, false);
    
    if (rotspec == "CANONICAL")
        platform->rotspec = InterpolatedMovingPlatform::RotationSpec::CANONICAL;
    else if (rotspec == "ARINC 705")
        platform->rotspec = InterpolatedMovingPlatform::RotationSpec::ARINC_705;
    
    platform->cfg_device_relativeMountPosition =
     basePlatform->cfg_device_relativeMountPosition;
    platform->cfg_device_relativeMountAttitude =
     basePlatform->cfg_device_relativeMountAttitude;
    // Also, propagate noise sources from base platform to interpolated
    platform->positionXNoiseSource = basePlatform->positionXNoiseSource;
    platform->positionYNoiseSource = basePlatform->positionYNoiseSource;
    platform->positionZNoiseSource = basePlatform->positionZNoiseSource;
    platform->attitudeXNoiseSource = basePlatform->attitudeXNoiseSource;
    platform->attitudeYNoiseSource = basePlatform->attitudeYNoiseSource;
    platform->attitudeZNoiseSource = basePlatform->attitudeZNoiseSource;

    return platform;
    
}