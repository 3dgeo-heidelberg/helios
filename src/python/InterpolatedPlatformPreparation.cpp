#include <python/InterpolatedPlatformPreparation.h>

std::shared_ptr<Platform>
load_interpolated_platform(std::shared_ptr<LinearPathPlatform> basePlatform,
                           py::array trajectory,
                           std::string rotspec,
                           bool syncGPSTime,
                           bool isRollPitchYawInRadians)
{
  std::shared_ptr<InterpolatedMovingPlatformEgg> platform =
    std::make_shared<InterpolatedMovingPlatformEgg>();

  py::buffer_info buf;
  try {
    buf = trajectory.request();
  } catch (const py::error_already_set& e) {
    std::ostringstream ss;
    ss << "Failed to export trajectory as a buffer. "
       << "Make sure it is a 1-D structured array with non-overlapping, "
       << "in-order fields and float64 dtype for all fields. "
       << "Original error: " << e.what();
    throw std::runtime_error(ss.str());
  }

  if (buf.ndim != 1) {
    throw std::runtime_error(
      "Trajectory must be a 1-D structured NumPy array (shape=(n,)).");
  }

  std::array<const char*, 7> reqNames = { "t", "roll", "pitch", "yaw",
                                          "x", "y",    "z" };
  py::object dtype = trajectory.attr("dtype");
  py::object names_obj = dtype.attr("names");
  if (names_obj.is_none()) {
    throw std::runtime_error(
      "Trajectory must be a structured NumPy array with named fields.");
  }
  py::tuple names = dtype.attr("names").cast<py::tuple>();
  if (names.size() != reqNames.size()) {
    throw std::runtime_error("Trajectory dtype must have exactly 7 fields: t, "
                             "roll, pitch, yaw, x, y, z.");
  }
  for (size_t i = 0; i < reqNames.size(); ++i) {
    if (names[i].cast<std::string>() != reqNames[i]) {
      throw std::runtime_error("Trajectory dtype fields must be in order: t, "
                               "roll, pitch, yaw, x, y, z.");
    }
  }

  size_t m = buf.shape[0];
  size_t rec_size = buf.itemsize;
  size_t stride = buf.strides[0];
  char* base = static_cast<char*>(buf.ptr);

  // Get the dtype.fields dict and the names tuple
  py::dict fields = dtype.attr("fields").cast<py::dict>();
  size_t n_fields = names.size();

  // Precompute offsets and collect column‐names
  std::vector<size_t> offsets;
  offsets.reserve(n_fields);
  std::vector<std::string> colnames;
  colnames.reserve(n_fields);
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
    char* row_ptr = base + i * stride;
    for (size_t j = 0; j < n_fields; ++j) {
      double* cell = reinterpret_cast<double*>(row_ptr + offsets[j]);
      fullX(i, j) = *cell;
    }
  }

  // Wrap into a DesignMatrix (with column‐names!), then into
  // TemporalDesignMatrix
  DesignMatrix<double> dm(fullX, colnames);
  // time column is assumed to be the first field (j==0)
  platform->tdm = std::make_shared<TemporalDesignMatrix<double, double>>(dm, 0);

  double startTime = 0.0;
  startTime = arma::min(platform->tdm->getTimeVector());
  platform->startTime = startTime;
  platform->tdm->shiftTime(-startTime);
  platform->syncGPSTime = syncGPSTime;
  if (!isRollPitchYawInRadians) {
    for (size_t j = 0; j < 3; ++j) {
      platform->tdm->setColumn(j, platform->tdm->getColumn(j) * PI_OVER_180);
    }
  }

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
