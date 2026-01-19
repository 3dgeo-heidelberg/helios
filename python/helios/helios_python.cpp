#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <gdal_priv.h>

#include <assetloading/ScenePart.h>
#include <platform/Platform.h>
#include <platform/PlatformSettings.h>
#include <scanner/FWFSettings.h>
#include <scanner/Measurement.h>
#include <scanner/Scanner.h>
#include <scanner/ScannerSettings.h>
#include <scanner/Trajectory.h>
#include <scene/Scene.h>
#include <sim/comps/Leg.h>
#include <sim/comps/Survey.h>

bool logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
  logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_TIME,
  logging::LOGGING_SHOW_WARN, logging::LOGGING_SHOW_ERR;

namespace py = pybind11;

using VectorString = std::vector<std::string>;

// Declare opaque types first
PYBIND11_MAKE_OPAQUE(std::vector<double>);
PYBIND11_MAKE_OPAQUE(std::vector<std::string>);
PYBIND11_MAKE_OPAQUE(std::vector<Measurement>);
PYBIND11_MAKE_OPAQUE(std::vector<Trajectory>);

#include <maths/Rotation.h>
#include <maths/WaveMaths.h>
#include <noise/NoiseSource.h>
#include <noise/RandomnessGenerator.h>
#include <scanner/beamDeflector/AbstractBeamDeflector.h>
#include <scanner/detector/AbstractDetector.h>
#include <scene/Material.h>
#include <scene/RaySceneIntersection.h>
#include <scene/primitives/AABB.h>
#include <scene/primitives/DetailedVoxel.h>
#include <scene/primitives/Primitive.h>
#include <scene/primitives/Triangle.h>
#include <scene/primitives/Vertex.h>
#include <sim/comps/SimulationCycleCallback.h>

#include <DynMovingObject.h>
#include <DynObject.h>
#include <DynScene.h>

#include <scanner/MultiScanner.h>
#include <scanner/ScannerHead.h>
#include <scanner/ScanningDevice.h>
#include <scanner/ScanningPulseProcess.h>
#include <scanner/SingleScanner.h>
#include <sim/core/SurveyPlayback.h>

#include <assetloading/geometryfilter/WavefrontObj.h>
#include <filems/facade/FMSFacade.h>
#include <python/AbstractDetectorWrap.h>
#include <python/GLMTypeCaster.h>
#include <python/NoiseSourceWrap.h>
#include <python/PyHeliosSimulation.h>
#include <python/ScannerWrap.h>
#include <python/SimulationWrap.h>
#include <python/utils.h>
#include <sim/comps/ScanningStrip.h>

namespace pyhelios {

PYBIND11_MODULE(_helios, m)
{
  m.doc() = "Helios python bindings";

  py::bind_vector<std::vector<std::string>>(m, "StringVector");
  py::bind_vector<std::vector<Measurement>>(m, "MeasurementVector");
  py::bind_vector<std::vector<Trajectory>>(m, "TrajectoryVector");
  py::bind_vector<std::vector<double>>(
    m, "DoubleVector"); // CHECK THIS!!!! : you'll need to make sure that you
                        // correctly handle the vector's indexing and
                        // modification methods.

  py::implicitly_convertible<py::iterable, VectorString>();

  logging::makeQuiet();
  logging::configure({ { "type", "std_out" } });

  // Enable GDAL (Load its drivers)
  GDALAllRegister();

  // Definitions
  m.def("logging_quiet",
        &logging::makeQuiet,
        "Set the logging verbosity level to quiet");
  m.def("logging_silent",
        &logging::makeSilent,
        "Set the logging verbosity level to silent");
  m.def("logging_default",
        &logging::makeDefault,
        "Set the logging verbosity level to default");
  m.def("logging_verbose",
        &logging::makeVerbose,
        "Set the logging verbosity level to verbose");
  m.def("logging_verbose2",
        &logging::makeVerbose2,
        "Set the logging verbosity level to verbose 2");
  m.def("logging_time",
        &logging::makeTime,
        "Set the logging verbosity level to time");

  m.def("default_rand_generator_seed",
        &setDefaultRandomnessGeneratorSeed,
        "Set the seed for the default randomness generator");

  py::class_<AABB> aabb(m, "AABB");
  aabb
    .def_static(
      "create",
      []() { return std::make_unique<AABB>(); },
      py::return_value_policy::take_ownership)
    .def_property_readonly(
      "min_vertex",
      [](AABB& aabb) { return &(aabb.vertices[0]); },
      py::return_value_policy::reference)
    .def_property_readonly(
      "max_vertex",
      [](AABB& aabb) { return &(aabb.vertices[1]); },
      py::return_value_policy::reference)
    .def("__str__", &AABB::toString);

  py::class_<AbstractBeamDeflector, std::shared_ptr<AbstractBeamDeflector>>
    abstract_beam_deflector(m, "AbstractBeamDeflector");
  abstract_beam_deflector
    .def_readwrite("scan_freq_max",
                   &AbstractBeamDeflector::cfg_device_scanFreqMax_Hz)
    .def_readwrite("scan_freq_min",
                   &AbstractBeamDeflector::cfg_device_scanFreqMin_Hz)
    .def_readwrite("scan_angle_max",
                   &AbstractBeamDeflector::cfg_device_scanAngleMax_rad)
    .def_readwrite("scan_freq", &AbstractBeamDeflector::cfg_setting_scanFreq_Hz)
    .def_readwrite("scan_angle",
                   &AbstractBeamDeflector::cfg_setting_scanAngle_rad)
    .def_readwrite("vertical_angle_min",
                   &AbstractBeamDeflector::cfg_setting_verticalAngleMin_rad)
    .def_readwrite("vertical_angle_max",
                   &AbstractBeamDeflector::cfg_setting_verticalAngleMax_rad)
    .def_readwrite("current_beam_angle",
                   &AbstractBeamDeflector::state_currentBeamAngle_rad)
    .def_readwrite("angle_diff_rad",
                   &AbstractBeamDeflector::state_angleDiff_rad)
    .def_readwrite("cached_angle_between_pulses",
                   &AbstractBeamDeflector::cached_angleBetweenPulses_rad)
    .def_property_readonly(
      "emitter_relative_attitude",
      &AbstractBeamDeflector::getEmitterRelativeAttitudeByReference)
    .def_property_readonly("optics_type",
                           &AbstractBeamDeflector::getOpticsType);

  py::class_<Primitive> primitive(m, "Primitive");
  primitive
    .def_property_readonly("scene_part",
                           [](Primitive& prim) { return prim.part.get(); })
    .def_property_readonly("material",
                           [](Primitive& prim) { return prim.material.get(); })
    .def_property_readonly("AABB", &Primitive::getAABB)
    .def_property_readonly("centroid", &Primitive::getCentroid)
    .def_property_readonly("num_vertices", &Primitive::getNumVertices)
    .def_property_readonly(
      "vertices",
      [](Primitive& prim, size_t index) { return &prim.getVertices()[index]; })

    .def(
      "incidence_angle",
      [](Primitive& prim,
         const glm::dvec3& rayOrigin,
         const glm::dvec3& rayDir,
         const glm::dvec3& intersectionPoint) {
        return prim.getIncidenceAngle_rad(rayOrigin, rayDir, intersectionPoint);
      },
      py::arg("rayOrigin"),
      py::arg("rayDir"),
      py::arg("intersectionPoint"))
    .def("ray_intersection",
         [](Primitive& prim,
            const glm::dvec3& rayOrigin,
            const glm::dvec3& rayDir) {
           const std::vector<double>& result =
             prim.getRayIntersection(rayOrigin, rayDir);
           return py::cast(result);
         })
    .def("ray_intersection_distance",
         [](Primitive& prim,
            const glm::dvec3& rayOrigin,
            const glm::dvec3& rayDir) {
           return prim.getRayIntersectionDistance(rayOrigin, rayDir);
         })

    .def("update", &Primitive::update)
    .def(
      "is_triangle",
      [](Primitive& prim) { return dynamic_cast<Triangle*>(&prim) != nullptr; })
    .def("is_AABB",
         [](Primitive& prim) { return dynamic_cast<AABB*>(&prim) != nullptr; })
    .def("is_voxel",
         [](Primitive& prim) { return dynamic_cast<Voxel*>(&prim) != nullptr; })
    .def("is_detailed_voxel", [](Primitive& prim) {
      return dynamic_cast<DetailedVoxel*>(&prim) != nullptr;
    });

  py::class_<DetailedVoxel> detailed_voxel(
    m, "DetailedVoxel", py::base<Primitive>());
  detailed_voxel.def(py::init<>())
    .def(py::init<double,
                  double,
                  double,
                  double,
                  std::vector<int>,
                  std::vector<double>>(),
         py::arg("x"),
         py::arg("y"),
         py::arg("z"),
         py::arg("halfVoxelSize"),
         py::arg("intValues"),
         py::arg("doubleValues"))

    .def_property(
      "nb_echos", &DetailedVoxel::getNbEchos, &DetailedVoxel::setNbEchos)
    .def_property("nb_sampling",
                  &DetailedVoxel::getNbSampling,
                  &DetailedVoxel::setNbSampling)
    .def_property_readonly("number_of_double_values",
                           &DetailedVoxel::getNumberOfDoubleValues)
    .def_property(
      "maxPad", &DetailedVoxel::getMaxPad, &DetailedVoxel::setMaxPad)
    .def(
      "doubleValue", &DetailedVoxel::getDoubleValue, "Get the value at index")
    .def("doubleValue",
         &DetailedVoxel::setDoubleValue,
         "Set the value at index",
         py::arg("index"),
         py::arg("value"));

  py::class_<AbstractDetector,
             AbstractDetectorWrap,
             std::shared_ptr<AbstractDetector>>
    abstract_detector(m, "AbstractDetector");
  abstract_detector
    .def(py::init<std::shared_ptr<Scanner>,
                  double,
                  double,
                  double,
                  std::shared_ptr<UnivarExprTreeNode<double>>>(),
         py::arg("scanner"),
         py::arg("accuracy_m"),
         py::arg("rangeMin_m"),
         py::arg("rangeMax_m") = std::numeric_limits<double>::max(),
         py::arg("errorDistanceExpr") = nullptr)

    .def_readwrite("accuracy", &AbstractDetector::cfg_device_accuracy_m)
    .def_readwrite("range_min", &AbstractDetector::cfg_device_rangeMin_m)
    .def_readwrite("range_max", &AbstractDetector::cfg_device_rangeMax_m)

    .def_property(
      "las_scale",
      [](AbstractDetector& self) {
        return self.getFMS()->write.getMeasurementWriterLasScale();
      },
      [](AbstractDetector& self, double lasScale) {
        self.getFMS()->write.setMeasurementWriterLasScale(lasScale);
      });

  py::class_<Triangle> triangle(m, "Triangle", py::base<Primitive>());
  triangle
    .def(py::init<Vertex, Vertex, Vertex>(),
         py::arg("v0"),
         py::arg("v1"),
         py::arg("v2"))
    .def("__str__", &Triangle::toString)
    .def("ray_intersection", &Triangle::getRayIntersection)
    .def_property_readonly("face_normal", &Triangle::getFaceNormal);

  py::class_<Vertex> vertex(m, "Vertex");
  vertex.def(py::init<>())
    .def(py::init<double, double, double>(),
         py::arg("x"),
         py::arg("y"),
         py::arg("z"))
    .def_property_readonly("position",
                           [](const Vertex& v) { return glm::dvec3(v.pos); })
    .def_property_readonly(
      "normal", [](const Vertex& v) { return glm::dvec3(v.normal); });

  py::class_<HeliosException>(m, "PyHeliosException")
    .def(py::init<std::string const&>(), py::arg("msg") = "");

  py::class_<Trajectory, std::shared_ptr<Trajectory>> trajectory(m,
                                                                 "Trajectory");
  trajectory.def(py::init<>())
    .def(py::init<double, glm::dvec3, double, double, double>())
    .def_readwrite("gps_time", &Trajectory::gpsTime)
    .def_property(
      "position",
      [](const Trajectory& t) { return t.position; },
      [](Trajectory& t, const glm::dvec3& pos) { t.position = pos; })
    .def_readwrite("roll", &Trajectory::roll)
    .def_readwrite("pitch", &Trajectory::pitch)
    .def_readwrite("yaw", &Trajectory::yaw);

  py::class_<Measurement, std::shared_ptr<Measurement>> measurement(
    m, "Measurement");
  measurement.def(py::init<>())
    .def(py::init<const Measurement&>())

    .def_readwrite("hit_object_id", &Measurement::hitObjectId)
    .def_readwrite("position", &Measurement::position)
    .def_readwrite("beam_direction", &Measurement::beamDirection)
    .def_readwrite("beam_origin", &Measurement::beamOrigin)
    .def_readwrite("distance", &Measurement::distance)
    .def_readwrite("intensity", &Measurement::intensity)
    .def_readwrite("echo_width", &Measurement::echo_width)
    .def_readwrite("return_number", &Measurement::returnNumber)
    .def_readwrite("pulse_return_number", &Measurement::pulseReturnNumber)
    .def_readwrite("fullwave_index", &Measurement::fullwaveIndex)
    .def_readwrite("classification", &Measurement::classification)
    .def_readwrite("gps_time", &Measurement::gpsTime);

  py::class_<NoiseSource<double>, NoiseSourceWrap<double>> noise_source(
    m, "NoiseSource");
  noise_source
    .def(py::init<>())

    .def_property("clip_min",
                  &NoiseSource<double>::getClipMin,
                  &NoiseSource<double>::setClipMin)
    .def_property("clip_max",
                  &NoiseSource<double>::getClipMax,
                  &NoiseSource<double>::setClipMax)
    .def_property("clip_enabled",
                  &NoiseSource<double>::isClipEnabled,
                  &NoiseSource<double>::setClipEnabled)
    .def_property_readonly("fixed_value_enabled",
                           &NoiseSource<double>::isFixedValueEnabled)

    .def_property("fixed_lifespan",
                  &NoiseSource<double>::getFixedLifespan,
                  &NoiseSource<double>::setFixedLifespan)
    .def_property("fixed_value_remaining_uses",
                  &NoiseSource<double>::getFixedValueRemainingUses,
                  &NoiseSource<double>::setFixedValueRemainingUses)
    .def("next", &NoiseSource<double>::next);

  py::class_<RandomnessGenerator<double>> randomness_generator(
    m, "RandomnessGenerator");
  randomness_generator.def(py::init<>())
    .def("compute_uniform_real_distribution",
         &RandomnessGenerator<double>::computeUniformRealDistribution)
    .def("uniform_real_distribution_next",
         &RandomnessGenerator<double>::uniformRealDistributionNext)
    .def("compute_normal_distribution",
         &RandomnessGenerator<double>::computeNormalDistribution)
    .def("normal_distribution_next",
         &RandomnessGenerator<double>::normalDistributionNext);

  py::class_<RaySceneIntersection> ray_scene_intersection(
    m, "RaySceneIntersection");
  ray_scene_intersection.def(py::init<>())
    .def(py::init<const RaySceneIntersection&>())
    .def_property(
      "primitive",
      [](RaySceneIntersection& self) { return self.prim; },
      [](RaySceneIntersection& self, Primitive* prim) { self.prim = prim; },
      py::return_value_policy::reference)
    .def_property(
      "point",
      [](RaySceneIntersection& self) { return self.point; },
      [](RaySceneIntersection& self, const glm::dvec3& point) {
        self.point = point;
      }) //????
    .def_property(
      "incidence_angle",
      [](RaySceneIntersection& self) { return self.incidenceAngle; },
      [](RaySceneIntersection& self, double angle) {
        self.incidenceAngle = angle;
      });

  py::class_<ScanningStrip, std::shared_ptr<ScanningStrip>> scanning_strip(
    m, "ScanningStrip");
  scanning_strip.def(py::init<const std::string&>())
    .def_property(
      "strip_id", &ScanningStrip::getStripId, &ScanningStrip::setStripId)
    .def_property_readonly("is_last_leg_in_strip",
                           &ScanningStrip::isLastLegInStrip)
    .def(
      "get_leg_ref",
      [](ScanningStrip& self, int serialId) -> Leg& {
        Leg* leg = self.getLeg(serialId);
        if (!leg)
          throw std::runtime_error("Leg not found");
        return *leg;
      },
      py::return_value_policy::reference)
    .def("has", py::overload_cast<int>(&ScanningStrip::has))
    .def("has", py::overload_cast<Leg&>(&ScanningStrip::has));

  py::class_<SimulationCycleCallback, SimulationCycleCallbackWrap>
    simulation_cycle_callback(m, "SimulationCycleCallback");
  simulation_cycle_callback.def(py::init<py::object>())
    .def("__call__",
         [](SimulationCycleCallback& callback,
            py::list measurements,
            py::list trajectories,
            const std::string& outpath) {
           py::gil_scoped_acquire acquire;

           // Convert Python lists to std::shared_ptr<std::vector<Measurement>>
           // and std::shared_ptr<std::vector<Trajectory>>
           auto measurements_vec = std::make_shared<std::vector<Measurement>>();
           for (auto item : measurements) {
             measurements_vec->push_back(item.cast<Measurement>());
           }

           auto trajectories_vec = std::make_shared<std::vector<Trajectory>>();
           for (auto item : trajectories) {
             trajectories_vec->push_back(item.cast<Trajectory>());
           }

           callback(*measurements_vec, *trajectories_vec, outpath);
         });

  py::class_<std::list<int>>(m, "IntegerList")
    .def(py::init<>())
    .def("__len__", &std::list<int>::size)
    .def("__getitem__",
         [](const std::list<int>& list, long index) {
           size_t idx = pybind11::handlePythonIndex(
             index, list.size()); // REWRITE TO HELIOS NAMESPACES
           auto it = list.begin();
           std::advance(it, idx);
           return *it;
         })
    .def("__setitem__",
         [](std::list<int>& list, long index, int value) {
           size_t idx = pybind11::handlePythonIndex(index, list.size());
           auto it = list.begin();
           std::advance(it, idx);
           *it = value;
         })
    .def("insert",
         [](std::list<int>& list, long index, int value) {
           size_t idx = pybind11::handlePythonIndex(index, list.size());
           auto it = list.begin();
           std::advance(it, idx);
           list.insert(it, value);
         })
    .def("erase",
         [](std::list<int>& list, long index) {
           size_t idx = pybind11::handlePythonIndex(index, list.size());
           auto it = list.begin();
           std::advance(it, idx);
           list.erase(it);
         })
    .def("__repr__", [](const std::list<int>& list) {
      std::ostringstream ss;
      ss << "[";
      bool first = true;
      for (const auto& item : list) {
        if (!first)
          ss << ", ";
        ss << item;
        first = false;
      }
      ss << "]";
      return ss.str();
    });

  py::class_<FWFSettings> fwf_settings(m, "FWFSettings");
  fwf_settings.def(py::init<>())
    .def(py::init<const FWFSettings&>(), py::arg("fwfSettings"))
    .def_readwrite("bin_size", &FWFSettings::binSize_ns)
    .def_readwrite("min_echo_width", &FWFSettings::minEchoWidth)
    .def_readwrite("peak_energy", &FWFSettings::peakEnergy)
    .def_readwrite("aperture_diameter", &FWFSettings::apertureDiameter)
    .def_readwrite("scanner_efficiency", &FWFSettings::scannerEfficiency)
    .def_readwrite("atmospheric_visibility",
                   &FWFSettings::atmosphericVisibility)
    .def_readwrite("scanner_wave_length", &FWFSettings::scannerWaveLength)
    .def_readwrite("beam_divergence_angle", &FWFSettings::beamDivergence_rad)
    .def_readwrite("pulse_length", &FWFSettings::pulseLength_ns)
    .def_readwrite("beam_sample_quality", &FWFSettings::beamSampleQuality)
    .def_readwrite("win_size", &FWFSettings::winSize_ns)
    .def_readwrite("max_fullwave_range", &FWFSettings::maxFullwaveRange_ns)
    .def("__str__", &FWFSettings::toString);

  py::class_<Rotation> rotation(m, "Rotation");
  rotation.def(py::init<>())
    .def(py::init<double, double, double, double, bool>(),
         py::arg("q0"),
         py::arg("q1"),
         py::arg("q2"),
         py::arg("q3"),
         py::arg("needsNormalization"))
    .def(py::init<glm::dvec3, double>(), py::arg("axis"), py::arg("angle"))
    .def(py::init<glm::dvec3, glm::dvec3>(), py::arg("u"), py::arg("v"))

    .def_property("q0", &Rotation::getQ0, &Rotation::setQ0)
    .def_property("q1", &Rotation::getQ1, &Rotation::setQ1)
    .def_property("q2", &Rotation::getQ2, &Rotation::setQ2)
    .def_property("q3", &Rotation::getQ3, &Rotation::setQ3)
    .def_property_readonly("axis", &Rotation::getAxis)
    .def_property_readonly("angle", &Rotation::getAngle);

  py::class_<ScannerHead> scanner_head(m, "ScannerHead");
  scanner_head
    .def(py::init<glm::dvec3, double>(),
         py::arg("headRotationAxis"),
         py::arg("headRotatePerSecMax_rad"))
    .def_property_readonly("mount_relative_attitude",
                           &ScannerHead::getMountRelativeAttitudeByReference)
    .def_property("rotate_per_sec_max",
                  &ScannerHead::getRotatePerSecMax,
                  &ScannerHead::setRotatePerSecMax)
    .def_property("rotate_per_sec",
                  &ScannerHead::getRotatePerSec_rad,
                  &ScannerHead::setRotatePerSec_rad)
    .def_property(
      "rotate_stop", &ScannerHead::getRotateStop, &ScannerHead::setRotateStop)
    .def_property("rotate_start",
                  &ScannerHead::getRotateStart,
                  &ScannerHead::setRotateStart)
    .def_property("rotate_range",
                  &ScannerHead::getRotateRange,
                  &ScannerHead::setRotateRange)

    .def_property("current_rotate_angle",
                  &ScannerHead::getRotateCurrent,
                  &ScannerHead::setCurrentRotateAngle_rad);

  py::class_<Material> material(m, "Material");
  material.def(py::init<>())
    .def(py::init<const Material&>(), py::arg("material"))

    .def_readwrite("name", &Material::name)
    .def_readwrite("is_ground", &Material::isGround)
    .def_readwrite("use_vertex_colors", &Material::useVertexColors)
    .def_readwrite("mat_file_path", &Material::matFilePath)
    .def_readwrite("map_Kd", &Material::map_Kd)
    .def_readwrite("reflectance", &Material::reflectance)
    .def_readwrite("specularity", &Material::specularity)
    .def_readwrite("specular_exponent", &Material::specularExponent)
    .def_readwrite("classification", &Material::classification)
    .def_readwrite("spectra", &Material::spectra)
    .def_property_readonly(
      "ka", [](const Material& mat) { return create_numpy_array(mat.ka); })
    .def_property_readonly(
      "kd", [](const Material& mat) { return create_numpy_array(mat.kd); })
    .def_property_readonly(
      "ks", [](const Material& mat) { return create_numpy_array(mat.ks); });

  py::class_<Survey, std::unique_ptr<Survey, py::nodelete>> survey(
    m, "Survey", py::module_local());
  survey.def(py::init<>())
    .def("calculate_length", &Survey::calculateLength)
    .def_property_readonly("length", &Survey::getLength)
    .def_property(
      "name",
      [](Survey& s) { return s.name; },
      [](Survey& s, const std::string& name) { s.name = name; })
    .def_property(
      "num_runs",
      [](Survey& s) { return s.numRuns; },
      [](Survey& s, int numRuns) { s.numRuns = numRuns; })
    .def_property(
      "sim_speed_factor",
      [](Survey& s) { return s.simSpeedFactor; },
      [](Survey& s, double simSpeedFactor) {
        s.simSpeedFactor = simSpeedFactor;
      });

  py::class_<Leg> leg(m, "Leg");
  leg.def(py::init<>())
    .def(py::init<double, int, std::shared_ptr<ScanningStrip>>(),
         py::arg("length"),
         py::arg("serialId"),
         py::arg("strip"))
    .def(py::init<Leg&>(), py::arg("leg"))
    .def_readwrite("scanner_settings", &Leg::mScannerSettings)
    .def_readwrite("platform_settings", &Leg::mPlatformSettings)

    .def_property("length", &Leg::getLength, &Leg::setLength)
    .def_property("serial_id", &Leg::getSerialId, &Leg::setSerialId)
    .def_property("strip", &Leg::getStrip, &Leg::setStrip)
    .def("belongs_to_strip", &Leg::isContainedInAStrip);

  py::class_<ScenePart> scene_part(m, "ScenePart");
  scene_part.def(py::init<>())
    .def(py::init<const ScenePart&, bool>(),
         py::arg("sp"),
         py::arg("shallowPrimitives") = false)

    .def_readwrite("origin", &ScenePart::mOrigin)
    .def_readwrite("rotation", &ScenePart::mRotation)
    .def_readwrite("scale", &ScenePart::mScale)
    .def_readwrite("bound", &ScenePart::bound)

    .def_property("centroid", &ScenePart::getCentroid, &ScenePart::setCentroid)
    .def_property("id", &ScenePart::getId, &ScenePart::setId)
    .def_property(
      "dyn_object_step",
      [](const ScenePart& self) -> size_t {
        if (self.getType() == ScenePart::ObjectType::DYN_OBJECT) {
          return dynamic_cast<const DynObject&>(self).getStepInterval();
        } else {
          throw std::runtime_error("ScenePart is not a DynObject.");
        }
      },
      [](ScenePart& self, size_t stepInterval) {
        if (self.getType() == ScenePart::ObjectType::DYN_OBJECT) {
          dynamic_cast<DynObject&>(self).setStepInterval(stepInterval);
        } else {
          throw std::runtime_error("ScenePart is not a DynObject.");
        }
      })

    .def_property(
      "observer_step",
      [](const ScenePart& self) -> size_t {
        if (self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT) {
          return dynamic_cast<const DynMovingObject&>(self)
            .getObserverStepInterval();
        } else {
          throw std::runtime_error("ScenePart is not a DynMovingObject.");
        }
      },
      [](ScenePart& self, size_t stepInterval) {
        if (self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT) {
          dynamic_cast<DynMovingObject&>(self).setObserverStepInterval(
            stepInterval);
        } else {
          throw std::runtime_error("ScenePart is not a DynMovingObject.");
        }
      })

    .def(
      "primitive",
      [](ScenePart& self, size_t index) -> Primitive* {
        if (index < self.mPrimitives.size()) {
          return self.mPrimitives[index];
        } else {
          throw std::out_of_range("Index out of range");
        }
      },
      py::return_value_policy::reference)
    .def_property_readonly(
      "num_primitives",
      [](const ScenePart& self) -> size_t { return self.mPrimitives.size(); })
    .def("isDynamicMovingObject",
         [](const ScenePart& self) -> bool {
           return self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT;
         })
    .def("compute_centroid",
         &ScenePart::computeCentroid,
         py::arg("computeBound") = false)
    .def("compute_centroid_w_bound",
         &ScenePart::computeCentroid,
         py::arg("computeBound") = true);

  py::enum_<ScenePart::ObjectType>(m, "ObjectType")
    .value("STATIC_OBJECT", ScenePart::STATIC_OBJECT)
    .value("DYN_OBJECT", ScenePart::DYN_OBJECT)
    .value("DYN_MOVING_OBJECT", ScenePart::DYN_MOVING_OBJECT)
    .export_values();

  py::enum_<ScenePart::PrimitiveType>(m, "PrimitiveType")
    .value("NONE", ScenePart::NONE)
    .value("TRIANGLE", ScenePart::TRIANGLE)
    .value("VOXEL", ScenePart::VOXEL)
    .export_values();

  py::class_<ScannerSettings, std::shared_ptr<ScannerSettings>>
    scanner_settings(m, "ScannerSettings");
  scanner_settings.def(py::init<>())
    .def(py::init<ScannerSettings*>(), py::arg("scannerSettings"))

    .def_readwrite("id", &ScannerSettings::id)
    .def_readwrite("is_active", &ScannerSettings::active)
    .def_readwrite("head_rotation", &ScannerSettings::headRotatePerSec_rad)
    .def_readwrite("rotation_start_angle",
                   &ScannerSettings::headRotateStart_rad)
    .def_readwrite("rotation_stop_angle", &ScannerSettings::headRotateStop_rad)
    .def_readwrite("pulse_frequency", &ScannerSettings::pulseFreq_Hz)
    .def_readwrite("scan_angle", &ScannerSettings::scanAngle_rad)
    .def_readwrite("min_vertical_angle", &ScannerSettings::verticalAngleMin_rad)
    .def_readwrite("max_vertical_angle", &ScannerSettings::verticalAngleMax_rad)
    .def_readwrite("scan_frequency", &ScannerSettings::scanFreq_Hz)
    .def_readwrite("beam_divergence_angle", &ScannerSettings::beamDivAngle)
    .def_readwrite("trajectory_time_interval",
                   &ScannerSettings::trajectoryTimeInterval)
    .def_readwrite("vertical_resolution",
                   &ScannerSettings::verticalResolution_rad)
    .def_readwrite("horizontal_resolution",
                   &ScannerSettings::horizontalResolution_rad)

    .def_property_readonly("base_template",
                           &ScannerSettings::getTemplate,
                           py::return_value_policy::reference)

    .def("cherry_pick",
         &ScannerSettings::cherryPick,
         py::arg("cherries"),
         py::arg("fields"),
         py::arg("templateFields") = nullptr)
    .def("fit_to_resolution", &ScannerSettings::fitToResolution)
    .def("has_template", &ScannerSettings::hasTemplate)
    .def("has_default_resolution", &ScannerSettings::hasDefaultResolution)
    .def("__repr__", &ScannerSettings::toString)
    .def("__str__", &ScannerSettings::toString);

  py::class_<PlatformSettings, std::shared_ptr<PlatformSettings>>
    platform_settings(m, "PlatformSettings");
  platform_settings.def(py::init<>())
    .def(py::init<PlatformSettings*>(), py::arg("platformSettings"))

    .def_readwrite("id", &PlatformSettings::id)
    .def_readwrite("x", &PlatformSettings::x)
    .def_readwrite("y", &PlatformSettings::y)
    .def_readwrite("z", &PlatformSettings::z)
    .def_readwrite("is_yaw_angle_specified",
                   &PlatformSettings::yawAtDepartureSpecified)
    .def_readwrite("yaw_angle", &PlatformSettings::yawAtDeparture)
    .def_readwrite("is_on_ground", &PlatformSettings::onGround)
    .def_readwrite("is_stop_and_turn", &PlatformSettings::stopAndTurn)
    .def_readwrite("is_smooth_turn", &PlatformSettings::smoothTurn)
    .def_readwrite("is_slowdown_enabled", &PlatformSettings::slowdownEnabled)
    .def_readwrite("speed_m_s", &PlatformSettings::movePerSec_m)

    .def_property_readonly("base_template",
                           &PlatformSettings::getTemplate,
                           py::return_value_policy::reference)
    .def_property_readonly("position", &PlatformSettings::getPosition)

    .def("cherryPick",
         &PlatformSettings::cherryPick,
         py::arg("cherries"),
         py::arg("fields"),
         py::arg("templateFields") = nullptr)

    .def("set_position",
         py::overload_cast<glm::dvec3>(&PlatformSettings::setPosition))
    .def(
      "set_position",
      py::overload_cast<double, double, double>(&PlatformSettings::setPosition))
    .def("has_template", &PlatformSettings::hasTemplate)
    .def("to_string", &PlatformSettings::toString);

  py::class_<Scene> scene(m, "Scene");
  scene.def(py::init<>())
    .def(py::init<Scene&>(), py::arg("scene"))

    .def_property("bbox", &Scene::getBBox, &Scene::setBBox)
    .def_property("bbox_crs", &Scene::getBBoxCRS, &Scene::setBBoxCRS)
    .def_property_readonly(
      "num_primitives",
      [](const ScenePart& self) -> size_t { return self.mPrimitives.size(); })
    .def_property_readonly(
      "AABB", &Scene::getAABB, py::return_value_policy::reference)

    .def_property_readonly("shift", &Scene::getShift)
    .def_property(
      "dyn_scene_step",
      [](const Scene& self) -> size_t {
        return dynamic_cast<const DynScene&>(self).getStepInterval();
      },
      [](Scene& self, size_t stepInterval) {
        dynamic_cast<DynScene&>(self).setStepInterval(stepInterval);
      })

    .def("scene_parts_size", [](Scene& self) { return self.parts.size(); })
    .def(
      "get_scene_part",
      [](Scene& self, size_t index) -> ScenePart& {
        if (index < self.parts.size()) {
          return *(self.parts[index]);
        } else {
          throw std::out_of_range("Index out of range");
        }
      },
      py::return_value_policy::reference)
    .def("ground_point_at",
         [](Scene& self, glm::dvec3 point) {
           return self.getGroundPointAt(point);
         })
    .def("finalize_loading", &Scene::finalizeLoading, py::arg("safe") = false)
    .def("write_object", &Scene::writeObject)
    .def("translate",
         [](Scene& scene, double x, double y, double z) {
           glm::dvec3 shift(x, y, z);
           for (Primitive* p : scene.primitives) {
             p->translate(shift);
           }
         })
    .def(
      "new_triangle",
      [](Scene& scene) {
        Vertex v;
        v.pos[0] = 0.0;
        v.pos[1] = 0.0;
        v.pos[2] = 0.0;
        Triangle* tri = new Triangle(v, v, v);
        scene.primitives.push_back(tri);
        return tri;
      },
      py::return_value_policy::reference)
    .def(
      "new_detailed_voxel",
      [](Scene& scene) {
        std::vector<int> vi({ 0, 0 });
        std::vector<double> vd({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        DetailedVoxel* dv = new DetailedVoxel(0.0, 0.0, 0.0, 0.5, vi, vd);
        scene.primitives.push_back(dv);
        return dv;
      },
      py::return_value_policy::reference)
    .def(
      "primitive",
      [](Scene& scene, size_t index) -> Primitive* {
        if (index < scene.primitives.size()) {
          return scene.primitives[index];
        } else {
          throw std::out_of_range("Index out of range");
        }
      },
      py::return_value_policy::reference)

    .def("intersection_min_max",
         py::overload_cast<std::vector<double> const&,
                           glm::dvec3 const&,
                           glm::dvec3 const&,
                           bool const>(&Scene::getIntersection, py::const_),
         py::arg("tMinMax"),
         py::arg("rayOrigin"),
         py::arg("rayDir"),
         py::arg("groundOnly"));

  py::class_<Platform> platform(m, "Platform");
  platform.def(py::init<>())
    .def(py::init<Platform&>(), py::arg("platform"))

    .def_readwrite("last_check_z", &Platform::lastCheckZ)
    .def_readwrite("dmax", &Platform::dmax)
    .def_readwrite("is_orientation_on_leg_init",
                   &Platform::mSetOrientationOnLegInit)

    .def_readwrite("is_on_ground", &Platform::onGround)
    .def_readwrite("is_stop_and_turn", &Platform::stopAndTurn)
    .def_readwrite("settings_speed_m_s", &Platform::cfg_settings_movePerSec_m)
    .def_readwrite("is_slowdown_enabled", &Platform::slowdownEnabled)
    .def_readwrite("is_smooth_turn", &Platform::smoothTurn)

    .def_property_readonly("device_relative_position",
                           [](const Platform& self) {
                             return self.cfg_device_relativeMountPosition;
                           })
    .def_property_readonly("device_relative_attitude",
                           [](const Platform& self) {
                             return self.cfg_device_relativeMountAttitude;
                           })
    .def_property_readonly(
      "position_x_noise_source",
      [](const Platform& self) { return self.positionXNoiseSource; })
    .def_property_readonly(
      "position_y_noise_source",
      [](const Platform& self) { return self.positionYNoiseSource; })
    .def_property_readonly(
      "position_z_noise_source",
      [](const Platform& self) { return self.positionZNoiseSource; })
    .def_property_readonly(
      "attitude_x_noise_source",
      [](const Platform& self) { return self.attitudeXNoiseSource; })
    .def_property_readonly(
      "attitude_y_noise_source",
      [](const Platform& self) { return self.attitudeYNoiseSource; })
    .def_property_readonly(
      "attitude_z_noise_source",
      [](const Platform& self) { return self.attitudeZNoiseSource; })
    .def_property_readonly(
      "target_waypoint",
      [](const Platform& self) { return self.targetWaypoint; })
    .def_property_readonly(
      "last_ground_check",
      [](const Platform& self) { return self.lastGroundCheck; })

    .def_property_readonly("position",
                           &Platform::getPosition) //, &Platform::setPosition)
    .def_property_readonly("attitude",
                           &Platform::getAttitude) //, &Platform::setAttitude)
    .def_property_readonly("absolute_mount_position",
                           &Platform::getAbsoluteMountPosition)
    .def_property_readonly("absolute_mount_attitude",
                           &Platform::getAbsoluteMountAttitude)

    .def_property_readonly(
      "cached_dir_current",
      [](const Platform& self) { return self.cached_dir_current; })
    .def_property_readonly(
      "cached_dir_current_xy",
      [](const Platform& self) { return self.cached_dir_current_xy; })
    .def_property_readonly(
      "cached_vector_to_target",
      [](const Platform& self) { return self.cached_vectorToTarget; })
    .def_property_readonly(
      "cached_vector_to_target_xy",
      [](const Platform& self) { return self.cached_vectorToTarget_xy; });

  py::class_<ScanningDevice> scanning_device(m, "ScanningDevice");
  scanning_device.def(py::init<ScanningDevice const&>())
    .def(py::init<size_t,
                  std::string,
                  double,
                  glm::dvec3,
                  Rotation,
                  std::list<int>,
                  double,
                  double,
                  double,
                  double,
                  double,
                  double,
                  double,
                  std::shared_ptr<UnivarExprTreeNode<double>>>())

    .def_readwrite("cached_dr2", &ScanningDevice::cached_Dr2)
    .def_readwrite("cached_bt2", &ScanningDevice::cached_Bt2)
    .def_readwrite("cached_subrayRotation",
                   &ScanningDevice::cached_subrayRotation)
    .def_readwrite("cached_subray_divergence",
                   &ScanningDevice::cached_subrayDivergenceAngle_rad)
    .def_readwrite("cached_subray_radius_step",
                   &ScanningDevice::cached_subrayRadiusStep)

    .def_property("energy_model",
                  &ScanningDevice::getEnergyModel,
                  &ScanningDevice::setEnergyModel)

    .def_property("FWF_settings",
                  &ScanningDevice::getFWFSettings,
                  &ScanningDevice::setFWFSettings)
    .def_property("received_energy_min",
                  &ScanningDevice::getReceivedEnergyMin,
                  &ScanningDevice::setReceivedEnergyMin)
    .def_property("last_pulse_was_hit",
                  &ScanningDevice::lastPulseWasHit,
                  &ScanningDevice::setLastPulseWasHit)

    .def("set_head_relative_emitter_position",
         &ScanningDevice::setHeadRelativeEmitterPosition)
    .def("set_head_relative_emitter_attitude",
         &ScanningDevice::setHeadRelativeEmitterAttitude)
    .def("prepare_simulation",
         &ScanningDevice::prepareSimulation,
         py::arg("legacyEnergyModel") = false)
    .def("configure_beam", &ScanningDevice::configureBeam)
    .def("calcAtmosphericAttenuation",
         &ScanningDevice::calcAtmosphericAttenuation)
    .def("calcRaysNumber", &ScanningDevice::calcRaysNumber)
    .def("doSimStep", &ScanningDevice::doSimStep)
    .def("calcAbsoluteBeamAttitude", &ScanningDevice::calcAbsoluteBeamAttitude)
    .def("calc_exact_absolute_beam_attitude",
         &ScanningDevice::calcExactAbsoluteBeamAttitude)
    .def("computeSubrays", &ScanningDevice::computeSubrays)
    .def("initializeFullWaveform", &ScanningDevice::initializeFullWaveform)
    .def("calcIntensity",
         py::overload_cast<double, double, const Material&, int>(
           &ScanningDevice::calcIntensity, py::const_))
    .def("calcIntensity",
         py::overload_cast<double, double, int>(&ScanningDevice::calcIntensity,
                                                py::const_))

    .def("eval_range_error_expression",
         &ScanningDevice::evalRangeErrorExpression);

  py::class_<Scanner, ScannerWrap, std::shared_ptr<Scanner>> scanner(m,
                                                                     "Scanner");
  scanner.def(py::init<>())
    .def(py::init<Scanner&>(), py::arg("scanner"))

    .def("initialize_sequential_generators",
         &Scanner::initializeSequentialGenerators)
    .def("build_scanning_pulse_process",
         &Scanner::buildScanningPulseProcess,
         py::arg("parallelization_strategy"),
         py::arg("dropper"),
         py::arg("pool"))
    .def("apply_settings",
         py::overload_cast<std::shared_ptr<ScannerSettings>>(
           &Scanner::applySettings))
    .def("apply_settings",
         py::overload_cast<std::shared_ptr<ScannerSettings>, size_t>(
           &Scanner::applySettings))
    .def("retrieve_current_settings",
         py::overload_cast<>(&Scanner::retrieveCurrentSettings))
    .def("retrieve_current_settings",
         py::overload_cast<size_t>(&Scanner::retrieveCurrentSettings))
    .def("apply_settings_FWF",
         py::overload_cast<FWFSettings>(&Scanner::applySettingsFWF))
    .def("apply_settings_FWF",
         py::overload_cast<FWFSettings, size_t>(&Scanner::applySettingsFWF))
    .def("do_sim_step",
         &Scanner::doSimStep,
         py::arg("legIndex"),
         py::arg("currentGpsTime"))
    .def("to_string", &Scanner::toString)
    .def("calc_rays_number", py::overload_cast<>(&Scanner::calcRaysNumber))
    .def("calc_rays_number",
         py::overload_cast<size_t>(&Scanner::calcRaysNumber))
    .def("prepare_discretization",
         py::overload_cast<>(&Scanner::prepareDiscretization))
    .def("prepare_discretization",
         py::overload_cast<size_t>(&Scanner::prepareDiscretization))

    .def("calc_atmospheric_attenuation",
         py::overload_cast<>(&Scanner::calcAtmosphericAttenuation, py::const_))
    .def("calc_atmospheric_attenuation",
         py::overload_cast<size_t>(&Scanner::calcAtmosphericAttenuation,
                                   py::const_))
    .def("check_max_NOR", py::overload_cast<int>(&Scanner::checkMaxNOR))
    .def("check_max_NOR", py::overload_cast<int, size_t>(&Scanner::checkMaxNOR))
    .def("calc_absolute_beam_attitude",
         py::overload_cast<size_t>(&Scanner::calcAbsoluteBeamAttitude))
    .def("calc_absolute_beam_attitude",
         py::overload_cast<>(&Scanner::calcAbsoluteBeamAttitude))
    .def("handle_sim_step_noise", &Scanner::handleSimStepNoise)
    .def("on_leg_complete", &Scanner::onLegComplete)
    .def("on_simulation_finished", &Scanner::onSimulationFinished)
    .def("handle_trajectory_output", &Scanner::handleTrajectoryOutput)
    .def("track_output_path", &Scanner::trackOutputPath)

    .def("specific_current_pulse_number",
         py::overload_cast<size_t>(&Scanner::getCurrentPulseNumber, py::const_),
         py::arg("index"))
    .def("get_specific_num_rays",
         py::overload_cast<size_t>(&Scanner::getNumRays, py::const_))
    .def("set_specific_num_rays",
         py::overload_cast<int, size_t>(&Scanner::setNumRays))
    .def("get_specific_pulse_length",
         py::overload_cast<size_t>(&Scanner::getPulseLength_ns, py::const_),
         py::arg("index"))
    .def("set_specific_pulse_length",
         py::overload_cast<double, size_t>(&Scanner::setPulseLength_ns),
         py::arg("value"),
         py::arg("index"))
    .def("get_specific_last_pulse_was_hit",
         py::overload_cast<size_t>(&Scanner::lastPulseWasHit, py::const_),
         py::arg("index"))
    .def("set_specific_last_pulse_was_hit",
         py::overload_cast<bool, size_t>(&Scanner::setLastPulseWasHit),
         py::arg("value"),
         py::arg("index"))
    .def("get_specific_beam_divergence",
         py::overload_cast<size_t>(&Scanner::getBeamDivergence, py::const_),
         py::arg("index"))
    .def("set_specific_beam_divergence",
         py::overload_cast<double, size_t>(&Scanner::setBeamDivergence),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_average_power",
         py::overload_cast<size_t>(&Scanner::getAveragePower, py::const_),
         py::arg("index"))
    .def("set_specific_average_power",
         py::overload_cast<double, size_t>(&Scanner::setAveragePower),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_beam_quality",
         py::overload_cast<size_t>(&Scanner::getBeamQuality, py::const_),
         py::arg("index"))
    .def("set_specific_beam_quality",
         py::overload_cast<double, size_t>(&Scanner::setBeamQuality),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_efficiency",
         py::overload_cast<size_t>(&Scanner::getEfficiency, py::const_),
         py::arg("index"))
    .def("set_specific_efficiency",
         py::overload_cast<double, size_t>(&Scanner::setEfficiency),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_receiver_diameter",
         py::overload_cast<size_t>(&Scanner::getReceiverDiameter, py::const_),
         py::arg("index"))
    .def("set_specific_receiver_diameter",
         py::overload_cast<double, size_t>(&Scanner::setReceiverDiameter),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_visibility",
         py::overload_cast<size_t>(&Scanner::getVisibility, py::const_),
         py::arg("index"))
    .def("set_specific_visibility",
         py::overload_cast<double, size_t>(&Scanner::setVisibility),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_wavelength",
         py::overload_cast<size_t>(&Scanner::getWavelength, py::const_),
         py::arg("index"))
    .def("set_specific_wavelength",
         py::overload_cast<double, size_t>(&Scanner::setWavelength),
         py::arg("value"),
         py::arg("index"))
    .def(
      "get_specific_atmospheric_extinction",
      py::overload_cast<size_t>(&Scanner::getAtmosphericExtinction, py::const_),
      py::arg("index"))
    .def("set_specific_atmospheric_extinction",
         py::overload_cast<double, size_t>(&Scanner::setAtmosphericExtinction),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_beam_waist_radius",
         py::overload_cast<size_t>(&Scanner::getBeamWaistRadius, py::const_),
         py::arg("index"))
    .def("set_specific_beam_waist_radius",
         py::overload_cast<double, size_t>(&Scanner::setBeamWaistRadius),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_max_nor",
         py::overload_cast<size_t>(&Scanner::getMaxNOR, py::const_),
         py::arg("index"))
    .def("set_specific_max_nor",
         py::overload_cast<int, size_t>(&Scanner::setMaxNOR),
         py::arg("value"),
         py::arg("index"))

    .def("get_head_relative_emitter_attitude",
         py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterAttitude,
                                   py::const_),
         py::arg("index"))
    .def("set_head_relative_emitter_attitude",
         py::overload_cast<const Rotation&, size_t>(
           &Scanner::setHeadRelativeEmitterAttitude),
         py::arg("value"),
         py::arg("index"))

    .def("get_head_relative_emitter_position",
         py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterPosition,
                                   py::const_),
         py::arg("index"))
    .def("set_head_relative_emitter_position",
         py::overload_cast<const glm::dvec3&, size_t>(
           &Scanner::setHeadRelativeEmitterPosition),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_bt2",
         py::overload_cast<size_t>(&Scanner::getBt2, py::const_),
         py::arg("index"))
    .def("set_specific_bt2",
         py::overload_cast<double, size_t>(&Scanner::setBt2),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_dr2",
         py::overload_cast<size_t>(&Scanner::getDr2, py::const_),
         py::arg("index"))
    .def("set_specific_dr2",
         py::overload_cast<double, size_t>(&Scanner::setDr2),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_device_id",
         py::overload_cast<size_t>(&Scanner::getDeviceId, py::const_),
         py::arg("index"))
    .def("set_specific_device_id",
         py::overload_cast<std::string, size_t>(&Scanner::setDeviceId),
         py::arg("value"),
         py::arg("index"))
    .def("get_specific_detector",
         py::overload_cast<size_t>(&Scanner::getDetector),
         py::arg("index"))
    .def("set_specific_detector",
         py::overload_cast<std::shared_ptr<AbstractDetector>, size_t>(
           &Scanner::setDetector),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_supported_pulse_freqs_hz",
         py::overload_cast<size_t>(&Scanner::getSupportedPulseFreqs_Hz),
         py::arg("index"))
    .def("set_specific_supported_pulse_freqs_hz",
         py::overload_cast<std::list<int>&, size_t>(
           &Scanner::setSupportedPulseFreqs_Hz),
         py::arg("value"),
         py::arg("index"))

    .def(
      "get_head_relative_emitter_position_by_ref",
      py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterPositionByRef),
      py::arg("index"))
    .def(
      "get__head_relative_emitter_attitude_by_ref",
      py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterAttitudeByRef),
      py::arg("index"))

    .def(
      "get_specific_num_time_bins",
      [](Scanner& self, size_t idx) { return self.getNumTimeBins(idx); },
      py::arg("index"))
    .def(
      "set_specific_num_time_bins",
      [](Scanner& self, int numTimeBins, size_t idx) {
        self.setNumTimeBins(numTimeBins, idx);
      },
      py::arg("value"),
      py::arg("index"))

    .def("get_specific_peak_intensity_index",
         py::overload_cast<size_t>(&Scanner::getPeakIntensityIndex, py::const_),
         py::arg("index"))
    .def("set_specific_peak_intensity_index",
         py::overload_cast<int, size_t>(&Scanner::setPeakIntensityIndex),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_scanner_head",
         py::overload_cast<size_t>(&Scanner::getScannerHead),
         py::arg("index"))
    .def("set_specific_scanner_head",
         py::overload_cast<std::shared_ptr<ScannerHead>, size_t>(
           &Scanner::setScannerHead),
         py::arg("value"),
         py::arg("index"))

    .def("get_specific_beam_deflector",
         py::overload_cast<size_t>(&Scanner::getBeamDeflector),
         py::arg("index"))
    .def("set_specific_beam_deflector",
         py::overload_cast<std::shared_ptr<AbstractBeamDeflector>, size_t>(
           &Scanner::setBeamDeflector),
         py::arg("value"),
         py::arg("index"))

    .def_property_readonly(
      "current_pulse_number",
      py::overload_cast<>(&Scanner::getCurrentPulseNumber, py::const_))
    .def_property("num_rays",
                  py::overload_cast<>(&Scanner::getNumRays, py::const_),
                  py::overload_cast<int>(&Scanner::setNumRays))
    .def_property("pulse_length",
                  py::overload_cast<>(&Scanner::getPulseLength_ns, py::const_),
                  py::overload_cast<double>(&Scanner::setPulseLength_ns))
    .def_property("last_pulse_was_hit",
                  py::overload_cast<>(&Scanner::lastPulseWasHit, py::const_),
                  py::overload_cast<bool>(&Scanner::setLastPulseWasHit))
    .def_property("beam_divergence",
                  py::overload_cast<>(&Scanner::getBeamDivergence, py::const_),
                  py::overload_cast<double>(&Scanner::setBeamDivergence))
    .def_property("average_power",
                  py::overload_cast<>(&Scanner::getAveragePower, py::const_),
                  py::overload_cast<double>(&Scanner::setAveragePower))
    .def_property("beam_quality",
                  py::overload_cast<>(&Scanner::getBeamQuality, py::const_),
                  py::overload_cast<double>(&Scanner::setBeamQuality))
    .def_property("efficiency",
                  py::overload_cast<>(&Scanner::getEfficiency, py::const_),
                  py::overload_cast<double>(&Scanner::setEfficiency))
    .def_property(
      "receiver_diameter",
      py::overload_cast<>(&Scanner::getReceiverDiameter, py::const_),
      py::overload_cast<double>(&Scanner::setReceiverDiameter))
    .def_property("visibility",
                  py::overload_cast<>(&Scanner::getVisibility, py::const_),
                  py::overload_cast<double>(&Scanner::setVisibility))
    .def_property("wavelength",
                  py::overload_cast<>(&Scanner::getWavelength, py::const_),
                  py::overload_cast<double>(&Scanner::setWavelength))
    .def_property(
      "atmospheric_extinction",
      py::overload_cast<>(&Scanner::getAtmosphericExtinction, py::const_),
      py::overload_cast<double>(&Scanner::setAtmosphericExtinction))

    .def_property("beam_waist_radius",
                  py::overload_cast<>(&Scanner::getBeamWaistRadius, py::const_),
                  py::overload_cast<double>(&Scanner::setBeamWaistRadius))

    .def_property("max_nor",
                  py::overload_cast<>(&Scanner::getMaxNOR, py::const_),
                  py::overload_cast<int>(&Scanner::setMaxNOR))

    .def_property("bt2",
                  py::overload_cast<>(&Scanner::getBt2, py::const_),
                  py::overload_cast<double>(&Scanner::setBt2))

    .def_property("dr2",
                  py::overload_cast<>(&Scanner::getDr2, py::const_),
                  py::overload_cast<double>(&Scanner::setDr2))

    .def_property("device_id",
                  py::overload_cast<>(&Scanner::getDeviceId, py::const_),
                  py::overload_cast<std::string>(&Scanner::setDeviceId))

    .def_property(
      "scanner_head",
      py::overload_cast<>(&Scanner::getScannerHead),
      py::overload_cast<std::shared_ptr<ScannerHead>>(&Scanner::setScannerHead))

    .def_property("beam_deflector",
                  py::overload_cast<>(&Scanner::getBeamDeflector),
                  py::overload_cast<std::shared_ptr<AbstractBeamDeflector>>(
                    &Scanner::setBeamDeflector))

    .def_property("detector",
                  py::overload_cast<>(&Scanner::getDetector),
                  py::overload_cast<std::shared_ptr<AbstractDetector>>(
                    &Scanner::setDetector))

    .def_property(
      "supported_pulse_freqs_hz",
      py::overload_cast<>(&Scanner::getSupportedPulseFreqs_Hz),
      py::overload_cast<std::list<int>&>(&Scanner::setSupportedPulseFreqs_Hz))

    .def_property("num_time_bins",
                  py::overload_cast<>(&Scanner::getNumTimeBins, py::const_),
                  py::overload_cast<int>(&Scanner::setNumTimeBins))

    .def_property(
      "peak_intensity_index",
      py::overload_cast<>(&Scanner::getPeakIntensityIndex, py::const_),
      py::overload_cast<int>(&Scanner::setPeakIntensityIndex))

    .def_property(
      "pulse_frequency", &Scanner::getPulseFreq_Hz, &Scanner::setPulseFreq_Hz)
    .def_property("is_state_active", &Scanner::isActive, &Scanner::setActive)
    .def_property(
      "write_wave_form", &Scanner::isWriteWaveform, &Scanner::setWriteWaveform)
    .def_property(
      "calc_echowidth", &Scanner::isCalcEchowidth, &Scanner::setCalcEchowidth)
    .def_property(
      "full_wave_noise", &Scanner::isFullWaveNoise, &Scanner::setFullWaveNoise)
    .def_property("platform_noise_disabled",
                  &Scanner::isPlatformNoiseDisabled,
                  &Scanner::setPlatformNoiseDisabled)
    .def_property("fixed_incidence_angle",
                  &Scanner::isFixedIncidenceAngle,
                  &Scanner::setFixedIncidenceAngle)
    .def_property("id", &Scanner::getScannerId, &Scanner::setScannerId)
    .def_property(
      "FWF_settings",
      py::overload_cast<>(&Scanner::getFWFSettings),
      py::overload_cast<const FWFSettings&>(&Scanner::setFWFSettings))
    .def_property_readonly("num_devices", &Scanner::getNumDevices)
    .def_property_readonly("time_wave",
                           py::overload_cast<>(&Scanner::getTimeWave))

    .def_readwrite("intersection_handling_noise_source",
                   &Scanner::intersectionHandlingNoiseSource)
    .def_readwrite("rand_gen1", &Scanner::randGen1)
    .def_readwrite("rand_gen2", &Scanner::randGen2)
    .def_readwrite("trajectory_time_interval",
                   &Scanner::trajectoryTimeInterval_ns);

  py::class_<PyHeliosSimulation> helios_simulation(m, "Simulation");
  helios_simulation.def(py::init<>())
    .def(py::init<std::string,
                  std::vector<std::string>,
                  std::string,
                  size_t,
                  bool,
                  bool,
                  bool,
                  bool,
                  int,
                  size_t,
                  size_t,
                  int,
                  int,
                  int>(),
         py::arg("surveyPath"),
         py::arg("assetsPath"),
         py::arg("outputPath") = "output/",
         py::arg("numThreads") = 0,
         py::arg("lasOutput") = false,
         py::arg("las10") = false,
         py::arg("zipOutput") = false,
         py::arg("splitByChannel") = false,
         py::arg("kdtFactory") = 4,
         py::arg("kdtJobs") = 0,
         py::arg("kdtSAHLossNodes") = 32,
         py::arg("parallelizationStrategy") = 1,
         py::arg("chunkSize") = 32,
         py::arg("warehouseFactor") = 1)
    .def("start", &PyHeliosSimulation::start)
    .def("pause", &PyHeliosSimulation::pause)
    .def("stop", &PyHeliosSimulation::stop)
    .def("resume", &PyHeliosSimulation::resume)
    .def("join", &PyHeliosSimulation::join)
    .def("load_survey",
         &PyHeliosSimulation::loadSurvey,
         py::arg("legNoiseDisabled") = false,
         py::arg("rebuildScene") = false,
         py::arg("writeWaveform") = false,
         py::arg("calcEchowidth") = false,
         py::arg("fullWaveNoise") = false,
         py::arg("platformNoiseDisabled") = true)
    .def("add_rotate_filter",
         &PyHeliosSimulation::addRotateFilter,
         py::arg("q0"),
         py::arg("q1"),
         py::arg("q2"),
         py::arg("q3"),
         py::arg("partId"))
    .def("add_scale_filter",
         &PyHeliosSimulation::addScaleFilter,
         py::arg("scaleFactor"),
         py::arg("partId"))
    .def("add_translate_filter",
         &PyHeliosSimulation::addTranslateFilter,
         py::arg("x"),
         py::arg("y"),
         py::arg("z"),
         py::arg("partId"))
    .def("copy", &PyHeliosSimulation::copy)
    .def("callback", &PyHeliosSimulation::setCallback)
    .def("assoc_leg_with_scanning_strip",
         &PyHeliosSimulation::assocLegWithScanningStrip)
    .def("remove_leg", &PyHeliosSimulation::removeLeg)
    .def("new_leg",
         &PyHeliosSimulation::newLeg,
         py::return_value_policy::reference)
    .def("new_scanning_strip", &PyHeliosSimulation::newScanningStrip)
    .def("clear_callback", &PyHeliosSimulation::clearCallback)
    .def("get_leg",
         &PyHeliosSimulation::getLeg,
         py::return_value_policy::reference)

    .def_property(
      "final_output",
      [](const PyHeliosSimulation& self) { return self.finalOutput; },
      [](PyHeliosSimulation& self, bool value) { self.finalOutput = value; })
    .def_property(
      "legacy_energy_model",
      [](const PyHeliosSimulation& self) { return self.legacyEnergyModel; },
      [](PyHeliosSimulation& self, bool value) {
        self.legacyEnergyModel = value;
      })
    .def_property(
      "export_to_file",
      [](const PyHeliosSimulation& self) { return self.exportToFile; },
      [](PyHeliosSimulation& self, bool value) { self.exportToFile = value; })

    .def_property_readonly("is_started", &PyHeliosSimulation::isStarted)
    .def_property_readonly("is_paused", &PyHeliosSimulation::isPaused)
    .def_property_readonly("is_stopped", &PyHeliosSimulation::isStopped)
    .def_property_readonly("is_finished", &PyHeliosSimulation::isFinished)
    .def_property_readonly("is_running", &PyHeliosSimulation::isRunning)
    .def_property_readonly("survey_path", &PyHeliosSimulation::getSurveyPath)
    .def_property_readonly("assets_path", &PyHeliosSimulation::getAssetsPath)
    .def_property_readonly("survey", &PyHeliosSimulation::getSurvey)
    .def_property_readonly("scanner", &PyHeliosSimulation::getScanner)
    .def_property_readonly("platform", &PyHeliosSimulation::getPlatform)
    .def_property_readonly("scene", &PyHeliosSimulation::getScene)
    .def_property_readonly("num_legs", &PyHeliosSimulation::getNumLegs)

    .def_property("num_threads",
                  &PyHeliosSimulation::getNumThreads,
                  &PyHeliosSimulation::setNumThreads)
    .def_property("callback_frequency",
                  &PyHeliosSimulation::getCallbackFrequency,
                  &PyHeliosSimulation::setCallbackFrequency)
    .def_property("simulation_frequency",
                  &PyHeliosSimulation::getSimFrequency,
                  &PyHeliosSimulation::setSimFrequency)
    .def_property("dyn_scene_step",
                  &PyHeliosSimulation::getDynSceneStep,
                  &PyHeliosSimulation::setDynSceneStep)
    .def_property("fixed_gps_time_start",
                  &PyHeliosSimulation::getFixedGpsTimeStart,
                  &PyHeliosSimulation::setFixedGpsTimeStart)
    .def_property("las_output",
                  &PyHeliosSimulation::getLasOutput,
                  &PyHeliosSimulation::setLasOutput)
    .def_property(
      "las10", &PyHeliosSimulation::getLas10, &PyHeliosSimulation::setLas10)
    .def_property("zip_output",
                  &PyHeliosSimulation::getZipOutput,
                  &PyHeliosSimulation::setZipOutput)
    .def_property("split_by_channel",
                  &PyHeliosSimulation::getSplitByChannel,
                  &PyHeliosSimulation::setSplitByChannel)
    .def_property("las_scale",
                  &PyHeliosSimulation::getLasScale,
                  &PyHeliosSimulation::setLasScale)
    .def_property("kdt_factory",
                  &PyHeliosSimulation::getKDTFactory,
                  &PyHeliosSimulation::setKDTFactory)
    .def_property("kdt_jobs",
                  &PyHeliosSimulation::getKDTJobs,
                  &PyHeliosSimulation::setKDTJobs)
    .def_property("kdt_SAH_loss_nodes",
                  &PyHeliosSimulation::getKDTSAHLossNodes,
                  &PyHeliosSimulation::setKDTSAHLossNodes)
    .def_property("parallelization_strategy",
                  &PyHeliosSimulation::getParallelizationStrategy,
                  &PyHeliosSimulation::setParallelizationStrategy)
    .def_property("chunk_size",
                  &PyHeliosSimulation::getChunkSize,
                  &PyHeliosSimulation::setChunkSize)
    .def_property("warehouse_factor",
                  &PyHeliosSimulation::getWarehouseFactor,
                  &PyHeliosSimulation::setWarehouseFactor);

  m.def("calc_time_propagation",
        &calcTimePropagation,
        py::arg("timeWave"),
        py::arg("numBins"),
        py::arg("scanner"));
}
}
