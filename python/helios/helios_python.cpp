#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp> 

#include <gdal_priv.h>

#include <sim/comps/Survey.h>
#include <sim/comps/Leg.h>
#include <assetloading/ScenePart.h>
#include <scanner/ScannerSettings.h>
#include <platform/PlatformSettings.h>
#include <scene/Scene.h>
#include <platform/Platform.h>
#include <platform/MovingPlatform.h>
#include <platform/SimplePhysicsPlatform.h>
#include <platform/LinearPathPlatform.h>
#include <platform/GroundVehiclePlatform.h>
#include <platform/HelicopterPlatform.h>
#include <scanner/Scanner.h>
#include <scanner/Trajectory.h>
#include <platform/trajectory/TrajectorySettings.h>
#include <scanner/FWFSettings.h>
#include <scanner/Measurement.h>


bool    logging::LOGGING_SHOW_TRACE,    logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO,     logging::LOGGING_SHOW_TIME,
        logging::LOGGING_SHOW_WARN,     logging::LOGGING_SHOW_ERR;


namespace py = pybind11;

using VectorString = std::vector<std::string>;

PYBIND11_MAKE_OPAQUE(std::vector<std::string>);
PYBIND11_MAKE_OPAQUE(std::vector<Measurement>);
PYBIND11_MAKE_OPAQUE(std::vector<Trajectory>);

#include <scanner/beamDeflector/AbstractBeamDeflector.h>
#include <scanner/beamDeflector/OscillatingMirrorBeamDeflector.h>
#include <scanner/beamDeflector/ConicBeamDeflector.h>
#include <scanner/beamDeflector/FiberArrayBeamDeflector.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>
#include <scanner/beamDeflector/RisleyBeamDeflector.h>
#include <scene/primitives/Primitive.h>
#include <scene/primitives/Triangle.h>
#include <scene/primitives/Vertex.h>
#include <scene/primitives/AABB.h>
#include <scene/primitives/DetailedVoxel.h>
#include <scanner/detector/AbstractDetector.h>
#include <noise/NoiseSource.h>
#include <noise/RandomnessGenerator.h>
#include <scene/RaySceneIntersection.h>
#include <sim/comps/SimulationCycleCallback.h>
#include <scene/Material.h>
#include <maths/Rotation.h>
#include <maths/WaveMaths.h>
#include <maths/model/BaseEnergyModel.h>
#include <maths/model/ImprovedEnergyModel.h>
#include <maths/model/EnergyModel.h>

#include <DynObject.h>  
#include <DynMovingObject.h>
#include <DynScene.h>  

#include <sim/core/SurveyPlayback.h>
#include <scanner/ScanningDevice.h>
#include <scanner/ScannerHead.h>
#include <scanner/EvalScannerHead.h>
#include <scanner/ScanningPulseProcess.h>
#include <scanner/BuddingScanningPulseProcess.h>
#include <scanner/WarehouseScanningPulseProcess.h>
#include <scanner/MultiScanner.h>
#include <scanner/SingleScanner.h>

#include <filems/facade/FMSFacade.h>
#include <filems/facade/FMSFactoryFacade.h>
#include <filems/factory/FMSFacadeFactory.h>

#include <scanner/detector/PulseThreadPoolInterface.h>
#include <scanner/detector/PulseThreadPoolFactory.h>
#include <scanner/detector/PulseWarehouseThreadPool.h>
#include <scanner/detector/PulseTask.h>
#include <scanner/detector/PulseTaskDropper.h>

#include <adt/grove/KDGrove.h>
#include <adt/grove/KDGroveFactory.h>
#include <adt/kdtree/KDTreeFactory.h>
#include <adt/kdtree/SimpleKDTreeFactory.h>
#include <adt/kdtree/SimpleKDTreeGeometricStrategy.h>
#include <adt/kdtree/MultiThreadKDTreeFactory.h>
#include <adt/kdtree/SAHKDTreeGeometricStrategy.h>
#include <adt/kdtree/SAHKDTreeFactory.h>
#include <adt/kdtree/MultiThreadSAHKDTreeFactory.h>
#include <adt/kdtree/AxisSAHKDTreeFactory.h>
#include <adt/kdtree/AxisSAHKDTreeGeometricStrategy.h>

#include <adt/kdtree/FastSAHKDTreeFactory.h>
#include <adt/kdtree/FastSAHKDTreeGeometricStrategy.h>

#include <sim/comps/ScanningStrip.h>
#include <sim/core/Simulation.h>
#include <assetloading/geometryfilter/WavefrontObj.h>
#include <assetloading/Asset.h>
#include <filems/facade/FMSFacade.h>
#include <python/GLMTypeCaster.h>
#include <python/ScannerWrap.h>
#include <python/PyHeliosSimulation.h>
#include <python/SimulationWrap.h>
#include <python/NoiseSourceWrap.h>
#include <python/AbstractDetectorWrap.h>
#include <python/utils.h>
#include <python/PrimitiveWrap.h>
#include <python/AbstractBeamDeflectorWrap.h>
#include <python/PulseThreadPoolInterfaceWrap.h>
#include <python/ScanningPulseProcessWrap.h>
#include <python/KDTreeFactoryWrapper.h>
#include <python/EnergyModelWrap.h>
#include <python/PyXMLReader.h>
#include <python/SceneHandling.h>
#include <python/NumpyArrayConversion.h>

using helios::filems::FMSFacadeFactory;

namespace helios{

    PYBIND11_MODULE(_helios, m) {
        m.doc() = "Helios python bindings";

        py::bind_vector<std::vector<std::string>>(m, "StringVector");
        py::bind_vector<std::vector<Measurement>>(m, "MeasurementList");
        py::bind_vector<std::vector<Trajectory>>(m, "TrajectoryList");

        py::implicitly_convertible<py::iterable, VectorString>();

        logging::makeQuiet();
        logging::configure({
            {"type", "std_out"}
        });

        py::register_exception<HeliosException>(m, "HeliosException");

        // Enable GDAL (Load its drivers)
        GDALAllRegister();
        if (GDALGetDriverCount() == 0) {
            throw std::runtime_error("GDAL failed to initialize properly.");
        }
        
        // Definitions
        m.def("logging_quiet", &logging::makeQuiet, "Set the logging verbosity level to quiet");
        m.def("logging_silent", &logging::makeSilent, "Set the logging verbosity level to silent");
        m.def("logging_default", &logging::makeDefault, "Set the logging verbosity level to default");
        m.def("logging_verbose", &logging::makeVerbose, "Set the logging verbosity level to verbose");
        m.def("logging_verbose2", &logging::makeVerbose2, "Set the logging verbosity level to verbose 2");
        m.def("logging_time", &logging::makeTime, "Set the logging verbosity level to time");

        m.def("default_rand_generator_seed", &setDefaultRandomnessGeneratorSeed, "Set the seed for the default randomness generator");
        

        py::class_<Asset, std::shared_ptr<Asset>> asset(m, "Asset");
        asset
            .def_readwrite("id", &Asset::id)
            .def_readwrite("name", &Asset::name);


        py::class_<AbstractBeamDeflector, AbstractBeamDeflectorWrap, std::shared_ptr<AbstractBeamDeflector>> abstract_beam_deflector(m, "AbstractBeamDeflector");
        abstract_beam_deflector
            .def(py::init<double, double, double>(),
            py::arg("scanAngleMax_rad"), py::arg("scanFreqMax_Hz"), py::arg("scanFreqMin_Hz"))
            .def_readwrite("scan_freq_max",&AbstractBeamDeflector::cfg_device_scanFreqMax_Hz)
            .def_readwrite("scan_freq_min",&AbstractBeamDeflector::cfg_device_scanFreqMin_Hz)
            .def_readwrite("scan_angle_max",&AbstractBeamDeflector::cfg_device_scanAngleMax_rad)
            .def_readwrite("scan_freq", &AbstractBeamDeflector::cfg_setting_scanFreq_Hz)
            .def_readwrite("scan_angle", &AbstractBeamDeflector::cfg_setting_scanAngle_rad)
            .def_readwrite("vertical_angle_min", &AbstractBeamDeflector::cfg_setting_verticalAngleMin_rad)
            .def_readwrite("vertical_angle_max", &AbstractBeamDeflector::cfg_setting_verticalAngleMax_rad)
            .def_readwrite("current_beam_angle", &AbstractBeamDeflector::state_currentBeamAngle_rad)
            .def_readwrite("angle_diff_rad", &AbstractBeamDeflector::state_angleDiff_rad)
            .def_readwrite("cached_angle_between_pulses", &AbstractBeamDeflector::cached_angleBetweenPulses_rad)
            .def_property_readonly("emitter_relative_attitude",
                                &AbstractBeamDeflector::getEmitterRelativeAttitudeByReference)
            .def_property_readonly("optics_type", &AbstractBeamDeflector::getOpticsType)
            .def("clone", &AbstractBeamDeflector::clone);

        py::class_<OscillatingMirrorBeamDeflector, AbstractBeamDeflector, std::shared_ptr<OscillatingMirrorBeamDeflector>> oscillating_mirror_beam_deflector(m, "OscillatingMirrorBeamDeflector");
        oscillating_mirror_beam_deflector
            .def(py::init<double, double, double, int>(),
            py::arg("scanAngleMax_rad"), py::arg("scanFreqMax_Hz"), py::arg("scanFreqMin_Hz"), py::arg("scanProduct"))

            .def_readwrite("scan_product", &OscillatingMirrorBeamDeflector::cfg_device_scanProduct)
            .def("clone", &OscillatingMirrorBeamDeflector::clone);

        py::class_<ConicBeamDeflector, AbstractBeamDeflector, std::shared_ptr<ConicBeamDeflector>> conic_beam_deflector(m, "ConicBeamDeflector");
        conic_beam_deflector
            .def(py::init<double, double, double>(),
            py::arg("scanAngleMax_rad"), py::arg("scanFreqMax_Hz"), py::arg("scanFreqMin_Hz"))
            .def("clone", &ConicBeamDeflector::clone);
        
        py::class_<FiberArrayBeamDeflector, AbstractBeamDeflector, std::shared_ptr<FiberArrayBeamDeflector>> fiber_array_beam_deflector(m, "FiberArrayBeamDeflector");
        fiber_array_beam_deflector
            .def(py::init<double, double, double, int>(),
            py::arg("scanAngleMax_rad"), py::arg("scanFreqMax_Hz"), py::arg("scanFreqMin_Hz"), py::arg("numFibers"))
            .def_property("num_fibers", &FiberArrayBeamDeflector::getNumFibers, &FiberArrayBeamDeflector::setNumFibers)
            .def("clone", &FiberArrayBeamDeflector::clone);

        
        py::class_<PolygonMirrorBeamDeflector, AbstractBeamDeflector, std::shared_ptr<PolygonMirrorBeamDeflector>> polygon_mirror_beam_deflector(m, "PolygonMirrorBeamDeflector");
        polygon_mirror_beam_deflector
            .def(py::init<double, double, double, double>(),
            py::arg("scanAngleMax_rad"), py::arg("scanFreqMax_Hz"), py::arg("scanFreqMin_Hz"), py::arg("ScanAngleEffectiveMax_rad"))
            .def_property_readonly("scan_angle_effective_max", &PolygonMirrorBeamDeflector::getScanAngleEffectiveMax_rad)
            .def("clone", &PolygonMirrorBeamDeflector::clone);

        
        py::class_<RisleyBeamDeflector, AbstractBeamDeflector, std::shared_ptr<RisleyBeamDeflector>> risley_beam_deflector(m, "RisleyBeamDeflector");
        risley_beam_deflector
            .def(py::init<double, double, double>(),
            py::arg("scanAngleMax_rad"), py::arg("rotorFreq_1_Hz"), py::arg("rotorFreq_2_Hz"))
            .def_readwrite("rotor_speed_rad_1", &RisleyBeamDeflector::rotorSpeed_rad_1)
            .def_readwrite("rotor_speed_rad_2", &RisleyBeamDeflector::rotorSpeed_rad_2)
            .def("clone", &RisleyBeamDeflector::clone);

           
        py::class_<Primitive, PrimitiveWrap, std::shared_ptr<Primitive>> primitive(m, "Primitive");
        primitive
            .def(py::init<>())
            
            .def_property("scene_part", [](Primitive &prim) {
                return prim.part.get();}, [](Primitive &prim, std::shared_ptr<ScenePart> part) {
                prim.part = part;
            })
            .def_property("material", [](Primitive &prim) {
                return prim.material.get();}, [](Primitive &prim, std::shared_ptr<Material> material) {
                prim.material = material;
            })

            .def("incidence_angle",
                [](Primitive &prim, const glm::dvec3& rayOrigin, const glm::dvec3& rayDir, const glm::dvec3& intersectionPoint) {
                    return prim.getIncidenceAngle_rad(rayOrigin, rayDir, intersectionPoint);
                }, py::arg("rayOrigin"), py::arg("rayDir"), py::arg("intersectionPoint"))
            .def("ray_intersection", [](Primitive &prim, const glm::dvec3& rayOrigin, const glm::dvec3& rayDir) {
                    const std::vector<double> &result = prim.getRayIntersection(rayOrigin, rayDir);
                    return py::cast(result);
                })
            .def("ray_intersection_distance", [](Primitive &prim, const glm::dvec3& rayOrigin, const glm::dvec3& rayDir) {
                    return prim.getRayIntersectionDistance(rayOrigin, rayDir);
                })
            .def("update", &Primitive::update)
            .def("is_triangle", [](Primitive &prim) {
                        return dynamic_cast<Triangle *>(&prim) != nullptr;
                    })
            .def("is_AABB", [](Primitive &prim) {
                        return dynamic_cast<AABB *>(&prim) != nullptr;
                    })
            .def("is_voxel", [](Primitive &prim) {
                        return dynamic_cast<Voxel *>(&prim) != nullptr;
                    })
            .def("is_detailed_voxel", [](Primitive &prim) {
                        return dynamic_cast<DetailedVoxel *>(&prim) != nullptr;
                    })
            .def("clone", &Primitive::clone);

        py::class_<AABB, Primitive, std::shared_ptr<AABB>> aabb(m, "AABB");
        aabb
           
            .def(py::init<>())  // Default constructor
            .def(py::init<glm::dvec3, glm::dvec3>(), "Construct AABB from min and max vertices")
            .def_property("vertices", 
                [](const AABB& aabb) {
                    return std::vector<Vertex>(aabb.vertices, aabb.vertices + 2); 
                }, 
                [](AABB& aabb, const std::vector<Vertex>& vertices) {
                    if (vertices.size() != 2) {
                        throw std::runtime_error("Vertices array must have exactly 2 elements.");
                    }
                    std::copy(vertices.begin(), vertices.end(), aabb.vertices); 
                }, 
                "Get and set the vertices of the AABB")
            .def_property("bounds", 
                [](const AABB& aabb) {
                    return std::vector<glm::dvec3>(aabb.bounds, aabb.bounds + 2); 
                }, 
                [](AABB& aabb, const std::vector<glm::dvec3>& bounds) {
                    if (bounds.size() != 2) {
                        throw std::runtime_error("Bounds array must have exactly 2 elements.");
                    }
                    std::copy(bounds.begin(), bounds.end(), aabb.bounds); 
                }, 
                "Get and set the cached bounds of the AABB")
            .def_property_readonly("min_vertex", [](AABB &aabb) { return &(aabb.vertices[0]); }, py::return_value_policy::reference)
            .def_property_readonly("max_vertex", [](AABB &aabb) { return &(aabb.vertices[1]); }, py::return_value_policy::reference)
            .def("__str__", &AABB::toString);


        py::class_<DetailedVoxel, std::shared_ptr<DetailedVoxel>, Primitive> detailed_voxel(m, "DetailedVoxel");
        detailed_voxel
            .def(py::init<>())
            .def(py::init<glm::dvec3, double, std::vector<int>, std::vector<double>>(),
                py::arg("center"), py::arg("VoxelSize"), py::arg("intValues"), py::arg("doubleValues"))
            
            .def_property("nb_echos", &DetailedVoxel::getNbEchos, &DetailedVoxel::setNbEchos)
            .def_property("nb_sampling", &DetailedVoxel::getNbSampling, &DetailedVoxel::setNbSampling)
            .def_property_readonly("number_of_double_values", &DetailedVoxel::getNumberOfDoubleValues)
            .def_property("max_pad", &DetailedVoxel::getMaxPad, &DetailedVoxel::setMaxPad)
            .def("get_double_value", &DetailedVoxel::getDoubleValue, "Get the value at index")
            .def("set_double_value", &DetailedVoxel::setDoubleValue, "Set the value at index", py::arg("index"), py::arg("value"));


        py::class_<AbstractDetector, AbstractDetectorWrap,  std::shared_ptr<AbstractDetector>> abstract_detector(m, "AbstractDetector");
        abstract_detector
                 .def(py::init<
                    std::shared_ptr<Scanner>, 
                    double, 
                    double, 
                    double
                >(), 
                py::arg("scanner"), 
                py::arg("accuracy_m"), 
                py::arg("rangeMin_m"), 
                py::arg("rangeMax_m") = std::numeric_limits<double>::max())
      
            .def_readwrite("accuracy", &AbstractDetector::cfg_device_accuracy_m)
            .def_readwrite("range_min", &AbstractDetector::cfg_device_rangeMin_m)
            .def_readwrite("range_max", &AbstractDetector::cfg_device_rangeMax_m)

            .def_property("las_scale", [](AbstractDetector &self) {
                    return self.getFMS()->write.getMeasurementWriterLasScale();},
                [](AbstractDetector &self, double lasScale) {
                    self.getFMS()->write.setMeasurementWriterLasScale(lasScale);})

            .def("shutdown", &AbstractDetector::shutdown)
                
            .def("clone", &AbstractDetector::clone);


        py::class_<FullWaveformPulseDetector, AbstractDetector, std::shared_ptr<FullWaveformPulseDetector>> full_waveform_pulse_detector(m, "FullWaveformPulseDetector");
        full_waveform_pulse_detector
            .def(py::init<
                std::shared_ptr<Scanner>, 
                double, 
                double, 
                double
            >(), 
            py::arg("scanner"), 
            py::arg("accuracy_m"), 
            py::arg("rangeMin_m"), 
            py::arg("rangeMax_m") = std::numeric_limits<double>::max())
            
            .def("clone", &FullWaveformPulseDetector::clone);


        py::class_<Triangle, std::shared_ptr<Triangle>, Primitive> triangle(m, "Triangle");
        triangle
            .def(py::init<Vertex, Vertex, Vertex>(), py::arg("v0"), py::arg("v1"), py::arg("v2"))
            .def("__str__", &Triangle::toString)
            .def("ray_intersection", &Triangle::getRayIntersection)

            .def_property("vertices", 
                [](const Triangle& tri) {
                    return std::vector<Vertex>(tri.verts, tri.verts + 3); 
                }, 
                [](Triangle& tri, const std::vector<Vertex>& vertices) {
                    if (vertices.size() != 3) {
                        throw std::runtime_error("Vertices array must have exactly 3 elements.");
                    }
                    std::copy(vertices.begin(), vertices.end(), tri.verts); 
                }, 
                "Get and set the vertices of the Triangle")
            .def_property_readonly("face_normal", &Triangle::getFaceNormal);


        py::class_<Vertex, std::shared_ptr<Vertex>> vertex(m, "Vertex"); 
        vertex
            .def(py::init<>())
            .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))

            .def_property("position", [](Vertex &self) {
                    return self.pos;
                }, [](Vertex &self, const glm::dvec3 &position) {
                    self.pos = position;
            })
            .def_readwrite("normal", &Vertex::normal)
            .def_readwrite("tex_coords", &Vertex::texcoords)
            .def("clone", &Vertex::copy);


        py::class_<Trajectory, std::shared_ptr<Trajectory>> trajectory(m, "Trajectory");
        trajectory
            .def(py::init<>())
            .def(py::init<double, glm::dvec3, double, double, double>())
            .def_readwrite("gps_time", &Trajectory::gpsTime)
            .def_readwrite("roll", &Trajectory::roll)
            .def_readwrite("pitch", &Trajectory::pitch)
            .def_readwrite("yaw", &Trajectory::yaw)
            .def_property("position", [](Trajectory &self) {
                    return self.position;
                }, [](Trajectory &self, const glm::dvec3 &position) {
                    self.position = position;
            });


        py::class_<TrajectorySettings, std::shared_ptr<TrajectorySettings>> trajectory_settings(m, "TrajectorySettings");
        trajectory_settings
            .def(py::init<>()) 
            .def_readwrite("start_time", &TrajectorySettings::tStart)
            .def_readwrite("end_time", &TrajectorySettings::tEnd)
            .def_readwrite("teleport_to_start", &TrajectorySettings::teleportToStart);


        py::class_<Measurement, std::shared_ptr<Measurement>> measurement(m, "Measurement");
        measurement
            .def(py::init<>())
            .def(py::init<const Measurement &>())
            

            .def_readwrite("dev_id", &Measurement::devId)
            .def_readwrite("dev_idx", &Measurement::devIdx)
            .def_readwrite("hit_object_id", &Measurement::hitObjectId)
            .def_readwrite("beam_direction", &Measurement::beamDirection)
            .def_readwrite("beam_origin", &Measurement::beamOrigin)
            .def_readwrite("distance", &Measurement::distance)
            .def_readwrite("intensity", &Measurement::intensity)
            .def_readwrite("echo_width", &Measurement::echo_width)
            .def_readwrite("return_number", &Measurement::returnNumber)
            .def_readwrite("pulse_return_number", &Measurement::pulseReturnNumber)
            .def_readwrite("fullwave_index", &Measurement::fullwaveIndex)
            .def_readwrite("classification", &Measurement::classification)
            .def_property("gps_time", 
              [](const Measurement& m) { return m.gpsTime / 1000000000.0; }, 
              [](Measurement& m, double gpsTime) { m.gpsTime = gpsTime * 1000000000.0; })  
            .def_property("position", [](Measurement &self) {
                    return self.position;
                }, [](Measurement &self, const glm::dvec3 &position) {
                    self.position = position;
            });


        py::class_<RandomnessGenerator<double>> randomness_generator(m, "RandomnessGenerator");
        randomness_generator
            .def(py::init<>()) 
            .def("compute_uniform_real_distribution", &RandomnessGenerator<double>::computeUniformRealDistribution)
            .def("uniform_real_distribution_next", &RandomnessGenerator<double>::uniformRealDistributionNext)
            .def("compute_normal_distribution", &RandomnessGenerator<double>::computeNormalDistribution)
            .def("normal_distribution_next", &RandomnessGenerator<double>::normalDistributionNext);


        py::class_<NoiseSource<double>, NoiseSourceWrap<double>> noise_source(m, "NoiseSource");
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


        py::class_<RandomNoiseSource<double>, NoiseSource<double>, RandomNoiseSourceWrap<double>>(m, "RandomNoiseSource")
            .def(py::init<>())

            .def("get_random_noise_type", &RandomNoiseSource<double>::getRandomNoiseType)
            .def("__str__", [](RandomNoiseSource<double> &self) {
                std::ostringstream oss;
                oss << self;
                return oss.str();
            });

        
        py::class_<UniformNoiseSource<double>, RandomNoiseSource<double>>(m, "UniformNoiseSource")
            .def(py::init<double, double>(), py::arg("min") = 0.0, py::arg("max") = 1.0)
        
            .def_property("min", &UniformNoiseSource<double>::getMin, &UniformNoiseSource<double>::setMin)
            .def_property("max", &UniformNoiseSource<double>::getMax, &UniformNoiseSource<double>::setMax)

            .def("configure_uniform_noise", &UniformNoiseSource<double>::configureUniformNoise)
            .def("noise_function", &UniformNoiseSource<double>::noiseFunction)
            .def("get_random_noise_type", &UniformNoiseSource<double>::getRandomNoiseType)
            .def("next", &UniformNoiseSource<double>::next)
            .def("__str__", [](const UniformNoiseSource<double> &ns) {
                std::ostringstream oss;
                oss << ns;
                return oss.str();
            });
        
        
        py::class_<RaySceneIntersection, std::shared_ptr<RaySceneIntersection>> ray_scene_intersection(m, "RaySceneIntersection");
        ray_scene_intersection
            .def(py::init<>())
            .def(py::init<const RaySceneIntersection&>())
            .def_property("primitive",
                        [](RaySceneIntersection &self) { return self.prim; },
                        [](RaySceneIntersection &self, Primitive* prim) { self.prim = prim; },
                        py::return_value_policy::reference)
            .def_property("point",
                        [](RaySceneIntersection &self) { return self.point; }, 
                        [](RaySceneIntersection &self, const glm::dvec3& point) { self.point = point; })
            .def_property("incidence_angle",
                        [](RaySceneIntersection &self) { return self.incidenceAngle; },
                        [](RaySceneIntersection &self, double angle) { self.incidenceAngle = angle; });


        py::class_<ScanningStrip, std::shared_ptr<ScanningStrip>> scanning_strip(m, "ScanningStrip");
        scanning_strip
            .def(py::init<const std::string&>()) 
            .def_property("strip_id",
                        &ScanningStrip::getStripId,
                        &ScanningStrip::setStripId)
            .def_property_readonly("is_last_leg_in_strip", &ScanningStrip::isLastLegInStrip)
            .def("get_leg_ref", [](ScanningStrip& self, int serialId) -> Leg& {
                Leg* leg = self.getLeg(serialId);
                if (!leg) throw std::runtime_error("Leg not found");
                return *leg;
            }, py::return_value_policy::reference)
            .def("has", py::overload_cast<int>(&ScanningStrip::has))
            .def("has", py::overload_cast<Leg&>(&ScanningStrip::has));


        py::class_<SimulationCycleCallback, SimulationCycleCallbackWrap, std::shared_ptr<SimulationCycleCallback>> simulation_cycle_callback(m, "SimulationCycleCallback");
        simulation_cycle_callback
            .def(py::init<py::object>())
            .def("__call__", [](SimulationCycleCallback &callback,
                            py::list measurements,
                            py::list trajectories,
                            const std::string &outpath) {
                py::gil_scoped_acquire acquire;

                // Convert Python lists to std::shared_ptr<std::vector<Measurement>> and std::shared_ptr<std::vector<Trajectory>>
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
        
        py::class_<FWFSettings, std::shared_ptr<FWFSettings>> fwf_settings(m, "FWFSettings");
        fwf_settings
            .def(py::init<>())
            .def(py::init<const FWFSettings&>(), py::arg("fwfSettings"))
            .def_readwrite("bin_size", &FWFSettings::binSize_ns)
            .def_readwrite("min_echo_width", &FWFSettings::minEchoWidth)
            .def_readwrite("peak_energy", &FWFSettings::peakEnergy)
            .def_readwrite("aperture_diameter", &FWFSettings::apertureDiameter)
            .def_readwrite("scanner_efficiency", &FWFSettings::scannerEfficiency)
            .def_readwrite("atmospheric_visibility", &FWFSettings::atmosphericVisibility)
            .def_readwrite("scanner_wave_length", &FWFSettings::scannerWaveLength)
            .def_readwrite("beam_divergence_angle", &FWFSettings::beamDivergence_rad)
            .def_readwrite("pulse_length", &FWFSettings::pulseLength_ns)
            .def_readwrite("beam_sample_quality", &FWFSettings::beamSampleQuality)
            .def_readwrite("win_size", &FWFSettings::winSize_ns)
            .def_readwrite("max_fullwave_range", &FWFSettings::maxFullwaveRange_ns)
            .def("__str__", &FWFSettings::toString);


        py::class_<Rotation, std::shared_ptr<Rotation>> rotation(m, "Rotation");
        rotation
            .def(py::init<>())
            .def(py::init<double, double, double, double, bool>(), py::arg("q0"), py::arg("q1"), py::arg("q2"), py::arg("q3"), py::arg("needsNormalization"))
            .def(py::init<glm::dvec3, double>(), py::arg("axis"), py::arg("angle"))
            .def(py::init<glm::dvec3, glm::dvec3>(), py::arg("u"), py::arg("v"))

            .def_property("q0", &Rotation::getQ0, &Rotation::setQ0)
            .def_property("q1", &Rotation::getQ1, &Rotation::setQ1)
            .def_property("q2", &Rotation::getQ2, &Rotation::setQ2)
            .def_property("q3", &Rotation::getQ3, &Rotation::setQ3)
            .def_property_readonly("axis", &Rotation::getAxis)
            .def_property_readonly("angle", &Rotation::getAngle);


        py::class_<ScannerHead, std::shared_ptr<ScannerHead>> scanner_head(m, "ScannerHead");
        scanner_head
            .def(py::init<glm::dvec3, double>(), py::arg("headRotationAxis"), py::arg("headRotatePerSecMax_rad"))

            .def_readwrite("rotation_axis", &ScannerHead::cfg_device_rotateAxis)
            .def_property_readonly("mount_relative_attitude",
                        &ScannerHead::getMountRelativeAttitudeByReference)
            .def_property("rotate_per_sec_max",
                        &ScannerHead::getRotatePerSecMax,
                        &ScannerHead::setRotatePerSecMax)
            .def_property("rotate_per_sec",
                        &ScannerHead::getRotatePerSec_rad,
                        &ScannerHead::setRotatePerSec_rad)
            .def_property("rotate_stop",
                        &ScannerHead::getRotateStop,
                        &ScannerHead::setRotateStop)
            .def_property("rotate_start",
                        &ScannerHead::getRotateStart,
                        &ScannerHead::setRotateStart)
            .def_property("rotate_range",
                        &ScannerHead::getRotateRange,
                        &ScannerHead::setRotateRange)

            .def_property("current_rotate_angle",
                        &ScannerHead::getRotateCurrent,
                        &ScannerHead::setCurrentRotateAngle_rad);

        
        py::class_<EvalScannerHead, ScannerHead, std::shared_ptr<EvalScannerHead>> eval_scanner_head(m, "EvalScannerHead");
        eval_scanner_head
            .def(py::init<glm::dvec3, double>(), py::arg("headRotationAxis"), py::arg("headRotatePerSecMax_rad"));


        py::class_<Material, std::shared_ptr<Material>> material(m, "Material");
        material
            .def(py::init<>())
            .def(py::init<const Material&>(), py::arg("material"))

            .def_readwrite("name", &Material::name)
            .def_readwrite("is_ground", &Material::isGround)
            .def_readwrite("use_vertex_colors", &Material::useVertexColors)
            .def_readwrite("mat_file_path", &Material::matFilePath)
            .def_readwrite("reflectance", &Material::reflectance)
            .def_readwrite("specularity", &Material::specularity)
            .def_readwrite("specular_exponent", &Material::specularExponent)
            .def_readwrite("classification", &Material::classification)
            .def_readwrite("spectra", &Material::spectra)
            .def_readwrite("map_kd", &Material::map_Kd)
            .def_property("ambient_components", 
                [](Material &m) {
                    return std::vector<float>(std::begin(m.ka), std::end(m.ka));
                }, 
                [](Material &m, const std::vector<float> &value) {
                    if (value.size() != 4) {
                        throw std::runtime_error("ambient_components must have exactly 4 elements");
                    }
                    std::copy(value.begin(), value.end(), m.ka);
                })
            .def_property("diffuse_components", 
                [](Material &m) {
                    return std::vector<float>(std::begin(m.kd), std::end(m.kd));
                }, 
                [](Material &m, const std::vector<float> &value) {
                    if (value.size() != 4) {
                        throw std::runtime_error("diffuse_components must have exactly 4 elements");
                    }
                    std::copy(value.begin(), value.end(), m.kd);
                })
            .def_property("specular_components", 
                [](Material &m) {
                    return std::vector<float>(std::begin(m.ks), std::end(m.ks));
                }, 
                [](Material &m, const std::vector<float> &value) {
                    if (value.size() != 4) {
                        throw std::runtime_error("specular_components must have exactly 4 elements");
                    }
                    std::copy(value.begin(), value.end(), m.ks);
                });


        py::class_<Survey, Asset, std::shared_ptr<Survey>> survey(m, "Survey", py::module_local()); 
        survey
            .def(py::init<>())
            
            .def_readwrite("scanner", &Survey::scanner)
            .def_readwrite("legs", &Survey::legs)
            .def("calculate_length", &Survey::calculateLength)
            .def_property_readonly("length", &Survey::getLength)
            .def_property("name", 
                        [](Survey &s) { return s.name; }, 
                        [](Survey &s, const std::string &name) { s.name = name; })
            .def_property("num_runs", 
                        [](Survey &s) { return s.numRuns; }, 
                        [](Survey &s, int numRuns) { s.numRuns = numRuns; })
            .def_property("sim_speed_factor", 
                        [](Survey &s) { return s.simSpeedFactor; }, 
                        [](Survey &s, double simSpeedFactor) { s.simSpeedFactor = simSpeedFactor; });

        // The following properties exist for ease of use in the Python API.
        // A better solution would be to sort out object ownership on the C++
        // side so that concepts such as scanners and platforms are independent.
        survey.def_property(
            "platform",
            [](const Survey &s) { return s.scanner->platform; },
            [](Survey& s, std::shared_ptr<Platform> p) { s.scanner->platform = p; }
        );
        survey.def_property(
            "scene",
            [](const Survey &s) { return s.scanner->platform->scene; },
            [](Survey& s, std::shared_ptr<Scene> sc) { s.scanner->platform->scene = sc; }
        );

        py::class_<Leg, std::shared_ptr<Leg>> leg(m, "Leg");
        leg
            .def(py::init<>())
            .def(py::init<double, int, std::shared_ptr<ScanningStrip>>(),
             py::arg("length"), py::arg("serialId"), py::arg("strip"))
            .def(py::init<Leg&>(), py::arg("leg"))
            .def_readwrite("scanner_settings", &Leg::mScannerSettings) 
            .def_readwrite("platform_settings", &Leg::mPlatformSettings)
            .def_readwrite("trajectory_settings", &Leg::mTrajectorySettings)

            .def_property("length", &Leg::getLength, &Leg::setLength)
            .def_property("serial_id", &Leg::getSerialId, &Leg::setSerialId)
            .def_property("strip", &Leg::getStrip, &Leg::setStrip)
            .def("belongs_to_strip", &Leg::isContainedInAStrip);


        py::class_<ScenePart, std::shared_ptr<ScenePart>> scene_part(m, "ScenePart");
        scene_part
            .def(py::init<>())
            .def(py::init<const ScenePart&, bool>(),
             py::arg("sp"), py::arg("shallowPrimitives") = false) 
            
            .def_readwrite("origin", &ScenePart::mOrigin)
            .def_readwrite("rotation", &ScenePart::mRotation)
            .def_readwrite("scale", &ScenePart::mScale)
            .def_readwrite("bound", &ScenePart::bound)
            .def_readwrite("force_on_ground", &ScenePart::forceOnGround)

            .def_property("is_ground",
                        [](const ScenePart& self) {
                            if (self.mPrimitives.empty()) {
                                throw std::runtime_error("It it required to have Primitives in the ScenePart to get the isGround property.");
                            }
                            return self.mPrimitives[0]->material->isGround;
                        },
                        [](ScenePart& self, bool isGround) {
                            if (self.mPrimitives.empty()) {
                                throw std::runtime_error("It it required to have Primitives in the ScenePart to set the isGround property.");
                            }
                            for (auto& primitive : self.mPrimitives) {
                                primitive->material->isGround = isGround;
                            }
                        })
            .def_property("centroid", &ScenePart::getCentroid, &ScenePart::setCentroid) 
            .def_property("id", &ScenePart::getId, &ScenePart::setId)
            .def_property("dyn_object_step",
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
            
            .def_property("observer_step",
                      [](const ScenePart& self) -> size_t {
                          if (self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT) {
                             return dynamic_cast<const DynMovingObject&>(self).getObserverStepInterval();
                          } else {
                              throw std::runtime_error("ScenePart is not a DynMovingObject.");
                          }
                      },
                      [](ScenePart& self, size_t stepInterval) {
                          if (self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT) {
                             dynamic_cast<DynMovingObject&>(self).setObserverStepInterval(stepInterval);
                          } else {
                              throw std::runtime_error("ScenePart is not a DynMovingObject.");
                          }
                      })

            .def_property("primitives", &ScenePart::getPrimitives, &ScenePart::setPrimitives)
            .def("primitive", [](ScenePart& self, size_t index) -> Primitive* {
                if (index < self.mPrimitives.size()) {
                    return self.mPrimitives[index];
                } else {
                    throw std::out_of_range("Index out of range");
                }
            }, py::return_value_policy::reference)
            .def_property_readonly("num_primitives", [](const ScenePart& self) -> size_t {
                return self.mPrimitives.size();})
            .def_property_readonly("all_vertices", &ScenePart::getAllVertices)
            .def("isDynamicMovingObject", [](const ScenePart& self) -> bool {
                return self.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT;})
            .def("compute_centroid", &ScenePart::computeCentroid, py::arg("computeBound") = false)
            .def("compute_centroid_w_bound", &ScenePart::computeCentroid, py::arg("computeBound") = true)
            .def("compute_transform", &ScenePart::computeTransformations);
             
        
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
        

        py::class_<ScannerSettings, std::shared_ptr<ScannerSettings>> scanner_settings(m, "ScannerSettings");
        scanner_settings
            .def(py::init<>())
            .def(py::init<ScannerSettings*>(), py::arg("scannerSettings"))
            
            .def_readwrite("id", &ScannerSettings::id)
            .def_readwrite("is_active", &ScannerSettings::active)
            .def_readwrite("head_rotation", &ScannerSettings::headRotatePerSec_rad)
            .def_readwrite("rotation_start_angle", &ScannerSettings::headRotateStart_rad)
            .def_readwrite("rotation_stop_angle", &ScannerSettings::headRotateStop_rad)
            .def_readwrite("pulse_frequency", &ScannerSettings::pulseFreq_Hz)
            .def_readwrite("scan_angle", &ScannerSettings::scanAngle_rad)
            .def_readwrite("min_vertical_angle", &ScannerSettings::verticalAngleMin_rad)
            .def_readwrite("max_vertical_angle", &ScannerSettings::verticalAngleMax_rad)
            .def_readwrite("scan_frequency", &ScannerSettings::scanFreq_Hz)
            .def_readwrite("beam_divergence_angle", &ScannerSettings::beamDivAngle)
            .def_readwrite("trajectory_time_interval", &ScannerSettings::trajectoryTimeInterval)
            .def_readwrite("vertical_resolution", &ScannerSettings::verticalResolution_rad)
            .def_readwrite("horizontal_resolution", &ScannerSettings::horizontalResolution_rad)

            .def_property_readonly("base_template", &ScannerSettings::getTemplate, py::return_value_policy::reference)

            .def("cherry_pick", &ScannerSettings::cherryPick, py::arg("cherries"), py::arg("fields"), py::arg("templateFields") = nullptr)
            .def("fit_to_resolution", &ScannerSettings::fitToResolution)
            .def("has_template", &ScannerSettings::hasTemplate)
            .def("has_default_resolution", &ScannerSettings::hasDefaultResolution)
            .def("__repr__", &ScannerSettings::toString)
            .def("__str__", &ScannerSettings::toString);

        
        py::class_<PlatformSettings, std::shared_ptr<PlatformSettings>> platform_settings(m, "PlatformSettings");
        platform_settings
            .def(py::init<>())
            .def(py::init<PlatformSettings*>(), py::arg("platformSettings"))
       
            .def_readwrite("id", &PlatformSettings::id)
            .def_readwrite("x", &PlatformSettings::x)
            .def_readwrite("y", &PlatformSettings::y)
            .def_readwrite("z", &PlatformSettings::z)
            .def_readwrite("is_yaw_angle_specified", &PlatformSettings::yawAtDepartureSpecified)
            .def_readwrite("yaw_angle", &PlatformSettings::yawAtDeparture)
            .def_readwrite("is_on_ground", &PlatformSettings::onGround)
            .def_readwrite("is_stop_and_turn", &PlatformSettings::stopAndTurn)
            .def_readwrite("is_smooth_turn", &PlatformSettings::smoothTurn)
            .def_readwrite("is_slowdown_enabled", &PlatformSettings::slowdownEnabled)
            .def_readwrite("speed_m_s", &PlatformSettings::movePerSec_m)

            .def_property_readonly("base_template", &PlatformSettings::getTemplate, py::return_value_policy::reference)
            .def_property("position", &PlatformSettings::getPosition, [](PlatformSettings& self, const glm::dvec3& pos) { self.setPosition(pos); })

            .def("cherryPick", &PlatformSettings::cherryPick, py::arg("cherries"), py::arg("fields"), py::arg("templateFields") = nullptr)
            
            .def("has_template", &PlatformSettings::hasTemplate)
            .def("to_string", &PlatformSettings::toString);
        

        py::class_<Scene, std::shared_ptr<Scene>> scene(m, "Scene");
        scene
            .def(py::init<>())
            .def(py::init<Scene&>(), py::arg("scene"))

            .def_readwrite("scene_parts", &Scene::parts)
            .def_readwrite("primitives", &Scene::primitives)

            .def_property("bbox", &Scene::getBBox, &Scene::setBBox)
            .def_property("bbox_crs", &Scene::getBBoxCRS, &Scene::setBBoxCRS)
            .def_property("kd_grove_factory", &Scene::getKDGroveFactory, &Scene::setKDGroveFactory)
            .def_property_readonly("num_primitives", [](const ScenePart& self) -> size_t {
                return self.mPrimitives.size();})
            .def_property_readonly("aabb", &Scene::getAABB, py::return_value_policy::reference)
           
            .def_property_readonly("shift", &Scene::getShift)
            .def_property("dyn_scene_step",
                      [](const Scene& self) -> size_t {
                        return dynamic_cast<const DynScene&>(self).getStepInterval();
                      },
                      [](Scene& self, size_t stepInterval) {
                        dynamic_cast<DynScene&>(self).setStepInterval(stepInterval); 
                      })            
            .def_property("default_reflectance", &Scene::getDefaultReflectance, &Scene::setDefaultReflectance)

            .def("scene_parts_size", [](Scene &self) { return self.parts.size(); })
            .def("get_scene_part", [](Scene &self, size_t index) -> ScenePart& {
                if (index < self.parts.size()) {
                    return *(self.parts[index]);
                } else {
                    throw std::out_of_range("Index out of range");
                }
            }, py::return_value_policy::reference)
            .def("ground_point_at", [](Scene &self, glm::dvec3 point) { return self.getGroundPointAt(point); })
            .def("finalize_loading", &Scene::finalizeLoading, py::arg("safe") = false)
            .def("write_object", &Scene::writeObject)
            .def("translate", [](Scene &scene, double x, double y, double z) {
                glm::dvec3 shift(x, y, z);
                for (Primitive* p : scene.primitives) {
                    p->translate(shift);
                }
            })
            .def("new_triangle", [](Scene &scene) {
                Vertex v;
                v.pos[0] = 0.0; v.pos[1] = 0.0; v.pos[2] = 0.0;
                Triangle* tri = new Triangle(v, v, v);
                scene.primitives.push_back(tri);
                return tri;
            }, py::return_value_policy::reference)
            .def("new_detailed_voxel", [](Scene &scene) {
                std::vector<int> vi({0, 0});
                std::vector<double> vd({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
                DetailedVoxel* dv = new DetailedVoxel(0.0, 0.0, 0.0, 0.5, vi, vd);
                scene.primitives.push_back(dv);
                return dv;
            }, py::return_value_policy::reference)
            .def("primitive", [](Scene &scene, size_t index) -> Primitive* {
                if (index < scene.primitives.size()) {
                    return scene.primitives[index];
                } else {
                    throw std::out_of_range("Index out of range");
                }
            }, py::return_value_policy::reference)
            .def("build_kd_grove", &Scene::buildKDGroveWithLog, py::arg("safe") = true)
            .def("intersection_min_max",
                py::overload_cast<std::vector<double> const&, glm::dvec3 const&, glm::dvec3 const&, bool const>(&Scene::getIntersection, py::const_),
                py::arg("tMinMax"), py::arg("rayOrigin"), py::arg("rayDir"), py::arg("groundOnly"))
            .def("shutdown", &Scene::shutdown);


        py::class_<StaticScene, Scene, std::shared_ptr<StaticScene>> static_scene(m, "StaticScene");
        static_scene
            .def(py::init<>())
            .def("get_static_object_part", &StaticScene::getStaticObject, py::arg("id"))
            .def("set_static_object_part", &StaticScene::setStaticObject, py::arg("id"), py::arg("part"))
            .def("append_static_object_part", &StaticScene::appendStaticObject, py::arg("part"))
            .def("remove_static_object_part", &StaticScene::removeStaticObject, py::arg("id"))
            .def("clear_static_object_parts", &StaticScene::clearStaticObjects)
            .def("write_object", &StaticScene::writeObject, py::arg("path"))
            .def("shutdown", &StaticScene::shutdown);


        py::class_<Platform, std::shared_ptr<Platform>> platform(m, "Platform");
        platform
            .def(py::init<>())
            .def(py::init<Platform&>(), py::arg("platform"))
           
            .def_readwrite("last_check_z", &Platform::lastCheckZ)
            .def_readwrite("dmax", &Platform::dmax)
            .def_readwrite("is_orientation_on_leg_init", &Platform::mSetOrientationOnLegInit)
            .def_readwrite("is_on_ground", &Platform::onGround)
            .def_readwrite("is_stop_and_turn", &Platform::stopAndTurn)
            .def_readwrite("settings_speed_m_s", &Platform::cfg_settings_movePerSec_m)
            .def_readwrite("is_slowdown_enabled", &Platform::slowdownEnabled)
            .def_readwrite("is_smooth_turn", &Platform::smoothTurn)
            .def_readwrite("device_relative_position", &Platform::cfg_device_relativeMountPosition)
            .def_readwrite("device_relative_attitude", &Platform::cfg_device_relativeMountAttitude)
            .def_readwrite("position_x_noise_source", &Platform::positionXNoiseSource)
            .def_readwrite("position_y_noise_source", &Platform::positionYNoiseSource)
            .def_readwrite("position_z_noise_source", &Platform::positionZNoiseSource)
            .def_readwrite("attitude_x_noise_source", &Platform::attitudeXNoiseSource)
            .def_readwrite("attitude_y_noise_source", &Platform::attitudeYNoiseSource)
            .def_readwrite("attitude_z_noise_source", &Platform::attitudeZNoiseSource)
            .def_readwrite("scene", &Platform::scene)
            .def_readwrite("target_waypoint", &Platform::targetWaypoint)
            .def_readwrite("origin_waypoint", &Platform::originWaypoint)
            .def_readwrite("next_waypoint", &Platform::nextWaypoint)
            .def_readwrite("cached_end_target_angle_xy", &Platform::cached_endTargetAngle_xy)
            .def_readwrite("cached_origin_to_target_angle_xy", &Platform::cached_originToTargetAngle_xy)
            .def_readwrite("cached_target_to_next_angle_xy", &Platform::cached_targetToNextAngle_xy)
            .def_readwrite("cached_distance_to_target_xy", &Platform::cached_distanceToTarget_xy)
        
            .def_property("position", &Platform::getPosition, &Platform::setPosition)
            .def_property("attitude", [](Platform &self) {
                    return self.attitude;
                }, [](Platform &self, const Rotation &attitude) {
                    self.attitude = attitude;
            })
            .def_property("absolute_mount_position", [](Platform &self) {
                    return self.cached_absoluteMountPosition;
                }, [](Platform &self, const glm::dvec3 &position) {
                    self.cached_absoluteMountPosition = position;
            })
            .def_property("absolute_mount_attitude", [](Platform &self) {
                    return self.cached_absoluteMountAttitude;
                }, [](Platform &self, const Rotation &attitude) {
                    self.cached_absoluteMountAttitude = attitude;
            })
            .def_property("last_ground_check", [](const Platform &self) {
                    return self.lastGroundCheck;
                }, [](Platform &self, const glm::dvec3 &position) {
                    self.lastGroundCheck = position;
            })
            .def_property("cached_dir_current", [](const Platform &self) {
                    return self.cached_dir_current;
                }, [](Platform &self, const glm::dvec3 &position) {
                    self.cached_dir_current = position;
            })
            .def_property("cached_dir_current_xy", [](const Platform &self) {
                    return self.cached_dir_current_xy;
                }, [](Platform &self, const glm::dvec3 &position) {
                    self.cached_dir_current_xy = position;
            })
            .def_property("cached_vector_to_target", [](const Platform &self) {
                    return self.cached_vectorToTarget;
                }, [](Platform &self, const glm::dvec3 &position) {
                    self.cached_vectorToTarget = position;
            })
            .def_property("cached_vector_to_target_xy", [](const Platform &self) {
                return self.cached_vectorToTarget_xy;
            }, [](Platform &self, const glm::dvec3 &position) {
                self.cached_vectorToTarget_xy = position;
            })
            .def_property("cached_origin_to_target_dir_xy", [](const Platform &self) {
                return self.cached_originToTargetDir_xy;
            }, [](Platform &self, const glm::dvec3 &position) {
                self.cached_originToTargetDir_xy = position;
            })
            .def_property("cached_target_to_next_dir_xy", [](const Platform &self) {
                return self.cached_targetToNextDir_xy;
            }, [](Platform &self, const glm::dvec3 &position) {
                self.cached_targetToNextDir_xy = position;
            });
        

        py::class_<MovingPlatform, Platform, std::shared_ptr<MovingPlatform>> moving_platform(m, "MovingPlatform");
        moving_platform

            .def(py::init<>())
            
            .def_property("velocity", &MovingPlatform::getVelocity, &MovingPlatform::setVelocity)
            .def("apply_settings", &MovingPlatform::applySettings)
            .def("do_sim_step", &MovingPlatform::doSimStep)
            .def("init_leg_manual", &MovingPlatform::initLegManual)
            .def("init_leg_manual_interactive", &MovingPlatform::initLegManualIterative)
            .def("waypoint_reached", &MovingPlatform::waypointReached)
            .def("can_move", &MovingPlatform::canMove)
            .def("clone", &MovingPlatform::clone);
        

        py::class_<SimplePhysicsPlatform, MovingPlatform, std::shared_ptr<SimplePhysicsPlatform>> simple_physics_platform(m, "SimplePhysicsPlatform");
        simple_physics_platform
            .def(py::init<>())
            .def_readwrite("drag_magnitude", &SimplePhysicsPlatform::mCfg_drag)

            .def("prepare_simulation", &SimplePhysicsPlatform::prepareSimulation)
            .def("prepare_leg", &SimplePhysicsPlatform::prepareLeg)
            .def("do_physics_step", &SimplePhysicsPlatform::doPhysicsStep)
            .def("do_sim_step", &SimplePhysicsPlatform::doSimStep)
            .def("do_control_step", &SimplePhysicsPlatform::doControlStep)
            .def("configure_step_magnitude", &SimplePhysicsPlatform::configureStepMagnitude)
            .def("check_speed_limit", &SimplePhysicsPlatform::checkSpeedLimit)
            .def("clone", &SimplePhysicsPlatform::clone);
        

        py::class_<GroundVehiclePlatform, SimplePhysicsPlatform, std::shared_ptr<GroundVehiclePlatform>> ground_vehicle_platform(m, "GroundVehiclePlatform");
        ground_vehicle_platform
            .def(py::init<>())
            .def("do_control_step", &GroundVehiclePlatform::doControlStep)
            .def("prepare_simulation", &GroundVehiclePlatform::prepareSimulation)
            .def("set_destination", &GroundVehiclePlatform::setDestination)
            .def("clone", &GroundVehiclePlatform::clone);


        py::class_<LinearPathPlatform, MovingPlatform, std::shared_ptr<LinearPathPlatform>> linear_path_platform(m, "LinearPathPlatform");
        linear_path_platform
            .def(py::init<>())
            .def("do_sim_step", &LinearPathPlatform::doSimStep)
            .def("set_destination", &LinearPathPlatform::setDestination)
            .def("clone", &LinearPathPlatform::clone);

        
        py::class_<HelicopterPlatform, SimplePhysicsPlatform, std::shared_ptr<HelicopterPlatform>> helicopter_platform(m, "HelicopterPlatform");
        helicopter_platform
            .def(py::init<>())
            .def_readwrite("slowdown_distance_xy", &HelicopterPlatform::cfg_slowdown_dist_xy)
            .def_readwrite("slowdown_magnitude", &HelicopterPlatform::cfg_slowdown_magnitude)
            .def_readwrite("speedup_magnitude", &HelicopterPlatform::cfg_speedup_magnitude)
            .def_readwrite("max_engine_force_xy", &HelicopterPlatform::ef_xy_max)
            .def_readwrite("roll", &HelicopterPlatform::roll)
            .def_readwrite("pitch", &HelicopterPlatform::pitch)
            .def_readwrite("last_sign", &HelicopterPlatform::lastSign)
            .def_readwrite("base_pitch_angle", &HelicopterPlatform::cfg_pitch_base)
            .def_readwrite("yaw_speed", &HelicopterPlatform::cfg_yaw_speed)
            .def_readwrite("roll_speed", &HelicopterPlatform::cfg_roll_speed)
            .def_readwrite("pitch_speed", &HelicopterPlatform::cfg_pitch_speed)
            .def_readwrite("max_roll_offset", &HelicopterPlatform::cfg_max_roll_offset)
            .def_readwrite("max_pitch_offset", &HelicopterPlatform::cfg_max_pitch_offset)
            .def_readwrite("max_pitch", &HelicopterPlatform::cfg_max_pitch)
            .def_readwrite("min_pitch", &HelicopterPlatform::cfg_min_pitch)
            .def_readwrite("slowdown_factor", &HelicopterPlatform::cfg_slowdownFactor)
            .def_readwrite("speedup_factor", &HelicopterPlatform::cfg_speedupFactor)
            .def_readwrite("pitch_step_magnitude", &HelicopterPlatform::cfg_pitchStepMagnitude)
            .def_readwrite("roll_step_magnitude", &HelicopterPlatform::cfg_rollStepMagnitude)
            .def_readwrite("yaw_step_magnitude", &HelicopterPlatform::cfg_yawStepMagnitude)
            .def_readwrite("alignment_threshold", &HelicopterPlatform::cfg_alignmentThreshold)
            .def_readwrite("speed_xy", &HelicopterPlatform::speed_xy)
            .def_readwrite("last_speed_xy", &HelicopterPlatform::lastSpeed_xy)
            .def_readwrite("rotation", &HelicopterPlatform::r)
            .def_readwrite("dir_attitude", &HelicopterPlatform::dirAttitudeXY)
            .def_readwrite("cache_turn_iterations", &HelicopterPlatform::cache_turnIterations)
            .def_readwrite("cache_turning", &HelicopterPlatform::cache_turning)
            .def_readwrite("cache_aligning", &HelicopterPlatform::cache_aligning)
            .def_readwrite("cache_distance_threshold", &HelicopterPlatform::cache_xyDistanceThreshold)
            .def_readwrite("cache_speedup_finished", &HelicopterPlatform::cache_speedUpFinished)
            
            .def_property("heading_rad", &HelicopterPlatform::getHeadingRad, &HelicopterPlatform::setHeadingRad)
            .def("do_control_step", &HelicopterPlatform::doControlStep)
            .def("compute_lift_sink_rate", &HelicopterPlatform::computeLiftSinkRate)
            .def("compute_xy_speed", &HelicopterPlatform::computeXYSpeed)
            .def("compute_engine_force", &HelicopterPlatform::computeEngineForce)
            .def("compute_rotation_angles", &HelicopterPlatform::computeRotationAngles)
            .def("compute_alignment_angles", &HelicopterPlatform::computeAlignmentAngles)
            .def("compute_turning_angles", &HelicopterPlatform::computeTurningAngles)
            .def("compute_slowdown_step", &HelicopterPlatform::computeSlowdownStep)
            .def("compute_speedup_step", &HelicopterPlatform::computeSpeedupStep)
            .def("rotate", &HelicopterPlatform::rotate)
            .def("handle_route", &HelicopterPlatform::handleRoute)
            .def("can_stop_and_turn", &HelicopterPlatform::canStopAndTurn)
            .def("prepare_simulation", &HelicopterPlatform::prepareSimulation)
            .def("init_leg_manual", &HelicopterPlatform::initLegManual)
            .def("init_leg", &HelicopterPlatform::initLeg)
            .def("waypoint_reached", &HelicopterPlatform::waypointReached)
            .def("update_static_cache", &HelicopterPlatform::updateStaticCache)
            .def("compute_turn_distance_threshold", &HelicopterPlatform::computeTurnDistanceThreshold)
            .def("compute_non_smooth_slowdown_dist", &HelicopterPlatform::computeNonSmoothSlowdownDist)
            .def("clone", &HelicopterPlatform::clone);


        py::class_<SwapOnRepeatHandler, std::shared_ptr<SwapOnRepeatHandler>> swap_on_repeat_handler(m, "SwapOnRepeatHandler");
        swap_on_repeat_handler
            .def(py::init<>())

            .def_property(
                "baseline",
                [](SwapOnRepeatHandler &self) {
                    return self.baseline.get();  
                },
                
                [](SwapOnRepeatHandler &self, ScenePart *baseline) {
                    self.baseline.reset(baseline); 
                },
                py::return_value_policy::take_ownership
            )
           
            .def_property_readonly("num_target_swaps", &SwapOnRepeatHandler::getNumTargetSwaps)
            .def_property_readonly("num_target_replays", &SwapOnRepeatHandler::getNumTargetReplays)
            .def_property("keep_crs", &SwapOnRepeatHandler::isKeepCRS, &SwapOnRepeatHandler::setKeepCRS)
            .def_property("discard_on_replay", &SwapOnRepeatHandler::needsDiscardOnReplay, &SwapOnRepeatHandler::setDiscardOnReplay)
            
            .def("swap", &SwapOnRepeatHandler::swap)
            .def("prepare", &SwapOnRepeatHandler::prepare)
            .def("push_time_to_live", &SwapOnRepeatHandler::pushTimeToLive)
            .def("push_swap_filters", &SwapOnRepeatHandler::pushSwapFilters);
        

        py::class_<EnergyModel, EnergyModelWrap, std::shared_ptr<EnergyModel>> energy_model(m, "EnergyModel");
        energy_model
            .def(py::init<ScanningDevice&>(), py::arg("device"))
            
            .def("compute_intensity", &EnergyModel::computeIntensity)
            .def("compute_received_power", &EnergyModel::computeReceivedPower)
            .def("compute_emitted_power", &EnergyModel::computeEmittedPower)
            .def("compute_target_area", &EnergyModel::computeTargetArea)
            .def("compute_cross_section", &EnergyModel::computeCrossSection);
        

        py::class_<BaseEnergyModel, EnergyModel, std::shared_ptr<BaseEnergyModel>> base_energy_model(m, "BaseEnergyModel");
        base_energy_model
            .def(py::init<ScanningDevice&>(), py::arg("device"))
            .def("compute_intensity", &BaseEnergyModel::computeIntensity)
            .def("compute_received_power", &BaseEnergyModel::computeReceivedPower)
            .def("compute_emitted_power", &BaseEnergyModel::computeEmittedPower)
            .def("compute_target_area", &BaseEnergyModel::computeTargetArea)
            .def("compute_cross_section", &BaseEnergyModel::computeCrossSection);


        py::class_<ScanningDevice, std::shared_ptr<ScanningDevice> > scanning_device(m, "ScanningDevice");
        scanning_device
            .def(py::init<ScanningDevice const &>())
            .def(py::init<
                size_t,
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
                double
                >())

            .def_readwrite("cached_dr2", &ScanningDevice::cached_Dr2)
            .def_readwrite("cached_bt2", &ScanningDevice::cached_Bt2)
            .def_readwrite("cached_subray_rotation", &ScanningDevice::cached_subrayRotation)
            .def_readwrite("cached_subray_divergence", &ScanningDevice::cached_subrayDivergenceAngle_rad)
            .def_readwrite("cached_subray_radius_step", &ScanningDevice::cached_subrayRadiusStep)

            .def_property("energy_model", &ScanningDevice::getEnergyModel, &ScanningDevice::setEnergyModel)

            .def_property("fwf_settings", 
                    &ScanningDevice::getFWFSettings, 
                      &ScanningDevice::setFWFSettings)
            .def_property("received_energy_min", 
                      &ScanningDevice::getReceivedEnergyMin, 
                      &ScanningDevice::setReceivedEnergyMin)
            .def_property("last_pulse_was_hit",
                        &ScanningDevice::lastPulseWasHit,
                        &ScanningDevice::setLastPulseWasHit)
            
            .def("set_head_relative_emitter_position", &ScanningDevice::setHeadRelativeEmitterPosition)
            .def("set_head_relative_emitter_attitude", &ScanningDevice::setHeadRelativeEmitterAttitude)
            .def("prepare_simulation", &ScanningDevice::prepareSimulation, py::arg("legacyEnergyModel") = false)
            .def("configure_beam", &ScanningDevice::configureBeam)
            .def("calcAtmosphericAttenuation", &ScanningDevice::calcAtmosphericAttenuation)
            .def("calcRaysNumber", &ScanningDevice::calcRaysNumber)
            .def("doSimStep", &ScanningDevice::doSimStep)
            .def("calcAbsoluteBeamAttitude", &ScanningDevice::calcAbsoluteBeamAttitude)
            .def("calc_exact_absolute_beam_attitude", &ScanningDevice::calcExactAbsoluteBeamAttitude)
            .def("computeSubrays", &ScanningDevice::computeSubrays)
            .def("initializeFullWaveform", &ScanningDevice::initializeFullWaveform)
            .def("calcIntensity", py::overload_cast<double, double, const Material&, int>(&ScanningDevice::calcIntensity, py::const_))
            .def("calcIntensity", py::overload_cast<double, double, int>(&ScanningDevice::calcIntensity, py::const_))
            .def("eval_range_error_expression", &ScanningDevice::evalRangeErrorExpression);


        py::class_<Scanner, ScannerWrap, std::shared_ptr<Scanner>> scanner(m, "Scanner");
        scanner
            .def(py::init<std::string const&, std::list<int> const&, bool, bool, bool, bool, bool>(),
                 py::arg("id"),
                 py::arg("pulseFreqs"),
                 py::arg("writeWaveform") = false,
                 py::arg("writePulse") = false,
                 py::arg("calcEchowidth") = false,
                 py::arg("fullWaveNoise") = false,
                 py::arg("platformNoiseDisabled") = false)
            .def(py::init<Scanner&>(), py::arg("scanner"))

            .def_readwrite("intersection_handling_noise_source", &Scanner::intersectionHandlingNoiseSource) 
            .def_readwrite("rand_gen1", &Scanner::randGen1)
            .def_readwrite("rand_gen2", &Scanner::randGen2)
            .def_readwrite("trajectory_time_interval", &Scanner::trajectoryTimeInterval_ns)
            .def_readwrite("platform", &Scanner::platform)

            .def("initialize_sequential_generators", &Scanner::initializeSequentialGenerators)
            .def("build_scanning_pulse_process", &Scanner::buildScanningPulseProcess,
                py::arg("parallelization_strategy"), py::arg("dropper"), py::arg("pool"))
            .def("apply_settings", py::overload_cast<std::shared_ptr<ScannerSettings>>(&Scanner::applySettings))
            .def("apply_settings", py::overload_cast<std::shared_ptr<ScannerSettings>, size_t>(&Scanner::applySettings))
            .def("retrieve_current_settings", py::overload_cast<>(&Scanner::retrieveCurrentSettings))
            .def("retrieve_current_settings", py::overload_cast<size_t>(&Scanner::retrieveCurrentSettings))
            .def("apply_settings_FWF", py::overload_cast<FWFSettings>(&Scanner::applySettingsFWF))
            .def("apply_settings_FWF", py::overload_cast<FWFSettings, size_t>(&Scanner::applySettingsFWF))
            .def("do_sim_step", &Scanner::doSimStep, py::arg("legIndex"), py::arg("currentGpsTime"))
            .def("to_string", &Scanner::toString)
            .def("calc_rays_number", py::overload_cast<>(&Scanner::calcRaysNumber))
            .def("calc_rays_number", py::overload_cast<size_t>(&Scanner::calcRaysNumber))
            .def("prepare_discretization", py::overload_cast<>(&Scanner::prepareDiscretization))
            .def("prepare_discretization", py::overload_cast<size_t>(&Scanner::prepareDiscretization))
        
            .def("calc_atmospheric_attenuation", py::overload_cast<>(&Scanner::calcAtmosphericAttenuation, py::const_))
            .def("calc_atmospheric_attenuation", py::overload_cast<size_t>(&Scanner::calcAtmosphericAttenuation, py::const_))
            .def("check_max_NOR", py::overload_cast<int>(&Scanner::checkMaxNOR))
            .def("check_max_NOR", py::overload_cast<int, size_t>(&Scanner::checkMaxNOR))
            .def("calc_absolute_beam_attitude", py::overload_cast<size_t>(&Scanner::calcAbsoluteBeamAttitude))
            .def("calc_absolute_beam_attitude", py::overload_cast<>(&Scanner::calcAbsoluteBeamAttitude))
            .def("handle_sim_step_noise", &Scanner::handleSimStepNoise)
            .def("on_leg_complete", &Scanner::onLegComplete)
            .def("on_simulation_finished", &Scanner::onSimulationFinished)
            .def("handle_trajectory_output", &Scanner::handleTrajectoryOutput)
            .def("track_output_path", &Scanner::trackOutputPath)

            .def("specific_current_pulse_number", py::overload_cast<size_t>(&Scanner::getCurrentPulseNumber, py::const_), py::arg("index"))
            .def("get_specific_num_rays", py::overload_cast<size_t>(&Scanner::getNumRays, py::const_))
            .def("set_specific_num_rays", py::overload_cast<int, size_t>(&Scanner::setNumRays))
            .def("get_specific_pulse_length", py::overload_cast<size_t>(&Scanner::getPulseLength_ns, py::const_), py::arg("index"))
            .def("set_specific_pulse_length", py::overload_cast<double, size_t>(&Scanner::setPulseLength_ns), py::arg("value"), py::arg("index"))
            .def("get_specific_last_pulse_was_hit", py::overload_cast<size_t>(&Scanner::lastPulseWasHit, py::const_), py::arg("index"))
            .def("set_specific_last_pulse_was_hit", py::overload_cast<bool, size_t>(&Scanner::setLastPulseWasHit), py::arg("value"), py::arg("index"))
            .def("get_specific_beam_divergence", py::overload_cast<size_t>(&Scanner::getBeamDivergence, py::const_), py::arg("index"))
            .def("set_specific_beam_divergence", py::overload_cast<double, size_t>(&Scanner::setBeamDivergence), py::arg("value"), py::arg("index"))
            
            .def("get_specific_average_power", py::overload_cast<size_t>(&Scanner::getAveragePower, py::const_), py::arg("index"))
            .def("set_specific_average_power", py::overload_cast<double, size_t>(&Scanner::setAveragePower), py::arg("value"), py::arg("index"))
            
            .def("get_specific_beam_quality", py::overload_cast<size_t>(&Scanner::getBeamQuality, py::const_), py::arg("index"))
            .def("set_specific_beam_quality", py::overload_cast<double, size_t>(&Scanner::setBeamQuality), py::arg("value"), py::arg("index"))
            
            .def("get_specific_efficiency", py::overload_cast<size_t>(&Scanner::getEfficiency, py::const_), py::arg("index"))
            .def("set_specific_efficiency", py::overload_cast<double, size_t>(&Scanner::setEfficiency), py::arg("value"), py::arg("index"))
            
            .def("get_specific_receiver_diameter", py::overload_cast<size_t>(&Scanner::getReceiverDiameter, py::const_), py::arg("index"))
            .def("set_specific_receiver_diameter", py::overload_cast<double, size_t>(&Scanner::setReceiverDiameter), py::arg("value"), py::arg("index"))
            
            .def("get_specific_visibility", py::overload_cast<size_t>(&Scanner::getVisibility, py::const_), py::arg("index"))
            .def("set_specific_visibility", py::overload_cast<double, size_t>(&Scanner::setVisibility), py::arg("value"), py::arg("index"))
            
            .def("get_specific_wavelength", py::overload_cast<size_t>(&Scanner::getWavelength, py::const_), py::arg("index"))
            .def("set_specific_wavelength", py::overload_cast<double, size_t>(&Scanner::setWavelength), py::arg("value"), py::arg("index"))
            .def("get_specific_atmospheric_extinction", py::overload_cast<size_t>(&Scanner::getAtmosphericExtinction, py::const_), py::arg("index"))
            .def("set_specific_atmospheric_extinction", py::overload_cast<double, size_t>(&Scanner::setAtmosphericExtinction), py::arg("value"), py::arg("index"))
            
            .def("get_specific_beam_waist_radius", py::overload_cast<size_t>(&Scanner::getBeamWaistRadius, py::const_), py::arg("index"))
            .def("set_specific_beam_waist_radius", py::overload_cast<double, size_t>(&Scanner::setBeamWaistRadius), py::arg("value"), py::arg("index"))
            
            .def("get_specific_max_nor", py::overload_cast<size_t>(&Scanner::getMaxNOR, py::const_), py::arg("index"))
            .def("set_specific_max_nor", py::overload_cast<int, size_t>(&Scanner::setMaxNOR), py::arg("value"), py::arg("index"))
    
            .def("get_head_relative_emitter_attitude", py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterAttitude, py::const_), py::arg("index"))
            .def("set_head_relative_emitter_attitude", py::overload_cast<const Rotation&, size_t>(&Scanner::setHeadRelativeEmitterAttitude), py::arg("value"), py::arg("index"))
            
            .def("get_head_relative_emitter_position", py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterPosition, py::const_), py::arg("index"))
            .def("set_head_relative_emitter_position", py::overload_cast<const glm::dvec3&, size_t>(&Scanner::setHeadRelativeEmitterPosition), py::arg("value"), py::arg("index"))

            .def("get_specific_bt2", py::overload_cast<size_t>(&Scanner::getBt2, py::const_), py::arg("index"))
            .def("set_specific_bt2", py::overload_cast<double, size_t>(&Scanner::setBt2), py::arg("value"), py::arg("index"))
            
            .def("get_specific_dr2", py::overload_cast<size_t>(&Scanner::getDr2, py::const_), py::arg("index"))
            .def("set_specific_dr2", py::overload_cast<double, size_t>(&Scanner::setDr2), py::arg("value"), py::arg("index"))
            
            .def("get_specific_device_id", py::overload_cast<size_t>(&Scanner::getDeviceId, py::const_), py::arg("index"))
            .def("set_specific_device_id", py::overload_cast<std::string, size_t>(&Scanner::setDeviceId), py::arg("value"), py::arg("index"))
            .def("get_specific_detector", py::overload_cast<size_t>(&Scanner::getDetector), py::arg("index"))
            .def("set_specific_detector", py::overload_cast<std::shared_ptr<AbstractDetector>, size_t>(&Scanner::setDetector), py::arg("value"), py::arg("index"))

            .def("get_head_relative_emitter_position_by_ref", py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterPositionByRef), py::arg("index"))
            .def("get__head_relative_emitter_attitude_by_ref", py::overload_cast<size_t>(&Scanner::getHeadRelativeEmitterAttitudeByRef), py::arg("index"))

            .def("get_specific_num_time_bins", [](Scanner &self, size_t idx) { return self.getNumTimeBins(idx); }, py::arg("index"))
            .def("set_specific_num_time_bins", [](Scanner &self, int numTimeBins, size_t idx) { self.setNumTimeBins(numTimeBins, idx); }, py::arg("value"), py::arg("index"))

            .def("get_specific_peak_intensity_index", py::overload_cast<size_t>(&Scanner::getPeakIntensityIndex, py::const_), py::arg("index"))
            .def("set_specific_peak_intensity_index", py::overload_cast<int, size_t>(&Scanner::setPeakIntensityIndex), py::arg("value"), py::arg("index"))
            
            .def("get_specific_scanner_head", py::overload_cast<size_t>(&Scanner::getScannerHead), py::arg("index"))
            .def("set_specific_scanner_head", py::overload_cast<std::shared_ptr<ScannerHead>, size_t>(&Scanner::setScannerHead), py::arg("value"), py::arg("index"))

            .def("get_specific_beam_deflector", py::overload_cast<size_t>(&Scanner::getBeamDeflector), py::arg("index"))
            .def("set_specific_beam_deflector", py::overload_cast<std::shared_ptr<AbstractBeamDeflector>, size_t>(&Scanner::setBeamDeflector), py::arg("value"), py::arg("index"))

            .def_property_readonly("current_pulse_number", py::overload_cast<>(&Scanner::getCurrentPulseNumber, py::const_))
            .def_property("fms",
                [](Scanner &self) { return self.fms; },
                [](Scanner &self, const std::shared_ptr<FMSFacade> &fms) { self.fms = fms; }
            )
            .def_property("all_output_paths",
                [](Scanner &self) { return *self.allOutputPaths; },
                [](Scanner &self, py::object paths) {
                    std::vector<std::string> cpp_paths = paths.cast<std::vector<std::string>>();
                    self.allOutputPaths = std::make_shared<std::vector<std::string>>(cpp_paths);
                }
            )
            .def_property("all_measurements",
                [](Scanner &self) -> py::array {
                    if (!self.allMeasurements || self.allMeasurements->empty())
                        throw std::runtime_error("allMeasurements is null or empty");
                    
                        
                    return detail::measurements_to_numpy(*(self.allMeasurements));
                },
                [](Scanner &self, py::array arr) {
                    std::vector<Measurement> vec = detail::numpy_to_measurements(arr);
                    self.allMeasurements = std::make_shared<std::vector<Measurement>>(std::move(vec));
                }
            )
            .def_property("all_trajectories",
                [](Scanner &self) -> py::array {
                    if (!self.allTrajectories  || self.allTrajectories->empty())
                        return py::array();
                    return detail::trajectories_to_numpy(*(self.allTrajectories));
                },
                [](Scanner &self, py::array arr) {
                    std::vector<Trajectory> vec = detail::numpy_to_trajectories(arr);
                    self.allTrajectories = std::make_shared<std::vector<Trajectory>>(std::move(vec));
                }
            )
            .def_property("cycle_measurements",
                [](Scanner &self) -> py::array {
                    if (!self.cycleMeasurements  || self.cycleMeasurements->empty())
                        throw std::runtime_error("cycleMeasurements is null");
                    return detail::measurements_to_numpy(*(self.cycleMeasurements));
                },
                [](Scanner &self, py::array arr) {
                    std::vector<Measurement> vec = detail::numpy_to_measurements(arr);
                    self.cycleMeasurements = std::make_shared<std::vector<Measurement>>(std::move(vec));
                }
            )
            .def_property("cycle_trajectories",
                [](Scanner &self) -> py::array {
                    if (!self.cycleTrajectories  || self.cycleTrajectories->empty())
                        throw std::runtime_error("cycleTrajectories is null");
                    return detail::trajectories_to_numpy(*(self.cycleTrajectories));
                },
                [](Scanner &self, py::array arr) {
                    std::vector<Trajectory> vec = detail::numpy_to_trajectories(arr);
                    self.cycleTrajectories = std::make_shared<std::vector<Trajectory>>(std::move(vec));
                }
            )

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
            .def_property("receiver_diameter",
                        py::overload_cast<>(&Scanner::getReceiverDiameter, py::const_),
                        py::overload_cast<double>(&Scanner::setReceiverDiameter))     
            .def_property("visibility",
                        py::overload_cast<>(&Scanner::getVisibility, py::const_),
                        py::overload_cast<double>(&Scanner::setVisibility))           
            .def_property("wavelength",
                        py::overload_cast<>(&Scanner::getWavelength, py::const_),
                        py::overload_cast<double>(&Scanner::setWavelength))
            .def_property("atmospheric_extinction", 
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
        
            .def_property_readonly("scanner_head",
                        py::overload_cast<>(&Scanner::getScannerHead))

            .def_property_readonly("beam_deflector",
                        py::overload_cast<>(&Scanner::getBeamDeflector))
            
            .def_property("detector",
                        py::overload_cast<>(&Scanner::getDetector),
                        py::overload_cast<std::shared_ptr<AbstractDetector>>(&Scanner::setDetector))

            .def_property("num_time_bins",
                        py::overload_cast<>(&Scanner::getNumTimeBins, py::const_),
                        py::overload_cast<int>(&Scanner::setNumTimeBins))
                 
            .def_property("peak_intensity_index",
                        py::overload_cast<>(&Scanner::getPeakIntensityIndex, py::const_),
                        py::overload_cast<int>(&Scanner::setPeakIntensityIndex))

            .def_property("pulse_freq_hz", &Scanner::getPulseFreq_Hz, &Scanner::setPulseFreq_Hz)    
            .def_property("is_state_active", &Scanner::isActive, &Scanner::setActive)
            .def_property("write_wave_form", &Scanner::isWriteWaveform, &Scanner::setWriteWaveform)
            .def_property("calc_echowidth", &Scanner::isCalcEchowidth, &Scanner::setCalcEchowidth)
            .def_property("full_wave_noise", &Scanner::isFullWaveNoise, &Scanner::setFullWaveNoise)
            .def_property("platform_noise_disabled", &Scanner::isPlatformNoiseDisabled, &Scanner::setPlatformNoiseDisabled)
            .def_property("fixed_incidence_angle", &Scanner::isFixedIncidenceAngle, &Scanner::setFixedIncidenceAngle)
            .def_property("id", &Scanner::getScannerId, &Scanner::setScannerId)
            .def_property("FWF_settings", 
                        py::overload_cast<>(&Scanner::getFWFSettings),
                        py::overload_cast<const FWFSettings&>(&Scanner::setFWFSettings))
            .def_property_readonly("num_devices", &Scanner::getNumDevices)
            .def_property_readonly("time_wave", py::overload_cast<>(&Scanner::getTimeWave))

            .def_property(
                "cycle_measurements_mutex",
                [](ScannerWrap &self) {
                    if (!self.get_cycle_measurements_mutex()) {
                        self.set_cycle_measurements_mutex(std::make_shared<std::mutex>());
                    }
                    return self.get_cycle_measurements_mutex();
                },
                [](ScannerWrap &self, py::object mutex_obj) {
                    if (mutex_obj.is_none()) {
                        self.set_cycle_measurements_mutex(nullptr);
                    } else {
                        auto mutex_ptr = mutex_obj.cast<std::shared_ptr<std::mutex>>();
                        self.set_cycle_measurements_mutex(mutex_ptr);
                    }
                },
                "A shared mutex for synchronized operations"
            )
            .def_property(
                "all_measurements_mutex",
                [](ScannerWrap &self) {
                    if (!self.get_all_measurements_mutex()) {
                        self.set_all_measurements_mutex(std::make_shared<std::mutex>());
                    }
                    return self.get_all_measurements_mutex();
                },
                [](ScannerWrap &self, py::object mutex_obj) {
                    if (mutex_obj.is_none()) {
                        self.set_all_measurements_mutex(nullptr);
                    } else {
                        auto mutex_ptr = mutex_obj.cast<std::shared_ptr<std::mutex>>();
                        self.set_all_measurements_mutex(mutex_ptr);
                    }
                },
                "A shared mutex for synchronized operations"
            );
      

        py::class_<PyHeliosSimulation> helios_simulation (m, "PyheliosSimulation");
        helios_simulation
            .def(py::init<>())
            .def(py::init<
                    std::string,
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
                    int
                >(),
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
                py::arg("warehouseFactor") = 1
            )
            .def("start", &PyHeliosSimulation::start)
            .def("pause", &PyHeliosSimulation::pause)
            .def("stop", &PyHeliosSimulation::stop)
            .def("resume", &PyHeliosSimulation::resume)
            .def("join", &PyHeliosSimulation::join)
            .def("load_survey", &PyHeliosSimulation::loadSurvey,
                 py::arg("legNoiseDisabled") = false,
                 py::arg("rebuildScene") = false,
                 py::arg("writeWaveform") = false,
                 py::arg("calcEchowidth") = false,
                 py::arg("fullWaveNoise") = false,
                 py::arg("platformNoiseDisabled") = true,
                 py::arg("writePulse") = false)
            .def("add_rotate_filter", &PyHeliosSimulation::addRotateFilter,
                 py::arg("q0"), py::arg("q1"), py::arg("q2"), py::arg("q3"), py::arg("partId"))
            .def("add_scale_filter", &PyHeliosSimulation::addScaleFilter,
                 py::arg("scaleFactor"), py::arg("partId"))
            .def("add_translate_filter", &PyHeliosSimulation::addTranslateFilter,
                 py::arg("x"), py::arg("y"), py::arg("z"), py::arg("partId"))
            .def("copy", &PyHeliosSimulation::copy)
            .def("callback", &PyHeliosSimulation::setCallback)
            .def("assoc_leg_with_scanning_strip", &PyHeliosSimulation::assocLegWithScanningStrip)
            .def("remove_leg", &PyHeliosSimulation::removeLeg)
            .def("new_leg", &PyHeliosSimulation::newLeg, py::return_value_policy::reference)
            .def("new_leg_from_template", &PyHeliosSimulation::newLegFromTemplate, py::return_value_policy::reference)
            .def("new_scanning_strip", &PyHeliosSimulation::newScanningStrip)
            .def("clear_callback", &PyHeliosSimulation::clearCallback)
            .def("get_leg", &PyHeliosSimulation::getLeg, py::return_value_policy::reference)

            .def_property("final_output",
                          [](const PyHeliosSimulation &self) { return self.finalOutput; },
                          [](PyHeliosSimulation &self, bool value) { self.finalOutput = value; })
            .def_property("legacy_energy_model",
                          [](const PyHeliosSimulation &self) { return self.legacyEnergyModel; },
                          [](PyHeliosSimulation &self, bool value) { self.legacyEnergyModel = value; })
            .def_property("export_to_file",
                          [](const PyHeliosSimulation &self) { return self.exportToFile; },
                          [](PyHeliosSimulation &self, bool value) { self.exportToFile = value; })

            .def_property_readonly("is_started", &PyHeliosSimulation::isStarted)
            .def_property_readonly("is_paused", &PyHeliosSimulation::isPaused)
            .def_property_readonly("is_stopped", &PyHeliosSimulation::isStopped)
            .def_property_readonly("is_finished", &PyHeliosSimulation::isFinished)
            .def_property_readonly("is_running", &PyHeliosSimulation::isRunning)
            .def_property_readonly("survey_path", &PyHeliosSimulation::getSurveyPath)
            .def_property_readonly("assets_path", &PyHeliosSimulation::getAssetsPath)
            .def_property("survey", &PyHeliosSimulation::getSurvey, &PyHeliosSimulation::setSurvey)
            .def_property_readonly("scanner", &PyHeliosSimulation::getScanner)
            .def_property_readonly("platform", &PyHeliosSimulation::getPlatform)
            .def_property_readonly("scene", &PyHeliosSimulation::getScene)
            .def_property_readonly("num_legs", &PyHeliosSimulation::getNumLegs)

            .def_property("num_threads", &PyHeliosSimulation::getNumThreads, &PyHeliosSimulation::setNumThreads)
            .def_property("callback_frequency", &PyHeliosSimulation::getCallbackFrequency, &PyHeliosSimulation::setCallbackFrequency)
            .def_property("simulation_frequency", &PyHeliosSimulation::getSimFrequency, &PyHeliosSimulation::setSimFrequency)
            .def_property("dyn_scene_step", &PyHeliosSimulation::getDynSceneStep, &PyHeliosSimulation::setDynSceneStep)
            .def_property("fixed_gps_time_start", &PyHeliosSimulation::getFixedGpsTimeStart, &PyHeliosSimulation::setFixedGpsTimeStart)
            .def_property("las_output", &PyHeliosSimulation::getLasOutput, &PyHeliosSimulation::setLasOutput)
            .def_property("las10", &PyHeliosSimulation::getLas10, &PyHeliosSimulation::setLas10)
            .def_property("zip_output", &PyHeliosSimulation::getZipOutput, &PyHeliosSimulation::setZipOutput)
            .def_property("split_by_channel", &PyHeliosSimulation::getSplitByChannel, &PyHeliosSimulation::setSplitByChannel)
            .def_property("las_scale", &PyHeliosSimulation::getLasScale, &PyHeliosSimulation::setLasScale)
            .def_property("kdt_factory", &PyHeliosSimulation::getKDTFactory, &PyHeliosSimulation::setKDTFactory)
            .def_property("kdt_jobs", &PyHeliosSimulation::getKDTJobs, &PyHeliosSimulation::setKDTJobs)
            .def_property("kdt_SAH_loss_nodes", &PyHeliosSimulation::getKDTSAHLossNodes, &PyHeliosSimulation::setKDTSAHLossNodes)
            .def_property("parallelization_strategy", &PyHeliosSimulation::getParallelizationStrategy, &PyHeliosSimulation::setParallelizationStrategy)
            .def_property("chunk_size", &PyHeliosSimulation::getChunkSize, &PyHeliosSimulation::setChunkSize)
            .def_property("warehouse_factor", &PyHeliosSimulation::getWarehouseFactor, &PyHeliosSimulation::setWarehouseFactor);


            py::class_<SingleScanner, Scanner, std::shared_ptr<SingleScanner>> single_scanner(m, "SingleScanner");
            single_scanner
                .def(py::init<double, glm::dvec3, Rotation, std::list<int>, double, std::string, double, double, double, double, double, int, bool, bool, bool, bool, bool>(),
                    py::arg("beam_div_rad"),
                    py::arg("beam_origin"),
                    py::arg("beam_orientation"),
                    py::arg("pulse_freqs"),
                    py::arg("pulse_length"),
                    py::arg("id"),
                    py::arg("average_power"),
                    py::arg("beam_quality"),
                    py::arg("efficiency"),
                    py::arg("receiver_diameter"),
                    py::arg("atmospheric_visibility"),
                    py::arg("wavelength"),
                    py::arg("write_waveform") = false,
                    py::arg("write_pulse") = false,
                    py::arg("calc_echowidth") = false,
                    py::arg("full_wave_noise") = false,
                    py::arg("platform_noise_disabled") = false
                )
                
                .def_property(
                    "supported_pulse_freqs_hz",
                    [](SingleScanner &self) {
                        return self.getSupportedPulseFreqs_Hz(0);  
                    },
                    [](SingleScanner &self, std::list<int> pulse_freqs) {
                        self.setSupportedPulseFreqs_Hz(pulse_freqs, 0); 
                    },
                    py::return_value_policy::reference_internal
                )
                .def_property("pulse_freq_hz", &SingleScanner::getPulseFreq_Hz, &SingleScanner::setPulseFreq_Hz)

                .def(py::init<SingleScanner&>(), py::arg("scanner"))
                .def("apply_settings", &SingleScanner::applySettings, py::arg("settings"), py::arg("idx") = 0)
                .def("do_sim_step", &SingleScanner::doSimStep, py::arg("leg_index"), py::arg("current_gps_time"))
                .def("calc_rays_number", &SingleScanner::calcRaysNumber, py::arg("idx") = 0)
                .def("prepare_discretization", &SingleScanner::prepareDiscretization, py::arg("idx") = 0)
                .def("calc_atmospheric_attenuation", py::overload_cast<size_t>(&SingleScanner::calcAtmosphericAttenuation, py::const_))
                .def("check_max_NOR", py::overload_cast<int, size_t>(&SingleScanner::checkMaxNOR))
                .def("calc_absolute_beam_attitude", py::overload_cast<size_t>(&SingleScanner::calcAbsoluteBeamAttitude))       
                .def("get_specific_current_pulse_number", py::overload_cast<size_t>(&SingleScanner::getCurrentPulseNumber, py::const_))
                .def("get_specific_num_rays", py::overload_cast<size_t>(&SingleScanner::getNumRays, py::const_))
                .def("set_specific_num_rays", py::overload_cast<int, size_t>(&SingleScanner::setNumRays))
                .def("get_specific_pulse_length", py::overload_cast<size_t>(&SingleScanner::getPulseLength_ns, py::const_))
                .def("set_specific_pulse_length", py::overload_cast<double, size_t>(&SingleScanner::setPulseLength_ns))
                .def("get_specific_last_pulse_was_hit", py::overload_cast<size_t>(&SingleScanner::lastPulseWasHit, py::const_))
                .def("set_specific_last_pulse_was_hit", py::overload_cast<bool, size_t>(&SingleScanner::setLastPulseWasHit))
                .def("get_specific_beam_divergence", py::overload_cast<size_t>(&SingleScanner::getBeamDivergence, py::const_))
                .def("set_specific_beam_divergence", py::overload_cast<double, size_t>(&SingleScanner::setBeamDivergence))

                .def("get_max_nor", py::overload_cast<size_t>(&SingleScanner::getMaxNOR, py::const_), py::arg("index"))
                .def("set_max_nor", py::overload_cast<int, size_t>(&SingleScanner::setMaxNOR), py::arg("value"), py::arg("index"))

                .def("get_specific_average_power", py::overload_cast<size_t>(&SingleScanner::getAveragePower, py::const_), py::arg("index"))
                .def("set_specific_average_power", py::overload_cast<double, size_t>(&SingleScanner::setAveragePower), py::arg("value"), py::arg("index"))
                
                .def("get_specific_beam_quality", py::overload_cast<size_t>(&SingleScanner::getBeamQuality, py::const_), py::arg("index"))
                .def("set_specific_beam_quality", py::overload_cast<double, size_t>(&SingleScanner::setBeamQuality), py::arg("value"), py::arg("index"))

                .def("get_specific_efficiency", py::overload_cast<size_t>(&SingleScanner::getEfficiency, py::const_), py::arg("index"))
                .def("set_specific_efficiency", py::overload_cast<double, size_t>(&SingleScanner::setEfficiency), py::arg("value"), py::arg("index"))

                .def("get_specific_receiver_diameter", py::overload_cast<size_t>(&SingleScanner::getReceiverDiameter, py::const_), py::arg("index"))
                .def("set_specific_receiver_diameter", py::overload_cast<double, size_t>(&SingleScanner::setReceiverDiameter), py::arg("value"), py::arg("index"))

                .def("get_specific_visibility", py::overload_cast<size_t>(&SingleScanner::getVisibility, py::const_), py::arg("index"))
                .def("set_specific_visibility", py::overload_cast<double, size_t>(&SingleScanner::setVisibility), py::arg("value"), py::arg("index"))

                .def("get_specific_wavelength", py::overload_cast<size_t>(&SingleScanner::getWavelength, py::const_), py::arg("index"))
                .def("set_specific_wavelength", py::overload_cast<double, size_t>(&SingleScanner::setWavelength), py::arg("value"), py::arg("index"))

                .def("get_specific_atmospheric_extinction", py::overload_cast<size_t>(&SingleScanner::getAtmosphericExtinction, py::const_), py::arg("index"))
                .def("set_specific_atmospheric_extinction", py::overload_cast<double, size_t>(&SingleScanner::setAtmosphericExtinction), py::arg("value"), py::arg("index"))

                .def("get_specific_beam_waist_radius", py::overload_cast<size_t>(&SingleScanner::getBeamWaistRadius, py::const_), py::arg("index"))
                .def("set_specific_beam_waist_radius", py::overload_cast<double, size_t>(&SingleScanner::setBeamWaistRadius), py::arg("value"), py::arg("index"))

                .def("get_head_relative_emitter_attitude", py::overload_cast<size_t>(&SingleScanner::getHeadRelativeEmitterAttitude, py::const_), py::arg("index"))
                .def("set_head_relative_emitter_attitude", py::overload_cast<const Rotation&, size_t>(&SingleScanner::setHeadRelativeEmitterAttitude), py::arg("value"), py::arg("index"))

                .def("get_head_relative_emitter_position", 
                    &SingleScanner::getHeadRelativeEmitterPosition, 
                    py::arg("index") )

                .def("set_head_relative_emitter_position", 
                    &SingleScanner::setHeadRelativeEmitterPosition, 
                    py::arg("position"), py::arg("index"))
                    
                .def("get_head_relative_emitter_position_by_ref", 
                    &SingleScanner::getHeadRelativeEmitterPositionByRef, 
                    py::return_value_policy::reference_internal, 
                    py::arg("index") )

                .def("get_head_relative_emitter_attitude_by_ref", 
                    &SingleScanner::getHeadRelativeEmitterAttitudeByRef, 
                    py::return_value_policy::reference_internal, 
                    py::arg("index"))

                .def("get_bt2", &SingleScanner::getBt2, py::arg("index") )
                .def("set_bt2", &SingleScanner::setBt2, py::arg("value"), py::arg("index") )

                .def("get_dr2", &SingleScanner::getDr2, py::arg("index") )
                .def("set_dr2", &SingleScanner::setDr2, py::arg("value"), py::arg("index") )

                .def("get_detector", &SingleScanner::getDetector, py::arg("index") )
                .def("set_detector", &SingleScanner::setDetector, py::arg("detector"), py::arg("index") )

                .def("get_device_id", &SingleScanner::getDeviceId, py::arg("index"))
                .def("set_device_id", &SingleScanner::setDeviceId, py::arg("device_id"), py::arg("index") )

                .def("get_fwf_settings", &SingleScanner::getFWFSettings, py::arg("index") )
                .def("set_fwf_settings", &SingleScanner::setFWFSettings, py::arg("fwf_settings"), py::arg("index") )

                .def("get_num_time_bins", &SingleScanner::getNumTimeBins, py::arg("index") )
                .def("set_num_time_bins", &SingleScanner::setNumTimeBins, py::arg("value"), py::arg("index"))

                .def("get_time_wave", &SingleScanner::getTimeWave, py::arg("index") )
                .def("set_time_wave",
                        static_cast<void (SingleScanner::*)(std::vector<double>&, size_t)>(&SingleScanner::setTimeWave),
                        py::arg("time_wave"), py::arg("index"))
                    
                .def("set_time_wave",
                        [](SingleScanner& self, std::vector<double> time_wave, size_t index) {
                            self.setTimeWave(std::move(time_wave), index);
                        },
                        py::arg("time_wave"), py::arg("index"))
                .def("get_peak_intensity_index", &SingleScanner::getPeakIntensityIndex, py::arg("index") )
                .def("set_peak_intensity_index", &SingleScanner::setPeakIntensityIndex, py::arg("value"), py::arg("index") )

                .def("get_scanner_head", &SingleScanner::getScannerHead, py::arg("index") )
                .def("set_scanner_head", &SingleScanner::setScannerHead, py::arg("scanner_head"), py::arg("index") )

                .def("get_beam_deflector", &SingleScanner::getBeamDeflector, py::arg("index") )
                .def("set_beam_deflector", &SingleScanner::setBeamDeflector, py::arg("beam_deflector"), py::arg("index") )
                
                .def("get_max_NOR", &SingleScanner::getMaxNOR, py::arg("index") )
                .def("set_max_NOR", &SingleScanner::setMaxNOR, py::arg("value"), py::arg("index") )
                .def("clone", &SingleScanner::clone);

            
            py::class_<MultiScanner, Scanner, std::shared_ptr<MultiScanner>> multi_scanner(m, "MultiScanner");
            multi_scanner
               .def(py::init<std::vector<ScanningDevice>, std::string, const std::list<int>&, bool, bool, bool, bool, bool>(),
                    py::arg("scan_devs"),
                    py::arg("id"),
                    py::arg("pulse_freqs"),
                    py::arg("write_waveform") = false,
                    py::arg("write_pulse") = false,
                    py::arg("calc_echowidth") = false,
                    py::arg("full_wave_noise") = false,
                    py::arg("platform_noise_disabled") = false)
                .def(py::init<std::string, const std::list<int>&, bool, bool, bool, bool>(),
                    py::arg("id"),
                    py::arg("pulse_freqs"),
                    py::arg("write_waveform") = false,
                    py::arg("calc_echowidth") = false,
                    py::arg("full_wave_noise") = false,
                    py::arg("platform_noise_disabled") = false)
                
                .def("on_leg_complete", &MultiScanner::onLegComplete)
                .def("prepare_simulation", &MultiScanner::prepareSimulation, py::arg("legacy_energy_model") = 0)
                .def("apply_settings", &MultiScanner::applySettings, py::arg("settings"), py::arg("idx"))
                .def("do_sim_step", &MultiScanner::doSimStep, py::arg("leg_index"), py::arg("current_gps_time"))
                .def("calc_rays_number", &MultiScanner::calcRaysNumber, py::arg("idx"))
                .def("prepare_discretization", &MultiScanner::prepareDiscretization, py::arg("idx"))
                .def("calc_absolute_beam_attitude", &MultiScanner::calcAbsoluteBeamAttitude, py::arg("idx"))
                .def("calc_atmospheric_attenuation", &MultiScanner::calcAtmosphericAttenuation, py::arg("idx"))
                .def("check_max_nor", &MultiScanner::checkMaxNOR, py::arg("nor"), py::arg("idx"))
                .def("compute_subrays", &MultiScanner::computeSubrays)
                .def("initialize_full_waveform", &MultiScanner::initializeFullWaveform,
                    py::arg("min_hit_dist_m"), py::arg("max_hit_dist_m"),
                    py::arg("min_hit_time_ns"), py::arg("max_hit_time_ns"),
                    py::arg("ns_per_bin"), py::arg("distance_threshold"),
                    py::arg("peak_intensity_index"), py::arg("num_fullwave_bins"), py::arg("idx"))

                .def("calc_intensity", 
                    py::overload_cast<double, double, const Material&, int, size_t>(&MultiScanner::calcIntensity, py::const_))

                .def("calc_intensity", 
                    py::overload_cast<double, double, int, size_t>(&MultiScanner::calcIntensity, py::const_))

                .def("set_device_index", &MultiScanner::setDeviceIndex, py::arg("new_idx"), py::arg("old_idx"))
                 

                .def("get_device_id", &MultiScanner::getDeviceId, py::arg("idx"))
                .def("set_device_id", &MultiScanner::setDeviceId, py::arg("device_id"), py::arg("idx"))

                .def("get_pulse_length", &MultiScanner::getPulseLength_ns, py::arg("idx"))
                .def("set_pulse_length", &MultiScanner::setPulseLength_ns, py::arg("pulse_length"), py::arg("idx"))

                .def("get_beam_divergence", &MultiScanner::getBeamDivergence, py::arg("idx"))
                .def("set_beam_divergence", &MultiScanner::setBeamDivergence, py::arg("beam_divergence"), py::arg("idx"))

                .def("get_average_power", &MultiScanner::getAveragePower, py::arg("idx"))
                .def("set_average_power", &MultiScanner::setAveragePower, py::arg("average_power"), py::arg("idx"))

                .def("get_num_rays", &MultiScanner::getNumRays, py::arg("idx"))
                .def("set_num_rays", &MultiScanner::setNumRays, py::arg("num_rays"), py::arg("idx"))

                .def("get_last_pulse_was_hit", &MultiScanner::lastPulseWasHit, py::arg("idx"))
                .def("set_last_pulse_was_hit", &MultiScanner::setLastPulseWasHit, py::arg("last_pulse_was_hit"), py::arg("idx"))

                .def("get_scanner_head", &MultiScanner::getScannerHead, py::arg("idx"))
                .def("set_scanner_head", &MultiScanner::setScannerHead, py::arg("scanner_head"), py::arg("idx"))

                .def("get_beam_deflector", &MultiScanner::getBeamDeflector, py::arg("idx"))
                .def("set_beam_deflector", &MultiScanner::setBeamDeflector, py::arg("beam_deflector"), py::arg("idx"))

                .def("get_detector", &MultiScanner::getDetector, py::arg("idx"))
                .def("set_detector", &MultiScanner::setDetector, py::arg("detector"), py::arg("idx"))

                .def("get_fwf_settings", &MultiScanner::getFWFSettings, py::arg("idx"))
                .def("set_fwf_settings", &MultiScanner::setFWFSettings, py::arg("fwf_settings"), py::arg("idx"))

                .def("get_beam_quality", &MultiScanner::getBeamQuality, py::arg("idx"))
                .def("set_beam_quality", &MultiScanner::setBeamQuality, py::arg("beam_quality"), py::arg("idx"))

                .def("get_efficiency", &MultiScanner::getEfficiency, py::arg("idx"))
                .def("set_efficiency", &MultiScanner::setEfficiency, py::arg("efficiency"), py::arg("idx"))

                .def("get_receiver_diameter", &MultiScanner::getReceiverDiameter, py::arg("idx"))
                .def("set_receiver_diameter", &MultiScanner::setReceiverDiameter, py::arg("receiver_diameter"), py::arg("idx"))

                .def("get_visibility", &MultiScanner::getVisibility, py::arg("idx"))
                .def("set_visibility", &MultiScanner::setVisibility, py::arg("visibility"), py::arg("idx"))

                .def("get_wavelength", &MultiScanner::getWavelength, py::arg("idx"))
                .def("set_wavelength", &MultiScanner::setWavelength, py::arg("wavelength"), py::arg("idx"))

                .def("get_atmospheric_extinction", &MultiScanner::getAtmosphericExtinction, py::arg("idx"))
                .def("set_atmospheric_extinction", &MultiScanner::setAtmosphericExtinction, py::arg("atmospheric_extinction"), py::arg("idx"))

                .def("get_beam_waist_radius", &MultiScanner::getBeamWaistRadius, py::arg("idx"))
                .def("set_beam_waist_radius", &MultiScanner::setBeamWaistRadius, py::arg("beam_waist_radius"), py::arg("idx"))

                .def("get_head_relative_emitter_position", &MultiScanner::getHeadRelativeEmitterPosition, py::arg("idx"))
                .def("set_head_relative_emitter_position", &MultiScanner::setHeadRelativeEmitterPosition, py::arg("pos"), py::arg("idx"))

                .def("get_head_relative_emitter_attitude", &MultiScanner::getHeadRelativeEmitterAttitude, py::arg("idx"))
                .def("set_head_relative_emitter_attitude", &MultiScanner::setHeadRelativeEmitterAttitude, py::arg("attitude"), py::arg("idx"))

                .def("get_head_relative_emitter_position_by_ref", &MultiScanner::getHeadRelativeEmitterPositionByRef, py::arg("idx") = 0)
                .def("get_head_relative_emitter_attitude_by_ref", &MultiScanner::getHeadRelativeEmitterAttitudeByRef, py::arg("idx") = 0)
                
                .def("get_time_wave", &MultiScanner::getTimeWave, py::arg("index") )
                .def("set_time_wave",
                        static_cast<void (MultiScanner::*)(std::vector<double>&, size_t)>(&MultiScanner::setTimeWave),
                        py::arg("time_wave"), py::arg("index"))
                    
                .def("set_time_wave",
                        [](MultiScanner& self, std::vector<double> time_wave, size_t index) {
                            self.setTimeWave(std::move(time_wave), index);
                        },
                        py::arg("time_wave"), py::arg("index"))
                .def("get_bt2", &MultiScanner::getBt2, py::arg("idx"))
                .def("set_bt2", &MultiScanner::setBt2, py::arg("bt2"), py::arg("idx"))

                .def("get_dr2", &MultiScanner::getDr2, py::arg("idx"))
                .def("set_dr2", &MultiScanner::setDr2, py::arg("dr2"), py::arg("idx"))

                .def("get_current_pulse_number", &MultiScanner::getCurrentPulseNumber, py::arg("idx"))

                .def("get_pulse_freqs", &MultiScanner::getSupportedPulseFreqs_Hz, py::arg("idx"))
                .def("set_pulse_freqs", &MultiScanner::setSupportedPulseFreqs_Hz, py::arg("pulse_freqs_hz"), py::arg("idx"))

                .def("get_max_nor", &MultiScanner::getMaxNOR, py::arg("idx"))
                .def("set_max_nor", &MultiScanner::setMaxNOR, py::arg("max_nor"), py::arg("idx"))

                .def("get_num_time_bins", &MultiScanner::getNumTimeBins, py::arg("idx"))
                .def("set_num_time_bins", &MultiScanner::setNumTimeBins, py::arg("num_time_bins"), py::arg("idx"))

                .def("get_peak_intensity_index", &MultiScanner::getPeakIntensityIndex, py::arg("idx"))
                .def("set_peak_intensity_index", &MultiScanner::setPeakIntensityIndex, py::arg("peak_intensity_index"), py::arg("idx"))
                
                .def("get_received_energy_min", &MultiScanner::getReceivedEnergyMin, py::arg("idx"))
                .def("set_received_energy_min", &MultiScanner::setReceivedEnergyMin, py::arg("received_energy_min"), py::arg("idx"))
                .def("clone", &MultiScanner::clone);


            py::class_<Simulation, SimulationWrap, std::shared_ptr<Simulation>> simulation(m, "Simulation");
            simulation
            
                .def(py::init<int, std::shared_ptr<PulseThreadPoolInterface>, int, std::string, bool>(),
                    py::arg("parallelizationStrategy"),
                    py::arg("pulseThreadPoolInterface"),
                    py::arg("chunkSize"),
                    py::arg("fixedGpsTimeStart") = "",
                    py::arg("legacyEnergyModel") = false
                )

                .def_readwrite("current_leg_index", &Simulation::mCurrentLegIndex)
                .def_readwrite("callback", &Simulation::callback)
                .def_readwrite("is_finished", &Simulation::finished)

                .def_property("sim_speed_factor", &Simulation::getSimSpeedFactor, &Simulation::setSimSpeedFactor)
                .def_property("scanner", &Simulation::getScanner, &Simulation::setScanner)
                .def_property("simulation_frequency", &Simulation::getSimFrequency, &Simulation::setSimFrequency)
                .def_property("callback_frequency", &Simulation::getCallbackFrequency, &Simulation::setCallbackFrequency)

                .def("start", &Simulation::start)
                .def("pause", &Simulation::pause)
                .def("stop", &Simulation::stop);
            

            py::class_<FMSFacade, std::shared_ptr<FMSFacade>> fms_facade(m, "FMSFacade");
            fms_facade
                .def(py::init<>())
                
                .def_readwrite("factory", &FMSFacade::factory)
                .def_readwrite("read", &FMSFacade::read)
                .def_readwrite("write", &FMSFacade::write)
                .def_readwrite("serialization", &FMSFacade::serialization)
                
                .def("disconnect", &FMSFacade::disconnect);


            py::class_<FMSWriteFacade, std::shared_ptr<FMSWriteFacade>> fms_write_facade(m, "FMSWriteFacade");
            fms_write_facade
                .def(py::init<>())
                .def("disconnect", &FMSWriteFacade::disconnect) 
                .def("get_measurement_writer_output_path", [](FMSWriteFacade& self){ return self.getMeasurementWriterOutputPath().string(); });

            py::class_<FMSFacadeFactory, std::shared_ptr<FMSFacadeFactory>> fms_facade_factory(m, "FMSFacadeFactory");
            fms_facade_factory
                .def(py::init<>())

                .def("build_facade", py::overload_cast<const std::string&, double, bool, bool, bool, bool, Survey&, bool>(&FMSFacadeFactory::buildFacade),
                    py::arg("outdir"),
                    py::arg("lasScale"),
                    py::arg("lasOutput"),
                    py::arg("las10"),
                    py::arg("zipOutput"),
                    py::arg("splitByChannel"),
                    py::arg("survey"),
                    py::arg("updateSurvey") = true)
                
                .def("build_facade", py::overload_cast<const std::string&, double, bool, bool, bool, Survey&, bool>(&FMSFacadeFactory::buildFacade),
                    py::arg("outdir"),
                    py::arg("lasScale"),
                    py::arg("lasOutput"),
                    py::arg("las10"),
                    py::arg("zipOutput"),
                    py::arg("survey"),
                    py::arg("updateSurvey") = true);


            py::class_<SurveyPlayback, Simulation, std::shared_ptr<SurveyPlayback>> survey_playback(m, "SurveyPlayback");
            survey_playback
                .def(py::init<std::shared_ptr<Survey>, int, std::shared_ptr<PulseThreadPoolInterface>, int, std::string, bool, bool, bool, std::shared_ptr<FMSFacade>>(),
                    py::arg("survey"),
                    py::arg("parallelizationStrategy"),
                    py::arg("pulseThreadPoolInterface"),
                    py::arg("chunkSize"),
                    py::arg("fixedGpsTimeStart"),
                    py::arg("legacyEnergyModel"),
                    py::arg("exportToFile") = true,
                    py::arg("disableShutdown") = false,
                    py::arg("fms") = nullptr
                )

                .def_readwrite("fms", &SurveyPlayback::fms)
                .def_readwrite("survey", &SurveyPlayback::mSurvey)

                .def("do_sim_step", &SurveyPlayback::doSimStep);


            py::class_<PulseThreadPoolFactory,  std::shared_ptr<PulseThreadPoolFactory>>(m, "PulseThreadPoolFactory")
                .def(py::init<int, std::size_t, double, int, int>(),
                    py::arg("parallelizationStrategy"),
                    py::arg("poolSize"),
                    py::arg("deviceAccuracy"),
                    py::arg("chunkSize"),
                    py::arg("warehouseFactor") = 1)
                .def("make_pulse_thread_pool", &PulseThreadPoolFactory::makePulseThreadPool)
                .def("make_basic_pulse_thread_pool", &PulseThreadPoolFactory::makeBasicPulseThreadPool)
                .def("make_pulse_warehouse_thread_pool", &PulseThreadPoolFactory::makePulseWarehouseThreadPool);
                

            py::class_<PulseThreadPoolInterface, PulseThreadPoolInterfaceWrap, std::shared_ptr<PulseThreadPoolInterface>>(m, "PulseThreadPoolInterface")
                .def(py::init<>())
                .def("run_pulse_task", &PulseThreadPoolInterface::run_pulse_task)
                .def("try_run_pulse_task", &PulseThreadPoolInterface::try_run_pulse_task)
                .def("join", &PulseThreadPoolInterface::join);
            
            
            py::class_<PulseWarehouseThreadPool, PulseThreadPoolInterface, std::shared_ptr<PulseWarehouseThreadPool>>(m, "PulseWarehouseThreadPool")
                .def(py::init<std::size_t, double, std::size_t>(),
                    py::arg("_pool_size"),
                    py::arg("deviceAccuracy"),
                    py::arg("maxTasks") = 256)
                .def("run_pulse_task", &PulseWarehouseThreadPool::run_pulse_task,
                    py::arg("dropper"))
                .def("try_run_pulse_task", &PulseWarehouseThreadPool::try_run_pulse_task,
                    py::arg("dropper"))
                .def("join", &PulseWarehouseThreadPool::join); 
            

            py::class_<TaskDropper<PulseTask, PulseThreadPoolInterface, 
                       std::vector<std::vector<double>>&,
                       RandomnessGenerator<double>&, 
                       RandomnessGenerator<double>&, 
                       NoiseSource<double>&>>(m, "PulseTaskDropper")
                .def(py::init<size_t>())
                
                .def("add", (bool (TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&>::*)(std::shared_ptr<PulseTask>)) 
                                &TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&>::add)
                
                .def("drop", (void (TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&>::*)()) 
                                &TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&>::drop)

                .def("drop", (void (TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&>::*)(std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                                NoiseSource<double>&)) 
                                &TaskDropper<PulseTask, PulseThreadPoolInterface, 
                                std::vector<std::vector<double>>&,
                                RandomnessGenerator<double>&, 
                                RandomnessGenerator<double>&, 
                       NoiseSource<double>&>::drop);


            py::class_<ScanningPulseProcess, ScanningPulseProcessWrap, std::shared_ptr<ScanningPulseProcess>>(m, "ScanningPulseProcess")
                .def(py::init<std::shared_ptr<Scanner>>())  
                .def("handle_pulse_computation", &ScanningPulseProcess::handlePulseComputation)
                .def("on_leg_complete", &ScanningPulseProcess::onLegComplete)
                .def("on_simulation_finished", &ScanningPulseProcess::onSimulationFinished)
                .def("get_scanner", &ScanningPulseProcess::getScanner)
                .def("is_write_waveform", &ScanningPulseProcess::isWriteWaveform)
                .def("is_calc_echowidth", &ScanningPulseProcess::isCalcEchowidth)
                .def("get_all_measurements", &ScanningPulseProcess::getAllMeasurements, py::return_value_policy::reference_internal)
                .def("get_all_measurements_mutex", &ScanningPulseProcess::getAllMeasurementsMutex, py::return_value_policy::reference_internal)
                .def("get_cycle_measurements", &ScanningPulseProcess::getCycleMeasurements, py::return_value_policy::reference_internal)
                .def("get_cycle_measurements_mutex", &ScanningPulseProcess::getCycleMeasurementsMutex, py::return_value_policy::reference_internal);


            py::class_<BuddingScanningPulseProcess, ScanningPulseProcess, std::shared_ptr<BuddingScanningPulseProcess>>(m, "BuddingScanningPulseProcess")
                .def(py::init<
                    std::shared_ptr<Scanner>,
                    PulseTaskDropper&,
                    PulseThreadPool&,
                    RandomnessGenerator<double>&,
                    RandomnessGenerator<double>&,
                    UniformNoiseSource<double>&
#if DATA_ANALYTICS >= 2
                    , std::shared_ptr<HDA_PulseRecorder>
#endif
                >(), 
                py::arg("scanner"),
                py::arg("dropper"),
                py::arg("pool"),
                py::arg("randGen1"),
                py::arg("randGen2"),
                py::arg("intersectionHandlingNoiseSource")
#if DATA_ANALYTICS >= 2
                , py::arg("pulseRecorder")
#endif
                )
                .def("handle_pulse_computation", &BuddingScanningPulseProcess::handlePulseComputation)
                .def("on_leg_complete", &BuddingScanningPulseProcess::onLegComplete)
                .def("on_simulation_finished", &BuddingScanningPulseProcess::onSimulationFinished);
        

            py::class_<WarehouseScanningPulseProcess, ScanningPulseProcess, std::shared_ptr<WarehouseScanningPulseProcess>>(m, "WarehouseScanningPulseProcess")
                .def(py::init<
                    std::shared_ptr<Scanner>,
                    PulseTaskDropper&,
                    PulseWarehouseThreadPool&,
                    RandomnessGenerator<double>&,
                    RandomnessGenerator<double>&,
                    UniformNoiseSource<double>&
#if DATA_ANALYTICS >= 2
                    , std::shared_ptr<HDA_PulseRecorder>
#endif
                >(), 
                py::arg("scanner"),
                py::arg("dropper"),
                py::arg("pool"),
                py::arg("randGen1"),
                py::arg("randGen2"),
                py::arg("intersectionHandlingNoiseSource")
#if DATA_ANALYTICS >= 2
                , py::arg("pulseRecorder")
#endif
                )
                .def("handle_pulse_computation", &WarehouseScanningPulseProcess::handlePulseComputation)
                .def("on_leg_complete", &WarehouseScanningPulseProcess::onLegComplete)
                .def("on_simulation_finished", &WarehouseScanningPulseProcess::onSimulationFinished);
            
            
            py::class_<KDGrove, std::shared_ptr<KDGrove>> kdgrove(m, "KDGrove");
            kdgrove
                .def(py::init<size_t>(), py::arg("initTreesCapacity") = 1);


            py::class_<KDGroveFactory, std::shared_ptr<KDGroveFactory>>(m, "KDGroveFactory")
                .def(py::init<std::shared_ptr<KDTreeFactory>>(), py::arg("kdtf"));


            py::class_<KDTreeFactory, KDTreeFactoryWrap, std::shared_ptr<KDTreeFactory>>(m, "KDTreeFactory")
                .def(py::init<>());
            

            py::class_<SimpleKDTreeFactory, KDTreeFactory, std::shared_ptr<SimpleKDTreeFactory>>(m, "SimpleKDTreeFactory")
                .def(py::init<>());
            

            py::class_<SimpleKDTreeGeometricStrategy, std::shared_ptr<SimpleKDTreeGeometricStrategy>>(m, "SimpleKDTreeGeometricStrategy")
                .def(py::init<SimpleKDTreeFactory&>(), py::arg("kdtf")); 

            
            py::class_<MultiThreadKDTreeFactory, SimpleKDTreeFactory, std::shared_ptr<MultiThreadKDTreeFactory>>(m, "MultiThreadKDTreeFactory")
                .def(py::init<std::shared_ptr<SimpleKDTreeFactory>, std::shared_ptr<SimpleKDTreeGeometricStrategy>, size_t, size_t>(),
                    py::arg("kdtf"),
                    py::arg("gs"),
                    py::arg("numJobs") = 2,
                    py::arg("geomJobs") = 2);
            

           py::class_<SAHKDTreeGeometricStrategy, SimpleKDTreeGeometricStrategy, std::shared_ptr<SAHKDTreeGeometricStrategy>>(m, "SAHKDTreeGeometricStrategy")
                .def(py::init<SAHKDTreeFactory&>(), py::arg("kdtf"));
            

            py::class_<SAHKDTreeFactory, SimpleKDTreeFactory, std::shared_ptr<SAHKDTreeFactory>>(m, "SAHKDTreeFactory")
                .def(py::init<size_t, double, double, double>(),
                    py::arg("lossNodes") = 21,
                    py::arg("ci") = 1,
                    py::arg("cl") = 1,
                    py::arg("co") = 1);
            

            py::class_<MultiThreadSAHKDTreeFactory, MultiThreadKDTreeFactory, std::shared_ptr<MultiThreadSAHKDTreeFactory>>(m, "MultiThreadSAHKDTreeFactory")
                .def(py::init<std::shared_ptr<SimpleKDTreeFactory>, std::shared_ptr<SimpleKDTreeGeometricStrategy>, size_t, size_t>(),
                    py::arg("kdtf"),
                    py::arg("gs"),
                    py::arg("numJobs") = 2,
                    py::arg("geomJobs") = 2);
            

            py::class_<AxisSAHKDTreeFactory, SAHKDTreeFactory, std::shared_ptr<AxisSAHKDTreeFactory>>(m, "AxisSAHKDTreeFactory")
                .def(py::init<size_t, double, double, double>(),
                    py::arg("lossNodes") = 21,
                    py::arg("ci") = 1,
                    py::arg("cl") = 1,
                    py::arg("co") = 1);


            py::class_<AxisSAHKDTreeGeometricStrategy, SAHKDTreeGeometricStrategy, std::shared_ptr<AxisSAHKDTreeGeometricStrategy>>(m, "AxisSAHKDTreeGeometricStrategy")
                .def(py::init<AxisSAHKDTreeFactory&>(), py::arg("kdtf"));
            
            
            py::class_<FastSAHKDTreeFactory, SAHKDTreeFactory, std::shared_ptr<FastSAHKDTreeFactory>>(m, "FastSAHKDTreeFactory")
                .def(py::init<size_t, double, double, double>(),
                    py::arg("lossNodes") = 32,
                    py::arg("ci") = 1,
                    py::arg("cl") = 1,
                    py::arg("co") = 1);
            

            py::class_<FastSAHKDTreeGeometricStrategy, SAHKDTreeGeometricStrategy, std::shared_ptr<FastSAHKDTreeGeometricStrategy>>(m, "FastSAHKDTreeGeometricStrategy")
                .def(py::init<FastSAHKDTreeFactory&>(),
                    py::arg("kdtf"));


        m.def("calc_time_propagation", &calcTimePropagation, py::arg("timeWave"), py::arg("numBins"), py::arg("scanner"));

        // Add helper methods for XML parsing
        m.def("read_survey_from_xml", &readSurveyFromXml);
        m.def("read_scanner_from_xml", &readScannerFromXml);
        m.def("read_platform_from_xml", &readPlatformFromXml);
        m.def("read_scene_from_xml", &readSceneFromXml);
        m.def("read_scene_part_from_xml", &readScenePartFromXml);
        
        // Add helper function for scene handling
        m.def("finalize_static_scene", &finalizeStaticScene);
        m.def("invalidate_static_scene", &invalidateStaticScene);
        m.def("set_scene_reflectances", &setSceneReflectances);
        m.def("read_obj_scene_part", &readObjScenePart);
        m.def("read_tiff_scene_part", &readTiffScenePart);
        m.def("read_xyz_scene_part", &readXYZScenePart);
        m.def("read_vox_scene_part", &readVoxScenePart);
        m.def("rotate_scene_part", &rotateScenePart);
        m.def("scale_scene_part", &scaleScenePart);
        m.def("translate_scene_part", &translateScenePart);
    }
}