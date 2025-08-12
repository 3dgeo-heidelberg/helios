from helios.leg import Leg
from helios.platforms import (
    Platform,
    PlatformSettings,
    traj_csv_dtype,
    TrajectorySettings,
)
from helios.scanner import Scanner, ScannerSettings
from helios.scene import StaticScene
from helios.settings import (
    ExecutionSettings,
    FullWaveformSettings,
    OutputFormat,
    OutputSettings,
    compose_execution_settings,
    compose_output_settings,
    apply_log_writing,
)
from helios.utils import (
    get_asset_directories,
    meas_dtype,
    traj_dtype,
    apply_scene_shift,
    is_xml_loaded,
    is_binary_loaded,
)
from helios.validation import AssetPath, Model, validate_xml_file

from datetime import datetime, timezone
from numpydantic import NDArray
from pathlib import Path
from pydantic import Field, validate_call
from typing import Annotated, Optional, Tuple

import numpy as np
import tempfile
import laspy

import _helios


class Survey(Model, cpp_class=_helios.Survey):
    scanner: Scanner
    platform: Platform
    scene: StaticScene
    legs: Tuple[Leg, ...] = ()
    name: str = ""
    gps_time: datetime = datetime.now(timezone.utc)
    full_waveform_settings: FullWaveformSettings = FullWaveformSettings()
    trajectory: Optional[NDArray] = None

    @validate_call
    def run(
        self,
        execution_settings: Optional[ExecutionSettings] = None,
        output_settings: Optional[OutputSettings] = None,
        **parameters,
    ):
        # TODO: Options that need to be incorporated:
        # * Logging options from execution_settings
        # Update the settings to use
        execution_settings = compose_execution_settings(execution_settings, parameters)
        output_settings = compose_output_settings(output_settings, parameters)

        # Update logs settings
        apply_log_writing(execution_settings)
        execution_settings.verbosity.apply()

        # Throw if there are still unknown parameters left
        if parameters:
            raise ValueError(f"Unknown parameters: {', '.join(parameters)}")

        # Ensure that the scene has been finalized
        if not (
            is_xml_loaded(self)
            or is_binary_loaded(self.scene)
            or is_xml_loaded(self.scene)
        ):
            self.scene._finalize(execution_settings)

        self.scene._set_reflectances(self.scanner._cpp_object.wavelength)

        # Apply shift once and only if the survey is not loaded from XML
        if not is_xml_loaded(self):
            apply_scene_shift(self, execution_settings)

        # Set the fullwave form settings on the scanner
        self.scanner._cpp_object.apply_settings_FWF(
            self.full_waveform_settings._to_cpp()
        )

        # we need to add serial IDs to the legs for proper process of writing into the file
        for i, leg in enumerate(self.legs):
            leg._cpp_object.serial_id = i

        if output_settings.format in (OutputFormat.NPY, OutputFormat.LASPY):
            las_output, zip_output, export_to_file = False, False, False
            fms = None

        else:
            # Make the given output path absolute
            output = Path(output_settings.output_dir).absolute()
            export_to_file = True
            # Determine boolean flags for the output
            las_output, zip_output = {
                "laz": (True, True),
                "las": (True, False),
                "xyz": (False, False),
            }.get(output_settings.format)

            self.scanner._cpp_object.write_waveform = output_settings.write_waveform
            self.scanner._cpp_object.write_pulse = output_settings.write_pulse

            fms = _helios.FMSFacadeFactory().build_facade(
                str(output),
                output_settings.las_scale,
                las_output,
                False,
                zip_output,
                output_settings.split_by_channel,
                self._cpp_object,
            )

        # Set up internal data structures for the execution

        accuracy = self.scanner._cpp_object.detector.accuracy
        ptpf = _helios.PulseThreadPoolFactory(
            execution_settings.parallelization,
            execution_settings.num_threads - 1,
            accuracy,
            execution_settings.chunk_size,
            execution_settings.warehouse_factor,
        )
        pulse_thread_pool = ptpf.make_pulse_thread_pool()
        playback = _helios.SurveyPlayback(
            self._cpp_object,
            execution_settings.parallelization,
            pulse_thread_pool,
            execution_settings.chunk_size,
            str(self.gps_time.timestamp()),
            True,
            export_to_file,
            execution_settings.discard_shutdown,
            fms,
        )
        playback.callback_frequency = 0

        self.scanner._cpp_object.cycle_measurements = np.empty((0,), dtype=meas_dtype)
        self.scanner._cpp_object.cycle_trajectories = np.empty((0,), dtype=traj_dtype)
        self.scanner._cpp_object.cycle_measurements_mutex = None
        self.scanner._cpp_object.all_measurements = np.empty((0,), dtype=meas_dtype)
        self.scanner._cpp_object.all_trajectories = np.empty((0,), dtype=traj_dtype)
        self.scanner._cpp_object.all_output_paths = np.empty((0,))
        self.scanner._cpp_object.all_measurements_mutex = None
        # Start simulating the survey
        playback.start()

        if output_settings.format in (OutputFormat.NPY, OutputFormat.LASPY):
            # TODO: Handle situation when measurements or trajectories are empty, since they turned out to be not necessarily required
            measurements = self.scanner._cpp_object.all_measurements

            trajectories = self.scanner._cpp_object.all_trajectories
            if output_settings.format == OutputFormat.NPY:
                return measurements, trajectories

            if output_settings.format == OutputFormat.LASPY:
                header = laspy.LasHeader(version="1.4", point_format=6)
                header.add_extra_dims(
                    [
                        laspy.ExtraBytesParams("echo_width", "f8"),
                        laspy.ExtraBytesParams("fullwaveIndex", "i4"),
                        # laspy.ExtraBytesParams("hitObjectId", "U50"),
                    ]
                )
                header.generating_software = "HELIOS++"

                las = laspy.LasData(header)
                las.synthetic = np.ones_like(las.synthetic)

                las.x = measurements["position"][:, 0]
                las.y = measurements["position"][:, 1]
                las.z = measurements["position"][:, 2]
                las.intensity = measurements["intensity"]
                las.return_number = measurements["return_number"]
                las.number_of_returns = measurements["number_of_returns"]
                las.gps_time = measurements["gps_time"]
                las.classification = measurements["classification"]
                las.echo_width = measurements["echo_width"]
                las.fullwaveIndex = measurements["fullwave_index"]
                # las.hitObjectId = data_mes["hit_object_id"]

                return las, trajectories

        # Return path to the created output directory
        return Path(playback.fms.write.get_measurement_writer_output_path()).parent

    def add_leg(
        self,
        leg: Optional[Leg] = None,
        platform_settings: Optional[PlatformSettings] = None,
        scanner_settings: Optional[ScannerSettings] = None,
        trajectory_settings: Optional[TrajectorySettings] = None,
        **parameters,
    ):
        """Add a new leg to the survey.

        It can either be already constructed or it will be constructed
        from the provided settings.
        """

        copy_platform_settings = (
            platform_settings.__class__()
            if platform_settings is not None
            else PlatformSettings()
        )
        copy_scanner_settings = (
            scanner_settings.__class__()
            if scanner_settings is not None
            else ScannerSettings()
        )
        # Set the parameters given as scanner + platform settings
        if platform_settings is not None:
            copy_platform_settings.update_from_object(platform_settings)
        if scanner_settings is not None:
            copy_scanner_settings.update_from_object(scanner_settings)

        if trajectory_settings is not None:
            copy_trajectory_settings = TrajectorySettings()
            copy_trajectory_settings.update_from_object(trajectory_settings)
        else:
            copy_trajectory_settings = None

        # We construct a leg if none was provided
        if leg is None:
            leg = Leg(
                platform_settings=copy_platform_settings,
                scanner_settings=copy_scanner_settings,
                trajectory_settings=copy_trajectory_settings,
            )
        else:
            if platform_settings is not None:
                leg.platform_settings.update_from_object(copy_platform_settings)
            if scanner_settings is not None:
                leg.scanner_settings.update_from_object(copy_scanner_settings)
            if trajectory_settings is not None:
                leg.trajectory_settings.update_from_object(copy_trajectory_settings)
        # Update with the rest of the given parameters
        leg.platform_settings.update_from_dict(parameters, skip_exceptions=True)
        leg.scanner_settings.update_from_dict(parameters, skip_exceptions=True)
        # If there are parameters left, we raise an error
        if parameters:
            raise ValueError(f"Unknown parameters: {', '.join(parameters)}")

        # By using assignment instead of append,
        # we ensure that the property is validated
        self.legs = self.legs + (leg,)

    @classmethod
    @validate_call
    def from_xml(
        cls,
        survey_file: AssetPath,
        load_scene_not_from_binary: bool = True,
        write_scene_to_binary: bool = False,
    ):
        """Construct the survey object from an XML file."""

        # Validate the XML
        validate_xml_file(survey_file, "xsd/survey.xsd")

        _cpp_survey = _helios.read_survey_from_xml(
            str(survey_file),
            [str(p) for p in get_asset_directories()],
            True,
            load_scene_not_from_binary,
            write_scene_to_binary,
        )

        survey = cls._from_cpp(_cpp_survey)
        survey._is_loaded_from_xml = True
        survey.scene._is_loaded_from_xml = True
        if not load_scene_not_from_binary:
            survey.scene._is_loaded_from_binary = True
        
        return survey

    def _pre_set(self, field, value):
        if field == "scanner":
            self._enforce_uniqueness_across_instances(field, value)
