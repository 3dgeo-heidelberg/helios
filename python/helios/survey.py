from helios.leg import Leg
from helios.platform import Platform, PlatformSettings
from helios.scanner import Scanner, ScannerSettings
from helios.scene import StaticScene
from helios.settings import (
    ExecutionSettings,
    OutputFormat,
    OutputSettings,
    compose_execution_settings,
    compose_output_settings,
)
from helios.util import get_asset_directories, meas_dtype, traj_dtype
from helios.validation import AssetPath, Model, Property, validate_xml_file

from datetime import datetime, timezone
from pathlib import Path
from pydantic import validate_call
from typing import Optional

import numpy as np
import tempfile

import _helios


class Survey(Model, cpp_class=_helios.Survey):
    scanner: Scanner = Property(
        cpp="scanner", wraptype=Scanner, unique_across_instances=True
    )
    platform: Platform = Property(cpp="platform", wraptype=Platform)
    scene: StaticScene = Property(cpp="scene", wraptype=StaticScene)
    legs: list[Leg] = Property(cpp="legs", wraptype=Leg, iterable=True, default=[])
    name: str = Property(cpp="name", default="")

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

        # Ensure that the scene has been finalized
        self.scene.finalize()

        if output is None:
            # TODO: Implement approach where we don't need to write to disk
            las_output, zip_output = False, False
            temp_dir_obj = tempfile.TemporaryDirectory()

            fms = _helios.FMSFacadeFactory().build_facade(
                temp_dir_obj.name,
                1.0,
                las_output,
                False,
                zip_output,
                False,
                self._cpp_object,
            )

        else:
            # Make the given output path absolute
            output = Path(output_settings.output_dir).absolute()

            # Determine boolean flags for the output
            las_output, zip_output = {
                "laz": (True, True),
                "las": (True, False),
                "xyz": (False, False),
            }.get(output_settings.format)
            fms = _helios.FMSFacadeFactory().build_facade(
                str(output), 1.0, las_output, False, zip_output, False, self._cpp_object
            )

        # Use the current time as GPS time (will be argument later)
        current_time = datetime.now(timezone.utc).isoformat(timespec="seconds")

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
            fms,
            execution_settings.parallelization,
            pulse_thread_pool,
            execution_settings.chunk_size,
            current_time,
            True,
            True,
        )
        playback.callback_frequency = 0

        self.scanner._cpp_object.cycle_measurements_mutex = None
        self.scanner._cpp_object.cycle_measurements = []
        self.scanner._cpp_object.cycle_trajectories = []
        self.scanner._cpp_object.all_measurements = []
        self.scanner._cpp_object.all_trajectories = []
        self.scanner._cpp_object.all_output_paths = []

        # Start simulating the survey
        playback.start()

        if output_settings.format == OutputFormat.NPY:
            measurements = self.scanner._cpp_object.all_measurements
            num_measurements = len(measurements)

            data_mes = np.empty(num_measurements, dtype=meas_dtype)

            for i, measurement in enumerate(measurements):
                data_mes[i] = (
                    measurement.dev_id,
                    measurement.dev_idx,
                    measurement.hit_object_id,
                    tuple(measurement.position),
                    tuple(measurement.beam_direction),
                    tuple(measurement.beam_origin),
                    measurement.distance,
                    measurement.intensity,
                    measurement.echo_width,
                    measurement.return_number,
                    measurement.pulse_return_number,
                    measurement.fullwave_index,
                    measurement.classification,
                    measurement.gps_time,
                )

            trajectories = self.scanner._cpp_object.all_trajectories
            num_trajectories = len(trajectories)

            data_traj = np.empty(num_trajectories, dtype=traj_dtype)

            for i, trajectory in enumerate(trajectories):
                data_traj[i] = (
                    trajectory.gps_time,
                    tuple(trajectory.position),
                    trajectory.roll,
                    trajectory.pitch,
                    trajectory.yaw,
                )

            temp_dir_obj.cleanup()

            return data_mes, data_traj

        # Return path to the created output directory
        return Path(playback.fms.write.get_measurement_writer_output_path()).parent

    def add_leg(
        self,
        leg: Leg = None,
        platform_settings: Optional[PlatformSettings] = None,
        scanner_settings: Optional[ScannerSettings] = None,
        **parameters,
    ):
        """Add a new leg to the survey.

        It can either be already constructed or it will be constructed
        from the provided settings.
        """

        # We construct a leg if none was provided
        if leg is None:
            leg = Leg()

        # Set the parameters given as scanner + platform settings
        if platform_settings is not None:
            leg.platform_settings.update_from_object(platform_settings)
        if scanner_settings is not None:
            leg.scanner_settings.update_from_object(scanner_settings)

        # Update with the rest of the given parameters
        leg.platform_settings.update_from_dict(parameters, skip_exceptions=True)
        leg.scanner_settings.update_from_dict(parameters, skip_exceptions=True)

        # If there are parameters left, we raise an error
        if parameters:
            raise ValueError(f"Unknown parameters: {', '.join(parameters)}")

        # By using assignment instead of append,
        # we ensure that the property is validated
        self.legs = self.legs + [leg]

    @classmethod
    @validate_call
    def from_xml(cls, survey_file: AssetPath):
        """Construct the survey object from an XML file."""

        # Validate the XML
        validate_xml_file(survey_file, "xsd/survey.xsd")

        _cpp_survey = _helios.read_survey_from_xml(
            str(survey_file), [str(p) for p in get_asset_directories()], True, True
        )
        return cls.__new__(cls, _cpp_object=_cpp_survey)
