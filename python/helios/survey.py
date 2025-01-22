from datetime import datetime, timezone
import numpy as np
from helios.leg import Leg
from helios.platform import Platform, PlatformSettingsBase
from helios.scanner import Scanner, ScannerSettingsBase
from helios.scene import Scene
from helios.util import get_asset_directories, meas_dtype, traj_dtype
from helios.validation import Validatable, ValidatedCppManagedProperty
from pathlib import Path
from pydantic import validate_call
from typing import Literal, Optional
import tempfile

import os

import _helios


class Survey(Validatable):
    def __init__(
        self,
        scanner: Scanner = None,
        platform: Platform = None,
        scene: Scene = None,
        legs: list[Leg] = [],
        name: str = "",
    ):
        """Construct a survey object programmatically."""
        # Instantiate the C++ object
        self._cpp_object = _helios.Survey()

        # Set the scanner - it is required for constructing a survey
        if scanner is None:
            raise ValueError("A scanner must be provided!")
        self.scanner = scanner

        if platform is None:
            raise ValueError("A platform must be provided")
        self.platform = platform

        if scene is None:
            raise ValueError("A scene must be provided")
        self.scene = scene

        # Set additional "easy" properties
        self.legs = legs
        self.name = name

    legs: list[Leg] = ValidatedCppManagedProperty("legs", Leg, iterable=True)
    name: str = ValidatedCppManagedProperty("name")
    platform: Platform = ValidatedCppManagedProperty("platform", Platform)
    scanner: Scanner = ValidatedCppManagedProperty("scanner", Scanner)
    scene: Scene = ValidatedCppManagedProperty("scene", Scene)

    @validate_call
    def run(
        self,
        output: Optional[Path] = None,
        num_threads: Optional[int] = None,
        format: Literal["laz", "las", "xyz"] = "las",
    ):

        # List of parameters to maybe incorporate in the future
        parallelization_strategy = 1
        warehouse_factor = 4

        if output is None:
            #TODO: Implement approach where we don't need to write to disk
            las_output, zip_output = False, False
            temp_dir_obj = tempfile.TemporaryDirectory()
            
            fms = _helios.FMSFacadeFactory().build_facade(
                    temp_dir_obj.name, 1.0, las_output, False, zip_output, False, self._cpp_object
                )
            
        else:
            # Make the given output path absolute
            output = Path(output).absolute()

            # Determine boolean flags for the output
            las_output, zip_output = {
                "laz": (True, True),
                "las": (True, False),
                "xyz": (False, False),
            }.get(format)
            fms = _helios.FMSFacadeFactory().build_facade(
                str(output), 1.0, las_output, False, zip_output, False, self._cpp_object
            )

        # Determine maximum number of threads to use
        if num_threads is None:
            num_threads = os.cpu_count()

        # Use the current time as GPS time (will be argument later)
        current_time = datetime.now(timezone.utc).isoformat(timespec="seconds")

        # Set up internal data structures for the execution
       
        accuracy = self.scanner._cpp_object.detector.accuracy
        ptpf = _helios.PulseThreadPoolFactory(
            parallelization_strategy, num_threads - 1, accuracy, 32, warehouse_factor
        )
        pulse_thread_pool = ptpf.make_pulse_thread_pool()
        playback = _helios.SurveyPlayback(
            self._cpp_object,
            fms,
            parallelization_strategy,
            pulse_thread_pool,
            32,
            current_time,
            True,
            True,
        )
        playback.callback_frequency = 0

        self.scanner._cpp_object.cycle_measurements_mutex  = None 
        self.scanner._cpp_object.cycle_measurements = []
        self.scanner._cpp_object.cycle_trajectories = []
        self.scanner._cpp_object.all_measurements = []
        self.scanner._cpp_object.all_trajectories = []
        self.scanner._cpp_object.all_output_paths = []

        # Start simulating the survey
        playback.start()

        if output is None:
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
        **parameters,
    ):
        """Add a new leg to the survey.

        It can either be already constructed or it will be constructed
        from the provided settings.
        """

        # We construct a leg if none was provided
        if leg is None:
            leg = Leg(**parameters)

        # By using assignment instead of append,
        # we ensure that the property is validated
        self.legs = self.legs + [leg]

    @classmethod
    def from_xml(cls, survey_file: Path):
        """Construct the survey object from an XML file."""

        _cpp_survey = _helios.read_survey_from_xml(
            survey_file, [str(p) for p in get_asset_directories()], True, True
        )
        return cls._from_cpp_object(_cpp_survey)
