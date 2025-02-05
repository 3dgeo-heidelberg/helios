from datetime import datetime, timezone
import numpy as np
from helios.leg import Leg
from helios.platform import Platform, PlatformSettings
from helios.scanner import Scanner, ScannerSettings
from helios.scene import Scene
from helios.util import get_asset_directories, meas_dtype, traj_dtype
from helios.validation import ValidatedCppModel, ValidatedCppManagedProperty
from pathlib import Path
from pydantic import validate_call
from typing import Literal, Optional
import tempfile

import os

import _helios


class Survey(ValidatedCppModel, cpp_class=_helios.Survey):
    scanner: Scanner = ValidatedCppManagedProperty(
        "scanner", Scanner, unique_across_instances=True
    )
    platform: Platform = ValidatedCppManagedProperty("platform", Platform)
    scene: Scene = ValidatedCppManagedProperty("scene", Scene)
    legs: list[Leg] = ValidatedCppManagedProperty(
        "legs", Leg, iterable=True, default=[]
    )
    name: str = ValidatedCppManagedProperty("name", default="")

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

        self.scanner._cpp_object.cycle_measurements_mutex = None
        self.scanner._cpp_object.cycle_measurements = np.empty((0,), dtype=meas_dtype)
        self.scanner._cpp_object.cycle_trajectories = np.empty((0,), dtype=traj_dtype)
        self.scanner._cpp_object.all_measurements = np.empty((0,), dtype=meas_dtype)
        self.scanner._cpp_object.all_trajectories = np.empty((0,), dtype=traj_dtype)
        self.scanner._cpp_object.all_output_paths = np.empty((0,))
        # Start simulating the survey
        playback.start()

        if output is None:
            measurements = self.scanner._cpp_object.all_measurements
            trajectories = self.scanner._cpp_object.all_trajectories
            temp_dir_obj.cleanup()
            
            return measurements, trajectories

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
    def from_xml(cls, survey_file: Path):
        """Construct the survey object from an XML file."""

        _cpp_survey = _helios.read_survey_from_xml(
            survey_file, [str(p) for p in get_asset_directories()], True, True
        )
        return cls.__new__(cls, _cpp_object=_cpp_survey)
