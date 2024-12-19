from pyhelios.scene import Scene
from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.scanner import Scanner, AbstractDetector
from pyhelios.primitives import Rotation, Measurement, Trajectory
from typing import Optional, List, Callable
import _helios

class SimulationCycleCallback:
    def __init__(self, callback: Optional[Callable] = None):
        # Create an instance of the C++ SimulationCycleCallback and pass the callback object
        self._cpp_object = _helios.SimulationCycleCallback(callback)
        self._callback = callback  # Store the Python callable

    @property
    def callback(self) -> Optional[Callable]:
        return self._callback

    @callback.setter
    def callback(self, value: Optional[Callable]):
        if not callable(value) and value is not None:
            raise ValueError("The callback must be callable.")
        self._callback = value
    

    def __call__(self, measurements: List['Measurement'], trajectories: List['Trajectory'], outpath: str):
        """
        Invokes the callback function with the measurements, trajectories, and outpath.
        """
        if self._callback:
            self._callback(measurements, trajectories, outpath)
        else:
            raise ValueError("No callback function set.")

    def set_callback(self, callback: Callable):
        """
        Set a new callback function for the C++ object.
        """
        self.callback = callback

class Simulation:
    def __init__(self, final_output: Optional[bool] = True, legacy_energy_model: Optional[bool] = False, export_to_file: Optional[bool] = True,
                 num_threads: Optional[int] = 0, num_runs: Optional[int] = 1, callback_frequency: Optional[int] = 0, 
                 simulation_frequency: Optional[SimulationCycleCallback] = None, fixed_gps_time: Optional[str] = "", 
                 las_output: Optional[bool] = False, las10_output: Optional[bool] = False,
                 zip_output: Optional[bool] = False , split_by_channel: Optional[bool] = False, las_scale: Optional[float] = 0.0001, 
                 kdt_factory: Optional[int] = 4,
                 kdt_jobs: Optional[int] = 0, kdt_SAH_loss_nodes: Optional[int] = 32, parallelization_strategy: Optional[int] = 1, chunk_size: Optional[int] = 32,
                 warehouse_factor: Optional[int] = 1) -> None:
        self._cpp_object = _helios.Simulation()


