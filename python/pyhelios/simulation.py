
from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.primitives import Rotation, Measurement, Trajectory
from pyhelios.survey import Survey
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

class PyheliosSimulation(Validatable):
    def __init__(self, final_output: Optional[bool] = True, legacy_energy_model: Optional[bool] = False, export_to_file: Optional[bool] = True,
                 num_threads: Optional[int] = 0, num_runs: Optional[int] = 1, callback_frequency: Optional[int] = 0, 
                 simulation_frequency: Optional[SimulationCycleCallback] = None, fixed_gps_time: Optional[str] = "", 
                 las_output: Optional[bool] = False, las10_output: Optional[bool] = False,
                 zip_output: Optional[bool] = False , split_by_channel: Optional[bool] = False, las_scale: Optional[float] = 0.0001, 
                 kdt_factory_type: Optional[int] = 4,
                 kdt_jobs: Optional[int] = 0, kdt_SAH_loss_nodes: Optional[int] = 32, parallelization_strategy: Optional[int] = 1, chunk_size: Optional[int] = 32,
                 warehouse_factor: Optional[int] = 1, survey: Optional[Survey] = None, survey_path: Optional[str] = "",
                 assets_path: Optional[str] = "", output_path: Optional[str] = ""
                 ) -> None:
        self._cpp_object = _helios.PyheliosSimulation()
        self.survey_path = survey_path
        self.assets_path = assets_path
        self.output_path = output_path
        self.survey = survey
        self.final_output = final_output
        self.legacy_energy_model = legacy_energy_model
        self.export_to_file = export_to_file
        self.num_threads = num_threads
        self.num_runs = num_runs
        self.callback_frequency = callback_frequency
        self.simulation_frequency = simulation_frequency
        self.fixed_gps_time = fixed_gps_time
        self.las_output = las_output

        self.las10_output = las10_output
        self.zip_output = zip_output
        self.split_by_channel = split_by_channel
        self.las_scale = las_scale
        self.kdt_factory_type = kdt_factory_type
        self.kdt_jobs = kdt_jobs
        self.kdt_SAH_loss_nodes = kdt_SAH_loss_nodes
        self.parallelization_strategy = parallelization_strategy
        self.chunk_size = chunk_size
        self.warehouse_factor = warehouse_factor

        
        self.is_started = False
        self.is_paused = False
        self.is_stopped = False


    final_output: Optional[bool] = ValidatedCppManagedProperty("final_output")
    legacy_energy_model: Optional[bool] = ValidatedCppManagedProperty("legacy_energy_model")
    export_to_file: Optional[bool] = ValidatedCppManagedProperty("export_to_file")
    num_threads: Optional[int] = ValidatedCppManagedProperty("num_threads")
    num_runs: Optional[int] = ValidatedCppManagedProperty("num_runs")
    callback_frequency: Optional[int] = ValidatedCppManagedProperty("callback_frequency")
    simulation_frequency: Optional[SimulationCycleCallback] = ValidatedCppManagedProperty("simulation_frequency")
    fixed_gps_time: Optional[str] = ValidatedCppManagedProperty("fixed_gps_time")
    las_output: Optional[bool] = ValidatedCppManagedProperty("las_output")
    las10_output: Optional[bool] = ValidatedCppManagedProperty("las10_output")
    zip_output: Optional[bool] = ValidatedCppManagedProperty("zip_output")
    split_by_channel: Optional[bool] = ValidatedCppManagedProperty("split_by_channel")
    las_scale: Optional[float] = ValidatedCppManagedProperty("las_scale")
    kdt_factory_type: Optional[int] = ValidatedCppManagedProperty("kdt_factory")
    kdt_jobs: Optional[int] = ValidatedCppManagedProperty("kdt_jobs")
    kdt_SAH_loss_nodes: Optional[int] = ValidatedCppManagedProperty("kdt_SAH_loss_nodes")
    parallelization_strategy: Optional[int] = ValidatedCppManagedProperty("parallelization_strategy")
    chunk_size: Optional[int] = ValidatedCppManagedProperty("chunk_size")
    warehouse_factor: Optional[int] = ValidatedCppManagedProperty("warehouse_factor")
    survey: Optional[Survey] = ValidatedCppManagedProperty("survey")



    def start(self):
        if self.is_started:
            raise ValueError("Simulation is already started")
        
        if self.survey is None:
            self.load_survey(self.survey_path)
        self.is_started = True
        
        # Add output here later

        self.build_pulse_thread_pool()

    def pause(self):
        pass

    def stop(self):
        pass

    def resume(self):
        pass

    def join(self):
        pass

    def load_survey(self, survey_path: str):
        self.survey = Survey.from_xml(survey_path)

    def add_rotation(self, rotation: Rotation):
        pass

    def add_scale_filter(self, scale_filter: float):
        pass

    def add_translate_filter(self, translate_filter: float):
        pass

    def build_pulse_thread_pool(self):
        
        #FOR NOW JUST CALL IT FROM THE C++ SIDE
        self._cpp_object.build_pulse_thread_pool()

    


    

