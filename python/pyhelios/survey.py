from pyhelios.scene import Scene
from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.scanner import Scanner, AbstractDetector
from pyhelios.primitives import Rotation
from typing import Optional, List
import _helios


class Survey(Validatable):
    def __init__(self, name: Optional[str] = "", num_runs: Optional[int] = -1, 
                 sim_speed_factor: Optional[float] = 1., scanner: Optional[Scanner] = None) -> None:
        
        self._cpp_object = _helios.Survey()
        self.name = name
        self.num_runs = num_runs
        self.sim_speed_factor = sim_speed_factor
        self.scanner = scanner

    name: Optional[str] = ValidatedCppManagedProperty("name")
    num_runs: Optional[int] = ValidatedCppManagedProperty("num_runs")
    sim_speed_factor: Optional[float] = ValidatedCppManagedProperty("sim_speed_factor")
    scanner: Optional[Scanner] = ValidatedCppManagedProperty("scanner")

    