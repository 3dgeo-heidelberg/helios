import os
import importlib_resources as resources
from pydantic import validate_call, GetCoreSchemaHandler 
from pydantic_core import core_schema
from typing import Optional, Type, Any, Callable
import random
import math
import numpy as np

class Validatable:
    @classmethod
    def __get_pydantic_core_schema__(cls, source: Type[Any], handler: GetCoreSchemaHandler):
        return core_schema.no_info_after_validator_function(cls._validate, core_schema.any_schema())

    @classmethod
    def _validate(cls, value):
        if not isinstance(value, cls):
            raise ValueError(f"Expected {cls.__name__}, got {type(value).__name__}")
        return value
    

class ValidatedCppManagedProperty:
    def __init__(self, name):
        self.name = name
        self._values = {}

    def __get__(self, obj, objtype=None):
        return self._values.get(obj, None)

    def __set__(self, obj, value):
        @validate_call
        def _validated_setter(value: self._get_annotation(obj)):
            if isinstance(value, list):
                cpp_value = [getattr(item, "_cpp_object") if hasattr(item, "_cpp_object") else item for item in value]
            else:
                cpp_value = getattr(value, "_cpp_object") if hasattr(value, "_cpp_object") else value
            setattr(obj._cpp_object, self.name, cpp_value)
            self._values[obj] = value

        _validated_setter(value)  
    def _get_annotation(self, obj):
        for cls in obj.__class__.__mro__:
            if self.name in getattr(cls, '__annotations__', {}):
                return cls.__annotations__[self.name]
        raise AttributeError(f"'{obj.__class__.__name__}' object has no annotation for '{self.name}'")

class AssetManager:
    _instance = None
    _assets = set()  

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AssetManager, cls).__new__(cls)
            cls._initialize_default_assets()
        return cls._instance

    @classmethod
    def _initialize_default_assets(cls):
        # Attempt to initialize default paths and verify they are valid
        current_dir = os.getcwd()
        pyhelios_dir = str(resources.files("pyhelios"))
        pyhelios_data_dir = str(resources.files("pyhelios") / "data")

        cls._assets.add(current_dir)
        if os.path.exists(pyhelios_dir):
            cls._assets.add(pyhelios_dir)
        else:
            print(f"Pyhelios directory does not exist: {pyhelios_dir}")

        if os.path.exists(pyhelios_data_dir):
            cls._assets.add(pyhelios_data_dir)
        else:
            print(f"Pyhelios data directory does not exist: {pyhelios_data_dir}")

    @classmethod
    def add_asset(cls, path: str):
        cls._assets.add(path)

    @classmethod
    def locate_asset(cls, filename: str) -> Optional[str]:
        # Look for the file in all registered assets
        for path in cls._assets:
            potential_path = os.path.join(path, filename)
            if os.path.exists(potential_path):
                return potential_path
        return None

    @classmethod
    def find_file_by_name(cls, filename: str, auto_add: bool = False) -> str:
        if os.path.isabs(filename):
            if os.path.exists(filename):
                if auto_add:
                    cls.add_asset(os.path.dirname(filename))
                return filename
            else:
                raise FileNotFoundError(f"File not found: {filename}")
        
        located_path = cls.locate_asset(filename)
        if located_path:
            return located_path
        
        raise FileNotFoundError(f"File not found in registered assets: {filename} \n existed assets: {cls._assets}")



def calc_propagation_time_legacy(time_wave, num_bins, bin_size, pulse_length, pulse_length_divisor):
    # Prepare variables

    tau = pulse_length / pulse_length_divisor
    peak_value = 0
    peak_index = 0

    # Do forward iterative process to compute nodes and pick max
    for i in range(num_bins):
        t = i * bin_size
        t_tau = t / tau
        pt = (t_tau ** 2) * math.exp(-t_tau)
        time_wave[i] = pt
        
        if pt > peak_value:
            peak_value = pt
            peak_index = i

    return peak_index

class RandomnessGenerator:
    """Randomness generator that mimics the behavior of the C++ version."""

    def __init__(self, mode="AUTO_SEED", seed=None):
        self.mode = mode
        self.urd_gen = None  # The random generator
        self.urd = None      # The uniform distribution

        if self.mode == "AUTO_SEED":
            self.seed = random.SystemRandom().randint(0, 2**32 - 1)  # Auto seed
        elif self.mode == "FIXED_SEED_DOUBLE" or self.mode == "FIXED_SEED_LONG":
            if seed is None:
                raise ValueError("Fixed seed mode requires a seed")
            self.seed = seed
        else:
            raise ValueError(f"Unknown mode: {self.mode}")

        random.seed(self.seed)

    def compute_uniform_real_distribution(self, lower_bound=0.0, upper_bound=1.0):
        """Compute the uniform real distribution with given bounds."""
        # For Python's random, no need to store the distribution explicitly
        self.urd_gen = random.uniform  # Directly use random.uniform

    def uniform_real_distribution_next(self):
        """Return the next value in the uniform distribution."""
        if self.urd_gen is None:
            self.compute_uniform_real_distribution(0.0, 1.0)  
        
        return self.urd_gen(0.0, 1.0) 
    

class PyHeliosException(Exception):
    """PyHelios exception class"""

    def __init__(self, msg):
        """PyHeliosException constructor: Build a new exception.

        Arguments:
            msg -- message for the exception
        """
        super().__init__(msg)


def create_property(property_name, cpp_function_setter, cpp_function_getter=None, index_function=None):
    """
    Factory function to create a property getter and setter for scanning_device and _cpp_object.
    
    Args:
        property_name (str): The name of the property.
        cpp_function_setter (str): The C++ method to call to set the property on _cpp_object.
        cpp_function_getter (str, optional): The C++ method to call to get the property from _cpp_object.
        index_function (function, optional): Function to get the index of the active scanning device (if multi-device).
    
    Returns:
        property: A property object with getter and setter methods.
    """
    
    def getter(self):
        if index_function:
            # If index_function is provided, use it to get the active device's property
            active_device = self.scanning_devices[index_function(self)]
            return getattr(active_device, property_name)
        return getattr(self.scanning_device, property_name)
    
    def setter(self, value):
        if index_function:
            # If index_function is provided, set the value for the active device
            active_device = self.scanning_devices[index_function(self)]
            setattr(active_device, property_name, value)
            if cpp_function_setter:
                cpp_value = value._cpp_object if hasattr(value, '_cpp_object') else value
                getattr(self._cpp_object, cpp_function_setter)(cpp_value, index_function(self))
        else:
            setattr(self.scanning_device, property_name, value)
            if cpp_function_setter:
                cpp_value = value._cpp_object if hasattr(value, '_cpp_object') else value
                getattr(self._cpp_object, cpp_function_setter)(cpp_value, 0)

    return property(getter, setter)
