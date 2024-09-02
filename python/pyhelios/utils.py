from pydantic import validate_call, GetCoreSchemaHandler
from pydantic_core import core_schema
from typing import Optional, Type, Any

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
        def _validated_setter(value: obj.__annotations__[self.name]):
            if isinstance(value, list):
                cpp_value = [getattr(item, "_cpp_object") if hasattr(item, "_cpp_object") else item for item in value]
            else:
                cpp_value = getattr(value, "_cpp_object") if hasattr(value, "_cpp_object") else value
            setattr(obj._cpp_object, self.name, cpp_value)
            self._values[obj] = value

        _validated_setter(value)  

