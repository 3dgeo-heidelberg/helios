from helios.utils import find_file, find_files, is_real_iterable

from collections.abc import Iterable
from pathlib import Path
from pint import UnitRegistry
from pydantic import validate_call, GetCoreSchemaHandler
from pydantic.functional_validators import AfterValidator, BeforeValidator
from pydantic_core import core_schema
from typing import Any, Optional, Type, Union, get_origin, get_args
from typing_extensions import Annotated, dataclass_transform

import annotated_types
import inspect
import multiprocessing
import os
import xmlschema

from copy import deepcopy

# The global registry of physical units used in Helios++. Currently there
# are no custom units, so we go with Pint's default registry.
units = UnitRegistry()


def _unit_validator(default_unit):
    """A validator function that converts to a unit if necessary"""

    def _validator(v):
        quantity = units.Quantity(v)
        if quantity.unitless:
            return v
        return quantity.to(default_unit).magnitude

    return BeforeValidator(_validator)


Angle = Annotated[float, _unit_validator(units.rad)]
AngleVelocity = Annotated[float, _unit_validator(units.rad / units.s)]
Frequency = Annotated[int, _unit_validator(units.Hz), annotated_types.Ge(0)]
Length = Annotated[float, _unit_validator(units.m), annotated_types.Ge(0)]
TimeInterval = Annotated[float, _unit_validator(units.s), annotated_types.Ge(0)]


def validate_xml_file(file_path: Path, schema_path: Path):
    """Validate an XML file against an XML schema"""

    # Resolve file paths
    file_path = find_file(file_path)
    schema_path = find_file(schema_path)

    # Validate the XML file against the schema
    schema = xmlschema.XMLSchema(str(schema_path))
    schema.validate(str(file_path))


def _validate_thread_count(count: Union[int, None]) -> int:
    physical = multiprocessing.cpu_count()
    if count is None:
        return physical

    if count < 1:
        raise ValueError(
            "Thread count must be greater than 0, use None for automatic detection"
        )
    if count > physical:
        raise ValueError(
            f"Thread count must be less than or equal to the available number of cores ({physical})"
        )

    return count


def _create_directory(directory: Path):
    os.makedirs(directory, exist_ok=True)
    return directory


# Some type annotations for convenience
AssetPath = Annotated[Path, AfterValidator(find_file)]
MultiAssetPath = Annotated[list[Path], BeforeValidator(find_files)]
ThreadCount = Annotated[Optional[int], AfterValidator(_validate_thread_count)]
CreatedDirectory = Annotated[Path, AfterValidator(_create_directory)]


def _is_iterable_annotation(a):
    """Determine whether a type annotation describes an iterable."""

    origin = get_origin(a)
    if origin is None:
        return False
    if not isinstance(origin, type):
        return False
    return issubclass(origin, Iterable)


def _is_iterable_of_model_annotation(a):
    """Determine whether this type annotation describes an iterable of Models"""

    if not _is_iterable_annotation(a):
        return False

    args = get_args(a)
    return issubclass(args[0], Model)


def _is_optional(t: Type) -> bool:
    origin = get_origin(t)
    return origin is Union and type(None) in get_args(t)


def _inner_optional_type(t: Type) -> Type:
    return next(arg for arg in get_args(t) if arg is not type(None))


def get_all_annotations(cls):
    """Collect annotations from cls and all its bases (excluding `object`)."""
    anns: dict[str, type] = {}
    for base in reversed(cls.__mro__):
        if base is object:
            continue
        anns.update(inspect.get_annotations(base))
    return anns


def get_all_defaults(cls, dct):
    """Collect default values for fields from cls and all its bases (excluding `object`)."""
    defaults = {}
    for base in reversed(cls.mro()[:-1]):
        if base is object:
            continue
        base_defaults = getattr(base, "_defaults", None)
        if isinstance(base_defaults, dict):
            defaults.update(base_defaults)

    for field in inspect.get_annotations(cls).keys():
        if field in dct:
            val = dct[field]
            if not isinstance(val, property):
                defaults[field] = val

    return defaults


@dataclass_transform()
class ValidatedModelMetaClass(type):
    def __new__(cls, name, bases, dct, **kwargs):
        cls = super().__new__(cls, name, bases, dct, **kwargs)

        annotations = get_all_annotations(cls)
        # Determine fields that should be properties
        defaults = get_all_defaults(cls, dct)
        setattr(cls, "_defaults", defaults)

        # Creation of properties need to be wrapped in a function to accommodate
        # a semantic weirdness of Python: If you define a function inside a loop,
        # the function will capture the loop variable, not the value of the loop
        # variable at the time of the function definition. Wrapping the logic in
        # a function will create a new scope for the loop variable, which will
        # then be captured correctly by the function.
        def _make_property(f, a):
            def _getter(self):
                if _is_optional(a) and hasattr(self, f"_{f}"):
                    return getattr(self, f"_{f}")

                # If the property is backed by a C++ object, we first check for
                # potential updates for the Python object
                if hasattr(self, "_cpp_object") and hasattr(self._cpp_object, f):
                    value = getattr(self._cpp_object, f)

                    if _is_optional(a):
                        T = _inner_optional_type(a)
                        if value is None:
                            wrapped = None
                        elif hasattr(T, "_from_cpp"):
                            wrapped = T._from_cpp(value)
                        else:
                            wrapped = value
                        setattr(self, f"_{f}", wrapped)
                        return wrapped

                    # Determine whether this is of type Iterable[Model]
                    if _is_iterable_of_model_annotation(a):

                        def _check_if_rebuild():
                            if len(value) != len(getattr(self, f"_{f}")):
                                return True
                            for i, val in enumerate(value):
                                if val is not getattr(self, f"_{f}")[i]._cpp_object:
                                    return True
                            return False

                        if _check_if_rebuild():
                            setattr(
                                self,
                                f"_{f}",
                                [get_args(a)[0]._from_cpp(val) for val in value],
                            )
                    else:
                        if hasattr(a, "_cpp_class"):
                            getattr(self, f"_{f}")._cpp_object = value
                            return getattr(self, f"_{f}")
                        return value

                # Otherwise, we take the property from a variable with a prefixed underscore
                return getattr(self, f"_{f}")

            @validate_call
            def _setter(self, value: a):
                # If a pre_set hook was provided, we execute it
                self._pre_set(f, value)

                # We always store the object in a variable with a prefixed underscore
                setattr(self, f"_{f}", value)

                # If this property is backed by a C++ object, we additionally set it on C++
                if hasattr(self, "_cpp_object") and hasattr(self._cpp_object, f):
                    if (
                        is_real_iterable(value)
                        and len(value) > 0
                        and hasattr(value[0], "_cpp_object")
                    ):
                        value = [v._cpp_object for v in value]
                    if hasattr(value, "_cpp_object"):
                        value = value._cpp_object
                    setattr(self._cpp_object, f, value)

                # If a post_set hook was provided, we execute it
                self._post_set(f)

            # Register the getter and setter on the property
            return property().getter(_getter).setter(_setter)

        # Make fields properties
        for field, annot in annotations.items():
            setattr(cls, field, _make_property(field, annot))

        # Create an __init__ for the class
        def __init__(self, *args, **instance_kwargs):

            self._during_init = True

            # Iterate the fields in exactly the given order. When using a different order,
            # we risk that properties are instantiated in an incorrect order.
            for i, field in enumerate(annotations):
                # Check whether we find this among positional arguments
                if i < len(args):
                    setattr(self, field, args[i])
                    continue

                # Check whether we find this among keyword arguments
                if field in instance_kwargs:
                    setattr(self, field, instance_kwargs[field])
                    continue

                if field in cls._defaults:
                    default_value = cls._defaults[field]
                    if default_value is None and _is_optional(annotations[field]):
                        setattr(self, field, None)
                        continue

                    # Make a deepcopy if it's a known mutable type
                    if isinstance(default_value, (list, dict, set)) or hasattr(
                        default_value, "__deepcopy__"
                    ):
                        default_value = deepcopy(default_value)

                    setattr(self, field, default_value)
                    continue

                if _is_optional(annotations[field]):
                    if hasattr(_inner_optional_type(annotations[field]), "_cpp_class"):
                        setattr(self, field, None)
                        continue

                # Raise an error if this was required and not we reached this point
                raise ValueError(f"Missing required argument: {field}")

            self._during_init = False
            instance_kwargs.pop("_cpp_object", None)
            invalid_fields = set(instance_kwargs) - set(annotations)
            if invalid_fields:
                raise ValueError(f"Invalid fields passed: {', '.join(invalid_fields)}")

        setattr(cls, "__init__", __init__)

        return cls


class Model(metaclass=ValidatedModelMetaClass):
    """Base class for validated objects in Helios++."""

    def __init_subclass__(cls, cpp_class=None, **kwargs):
        super().__init_subclass__(**kwargs)
        if not hasattr(cls, "_cpp_class"):
            cls._cpp_class = None
        if cpp_class is not None:
            cls._cpp_class = cpp_class

    def __new__(cls, *args, **kwargs):
        obj = super().__new__(cls)
        if cls._cpp_class is not None:
            cpp_object = kwargs.pop("_cpp_object", None)
            if cpp_object is None:
                cpp_object = cls._cpp_class()
            obj._cpp_object = cpp_object
        return obj

    def _post_set(self, field):
        """Hook that is called after a property is set"""
        pass

    def _pre_set(self, field, value):
        """Hook that is called before a property is set"""
        pass

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source: Type[Any], handler: GetCoreSchemaHandler
    ):
        return core_schema.no_info_after_validator_function(
            cls._validate, core_schema.any_schema()
        )

    @classmethod
    def _validate(cls, value):
        if not isinstance(value, cls):
            raise ValueError(f"Expected {cls.__name__}, got {type(value).__name__}")
        return value

    def __repr__(self):
        if hasattr(self, "_cpp_object"):
            return f"<{self.__class__.__name__} at {hex(id(self._cpp_object))}>"
        else:
            return f"<{self.__class__.__name__} at {hex(id(self))}>"

    @classmethod
    def _from_cpp(cls, value):
        params = {}
        for field, annot in get_all_annotations(cls).items():
            if hasattr(value, field):
                cpp_value = getattr(value, field)
                if _is_optional(annot):
                    T = _inner_optional_type(annot)
                    params[field] = (
                        None if cpp_value is None else T._from_cpp(cpp_value)
                    )
                    continue

                if not _is_iterable_annotation(annot):
                    if hasattr(annot, "_from_cpp"):
                        cpp_value = annot._from_cpp(cpp_value)
                else:
                    args = get_args(annot)
                    if hasattr(args[0], "_from_cpp"):
                        cpp_value = [args[0]._from_cpp(v) for v in cpp_value]
                params[field] = cpp_value

        return cls(_cpp_object=value, **params)

    def _enforce_uniqueness_across_instances(self, field, value):
        """Enforce uniqueness of a field across all instances of a class"""

        # Ensure that the uniqueness dictionary is present
        if not hasattr(self.__class__, f"_uniqueness_{field}"):
            setattr(self.__class__, f"_uniqueness_{field}", {})

        # Extract the information mapping values to instances
        info = getattr(self.__class__, f"_uniqueness_{field}")

        # If the value already exists on another instance, raise an error
        def _check_if_exists(val):
            if val in info.keys() and not info[val] is self:
                raise ValueError(f"Value {val} is already used by another instance")

        # For iterables, we need to check every single value
        if is_real_iterable(value):
            for val in value:
                _check_if_exists(val)
                info[val] = self
        else:
            _check_if_exists(value)
            info[value] = self

    def clone(self):
        """Create a deep copy of the object"""

        cpp_object = None

        # Check whether the underlying C++ class supports cloning
        if self.__class__._cpp_class is not None:
            if not hasattr(self._cpp_object, "clone"):
                raise ValueError("Underlying C++ class does not support cloning.")

            # Clone the underlying C++ object
            cpp_object = self._cpp_object.clone()

        properties = {
            f: getattr(self, f) for f in get_all_annotations(self.__class__).keys()
        }

        return self.__class__(_cpp_object=cpp_object, **properties)


class UpdateableMixin:
    """Mixin for objects that can be updated from another object"""

    def update_from_dict(self, kwargs, skip_exceptions=False):
        """Update a ValidatedCppModel object from a dictionary of keyword arguments"""

        keys = list(kwargs.keys())

        anns = get_all_annotations(self.__class__)
        for key in keys:
            cls_attr = self.__class__.__dict__.get(key, None)
            if key in anns:
                setattr(self, key, kwargs.pop(key))
            elif isinstance(cls_attr, property) and cls_attr.fset is not None:
                setattr(self, key, kwargs.pop(key))
            elif not skip_exceptions:
                raise ValueError(f"Invalid key: {key}")

    def update_from_object(self, other, skip_exceptions=False):
        """Update a ValidatedCppModel object from another object"""

        if not isinstance(self, Model):
            raise ValueError("update_from_object can only be called on Model objects")

        parameters = {}
        for field in get_all_annotations(self.__class__).keys():
            parameters[field] = getattr(other, field)
        self.update_from_dict(parameters, skip_exceptions=skip_exceptions)
