from pydantic import validate_call, GetCoreSchemaHandler
from pydantic_core import core_schema
from typing import Any, Type
from typing_extensions import dataclass_transform


class Property:
    """Descriptor that performs pydantic validation and delegates to a C++ property

    An implementation of the descriptor protocol that allows us to perform
    pydantic validation of values for a property but then forward them to a
    C++ object. Used heavily across the Helios++ codebase.
    """

    def __init__(
        self,
        cpp=None,
        wraptype=None,
        iterable=False,
        default=None,
        unique_across_instances=False,
    ):
        if wraptype is not None and not issubclass(wraptype, Model):
            raise TypeError("wraptype must be a subclass of Model")

        if wraptype is not None and cpp is None:
            raise ValueError("Wraptype can only used with C++-managed properties")

        self.cpp = cpp
        self.wraptype = wraptype
        self.iterable = iterable
        self.default = default
        self.unique_across_instances = unique_across_instances

    def _maybe_wrap(self, value):
        if self.wraptype is not None and value is not None:
            return self.wraptype.__new__(self.wraptype, _cpp_object=value)
        return value

    def __get__(self, obj, objtype=None):
        if self.cpp is None:
            val = (
                obj.__class__.__dict__.get("_instance_data", {})
                .get(repr(obj), {})
                .get(repr(self), self.default)
            )
        else:
            val = getattr(obj._cpp_object, self.cpp)

        if self.iterable:
            return type(val)(self._maybe_wrap(item) for item in val)
        return self._maybe_wrap(val)

    def __set__(self, obj, value):
        @validate_call
        def _validated_setter(value: self._get_annotation(obj)):
            def _assign_obj(value):
                if self.cpp is None:
                    return value
                return (
                    getattr(value, "_cpp_object")
                    if hasattr(value, "_cpp_object")
                    else value
                )

            if self.iterable:
                if len(value) > 0:
                    if self.wraptype is not None:
                        assert issubclass(type(value[0]), Model)
                        self.wraptype = type(value[0])
                    value = [_assign_obj(item) for item in value]
                else:
                    value = []
            else:
                if self.wraptype is not None:
                    assert issubclass(type(value), Model)
                    self.wraptype = type(value)
                value = _assign_obj(value)

            if self.unique_across_instances:
                if self.iterable:
                    raise NotImplementedError(
                        "Unique-across-instances is not implemented for iterable properties"
                    )
                if not hasattr(obj.__class__, "_uniqueness"):
                    obj.__class__._uniqueness = {}
                if value in obj.__class__.__dict__["_uniqueness"].values():
                    raise ValueError(
                        f"This value for property {self.cpp} is already in use by a different instance."
                    )
                obj.__class__.__dict__["_uniqueness"][self.cpp] = value

            if self.cpp is None:
                if not hasattr(obj.__class__, "_instance_data"):
                    setattr(obj.__class__, "_instance_data", {})
                obj.__class__._instance_data.setdefault(repr(obj), {})[
                    repr(self)
                ] = value
            else:
                setattr(obj._cpp_object, self.cpp, value)

        _validated_setter(value)

    def _get_annotation(self, obj):
        for cls in obj.__class__.__mro__:
            for name, prop in cls.__dict__.items():
                if prop is self:
                    annotations = getattr(cls, "__annotations__", {})
                    if name in annotations:
                        return annotations[name]
                    else:
                        raise TypeError("Missing type annotation")


@dataclass_transform()
class ValidatedCppModelMetaClass(type):
    def __new__(cls, name, bases, dct, *args, **kwargs):

        cls = super().__new__(cls, name, bases, dct, **kwargs)

        # Retrieve properties while preserving their definition order
        fields = {}
        for base in reversed(cls.mro()[:-1]):  # Traverse from base to derived
            for key, value in base.__dict__.items():
                if isinstance(value, Property) and key not in fields:
                    fields[key] = value

        def __init__(self, *args, **instance_kwargs):
            # Iterate the fields in exactly the given order. When using a different order,
            # we risk that properties are instantiated in an incorrect order.
            for i, (name, field) in enumerate(fields.items()):
                # Check whether we find this among positional arguments
                if i < len(args):
                    setattr(self, name, args[i])
                    continue

                # Check whether we find this among keyword arguments
                if name in instance_kwargs:
                    setattr(self, name, instance_kwargs[name])
                    continue

                # Use the provided default
                if field.default is not None:
                    setattr(self, name, field.default)
                    continue

                # Raise an error if this was required and not we reached this point
                raise ValueError(f"Missing required argument: {name}")

        setattr(cls, "__init__", __init__)

        return cls


class Model(metaclass=ValidatedCppModelMetaClass):
    """Base class for objects that use ValidatedCppManagedProperty"""

    def __init_subclass__(cls, cpp_class=None, **kwargs):
        super().__init_subclass__(**kwargs)
        if cpp_class is not None:
            cls._cpp_class = cpp_class

    def __new__(cls, *args, **kwargs):
        obj = super().__new__(cls)
        if hasattr(cls, "_cpp_class"):
            cpp_object = kwargs.pop("_cpp_object", None)
            if cpp_object is None:
                cpp_object = cls._cpp_class()
            obj._cpp_object = cpp_object
        return obj

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
        if hasattr(self.__class__, "_cpp_class"):
            return f"<{self.__class__.__name__} at {hex(id(self._cpp_object))}>"
        else:
            return f"<{self.__class__.__name__} at {hex(id(self))}>"

    def clone(self):
        if not hasattr(self._cpp_object, "clone"):
            raise NotImplementedError(
                f"{self.__class__.__name__} does not support cloning, as the C++ object "
                "does not yet implement a clone method. Please open an issue with your use "
                "case."
            )
        return self.__class__.__new__(
            self.__class__, _cpp_object=self._cpp_object.clone()
        )


class UpdateableMixin:
    """Mixin for objects that can be updated from another object"""

    def update_from_dict(self, kwargs, skip_exceptions=False):
        """Update a ValidatedCppModel object from a dictionary of keyword arguments"""

        keys = list(kwargs.keys())
        for key in keys:
            if key in self.__class__.__dict__ and isinstance(
                self.__class__.__dict__[key], Property
            ):
                setattr(self, key, kwargs.pop(key))
            elif not skip_exceptions:
                raise ValueError(f"Invalid key: {key}")

    def update_from_object(self, other, skip_exceptions=False):
        """Update a ValidatedCppModel object from another object"""

        parameters = {}
        for key, value in self.__class__.__dict__.items():
            if isinstance(value, Property):
                parameters[key] = getattr(other, key)
        self.update_from_dict(parameters, skip_exceptions=skip_exceptions)
