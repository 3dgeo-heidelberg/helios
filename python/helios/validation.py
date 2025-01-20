from pydantic import validate_call, GetCoreSchemaHandler
from pydantic_core import core_schema
from typing import Any, Type


class ValidatedCppManagedProperty:
    """Descriptor that performs pydantic validation and delegates to a C++ property

    An implementation of the descriptor protocol that allows us to perform
    pydantic validation of values for a property but then forward them to a
    C++ object. Used heavily across the Helios++ codebase.
    """

    def __init__(self, name, wraptype=None, iterable=False):
        self.name = name
        assert wraptype is None or issubclass(wraptype, Validatable)
        self.wraptype = wraptype
        self.iterable = iterable

    def _maybe_wrap(self, value):
        if self.wraptype is not None and value is not None:
            return self.wraptype._from_cpp_object(value)
        return value

    def __get__(self, obj, objtype=None):
        val = getattr(obj._cpp_object, self.name)

        if self.iterable:
            return type(val)(self._maybe_wrap(item) for item in val)
        return self._maybe_wrap(val)

    def __set__(self, obj, value):
        @validate_call
        def _validated_setter(value: self._get_annotation(obj)):
            if self.iterable:
                if len(value) > 0:
                    if self.wraptype is not None:
                        assert issubclass(type(value[0]), Validatable)
                        self.wraptype = type(value[0])
                    cpp_value = [
                        (
                            getattr(item, "_cpp_object")
                            if hasattr(item, "_cpp_object")
                            else item
                        )
                        for item in value
                    ]
                else:
                    cpp_value = []
            else:
                if self.wraptype is not None:
                    assert issubclass(type(value), Validatable)
                    self.wraptype = type(value)
                cpp_value = (
                    getattr(value, "_cpp_object")
                    if hasattr(value, "_cpp_object")
                    else value
                )

            setattr(obj._cpp_object, self.name, cpp_value)

        _validated_setter(value)

    def _get_annotation(self, obj):
        for cls in obj.__class__.__mro__:
            if self.name in getattr(cls, "__annotations__", {}):
                return cls.__annotations__[self.name]
        raise AttributeError(
            f"'{obj.__class__.__name__}' object has no annotation for '{self.name}'"
        )


class Validatable:
    """Base class for objects that use ValidatedCppManagedProperty"""

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

    @classmethod
    def _from_cpp_object(cls, cpp_object):
        obj = cls.__new__(cls)
        obj._cpp_object = cpp_object
        return obj

    def __repr__(self):
        return f"<{self.__class__.__name__} at {hex(id(self._cpp_object))}>"

    def clone(self):
        if not hasattr(self._cpp_object, "clone"):
            raise NotImplementedError(
                f"{self.__class__.__name__} does not support cloning, as the C++ object "
                "does not yet implement a clone method. Please open an issue with your use "
                "case."
            )
        return self.__class__._from_cpp_object(self._cpp_object.clone())


class UpdateableMixin:
    """Mixin for objects that can be updated from another object"""

    def update_from_dict(self, kwargs, skip_exceptions=False):
        """Update a validatable object from a dictionary of keyword arguments"""

        keys = list(kwargs.keys())
        for key in keys:
            if key in self.__class__.__dict__ and isinstance(
                self.__class__.__dict__[key], ValidatedCppManagedProperty
            ):
                setattr(self, key, kwargs.pop(key))
            elif not skip_exceptions:
                raise ValueError(f"Invalid key: {key}")

    def update_from_object(self, other, skip_exceptions=False):
        """Update a validatable object from another object"""

        parameters = {}
        for key, value in self.__class__.__dict__.items():
            if isinstance(value, ValidatedCppManagedProperty):
                parameters[key] = getattr(other, key)
        self.update_from_dict(parameters, skip_exceptions=skip_exceptions)
