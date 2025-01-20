from helios.validation import *

import copy
import pytest


class MockCppObject:
    someint = 42
    somestr = "Foobar"
    somebool = True
    somevec = [0, 1]


class RelatedCppMockObject:
    other = MockCppObject()


def test_validatable():
    class Obj(Validatable):
        def __init__(self):
            self._cpp_object = MockCppObject()

        someint: int = ValidatedCppManagedProperty("someint")
        somestr: str = ValidatedCppManagedProperty("somestr")
        somebool: bool = ValidatedCppManagedProperty("somebool")

    obj = Obj()

    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj.someint = 43
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.someint = "Foobar"


def test_iterable_property():
    class IterObj(Validatable):
        def __init__(self):
            self._cpp_object = MockCppObject()

        somevec: list[int] = ValidatedCppManagedProperty("somevec", iterable=True)

    obj = IterObj()

    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 0
    assert obj.somevec[1] == 1

    obj.somevec = [1, 2]
    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 1
    assert obj.somevec[1] == 2


def test_wrapping():
    class Obj(Validatable):
        def __init__(self):
            self._cpp_object = MockCppObject()

        someint: int = ValidatedCppManagedProperty("someint")

    class RelatedObj(Validatable):
        def __init__(self):
            self._cpp_object = RelatedCppMockObject()

        other: Obj = ValidatedCppManagedProperty("other", Obj)
        otherlist: list[Obj] = ValidatedCppManagedProperty(
            "otherlist", Obj, iterable=True
        )

    class DerivedObj(Obj):
        pass

    obj = RelatedObj()

    assert isinstance(obj.other, Obj)

    obj.otherlist = [Obj(), Obj()]
    assert isinstance(obj.otherlist[0], Obj)
    assert isinstance(obj.otherlist[1], Obj)

    # Test preservation of types
    obj.other = DerivedObj()
    assert isinstance(obj.other, DerivedObj)

    obj.otherlist = [DerivedObj(), DerivedObj()]
    assert isinstance(obj.otherlist[0], DerivedObj)
    assert isinstance(obj.otherlist[1], DerivedObj)


def test_repr():
    class Obj(Validatable):
        def __init__(self):
            self._cpp_object = MockCppObject()

    x1 = Obj()
    x2 = Obj()

    assert repr(x1) != repr(x2)

    x2._cpp_object = x1._cpp_object
    assert repr(x1) == repr(x2)


def test_cloning_not_implemented():
    class Obj(Validatable):
        def __init__(self):
            self._cpp_object = MockCppObject()

    obj = Obj()
    with pytest.raises(NotImplementedError):
        obj.clone()


def test_cloning():
    class CloneableMockCppObject:
        someint = 42

        def clone(self):
            return copy.deepcopy(self)

    class Obj(Validatable):
        def __init__(self):
            self._cpp_object = CloneableMockCppObject()

        someint: int = ValidatedCppManagedProperty("someint")

    obj = Obj()
    clone = obj.clone()

    assert repr(obj) != repr(clone)
    assert obj.someint == clone.someint


def test_updateable_mixin():
    class Obj(Validatable, UpdateableMixin):
        def __init__(self):
            self._cpp_object = MockCppObject()

        someint: int = ValidatedCppManagedProperty("someint")

    obj = Obj()
    obj.update_from_dict({"someint": 43})
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.update_from_dict({"someint": "Foobar"})

    with pytest.raises(ValueError):
        obj.update_from_dict({"unknown": 42})

    obj.update_from_dict({"unknown": 42}, skip_exceptions=True)
