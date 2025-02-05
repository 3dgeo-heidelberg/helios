from helios.validation import *

import copy
import pytest


class MockCppObject:
    someint = 41
    somestr = "Foobar2"
    somebool = False
    somevec = [0, 1]


class DerivedMockCppObject(MockCppObject):
    derived = 42


class RelatedCppMockObject:
    other = MockCppObject()


def test_validated_cpp_model():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = Property("someint", default=42)
        somestr: str = Property("somestr", default="Foobar")
        somebool: bool = Property("somebool", default=True)

    obj = Obj()

    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj.someint = 43
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.someint = "Foobar"


def test_instantiation():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = Property("someint")
        somestr: str = Property("somestr")
        somebool: bool = Property("somebool", default=True)

    obj = Obj(42, "Foobar")
    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj = Obj(somestr="Foobar", someint=42)
    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj = Obj(42, somebool=False, somestr="Foobar")
    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == False

    with pytest.raises(ValueError):
        obj = Obj(42)


class test_derived():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = Property("someint", default=42)

    class DerivedObj(Obj, cpp_class=DerivedMockCppObject):
        derived: int = Property("derived", default=42)

    obj = DerivedObj()
    assert isinstance(obj._cpp_object, DerivedMockCppObject)
    assert obj.someint == 42
    assert obj.derived == 42


def test_iterable_property():
    class IterObj(Model, cpp_class=MockCppObject):
        somevec: list[int] = Property(
            "somevec", iterable=True, default=[0, 1]
        )

    obj = IterObj()

    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 0
    assert obj.somevec[1] == 1

    obj.somevec = [1, 2]
    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 1
    assert obj.somevec[1] == 2


def test_wrapping():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = Property("someint", default=0)

    class RelatedObj(Model, cpp_class=RelatedCppMockObject):
        other: Obj = Property("other", Obj, default=Obj())
        otherlist: list[Obj] = Property(
            "otherlist", Obj, iterable=True, default=[]
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


def test_unique_across_instances():
    class Obj(Model, cpp_class=MockCppObject):
        pass

    class RelatedObj(Model, cpp_class=RelatedCppMockObject):
        other: Obj = Property(
            "other", Obj, unique_across_instances=True
        )

    obj = Obj()
    related1 = RelatedObj(obj)

    with pytest.raises(ValueError):
        related2 = RelatedObj(obj)


def test_repr():
    class Obj(Model, cpp_class=MockCppObject):
        pass

    x1 = Obj()
    x2 = Obj()

    assert repr(x1) != repr(x2)

    x2._cpp_object = x1._cpp_object
    assert repr(x1) == repr(x2)


def test_cloning_not_implemented():
    class Obj(Model, cpp_class=MockCppObject):
        pass

    obj = Obj()
    with pytest.raises(NotImplementedError):
        obj.clone()


def test_cloning():
    class CloneableMockCppObject:
        someint = 42

        def clone(self):
            return copy.deepcopy(self)

    class Obj(Model, cpp_class=CloneableMockCppObject):
        someint: int = Property("someint", default=42)

    obj = Obj()
    clone = obj.clone()

    assert repr(obj) != repr(clone)
    assert obj.someint == clone.someint


def test_updateable_mixin():
    class Obj(Model, UpdateableMixin, cpp_class=MockCppObject):
        someint: int = Property("someint", default=42)

    obj = Obj()
    obj.update_from_dict({"someint": 43})
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.update_from_dict({"someint": "Foobar"})

    with pytest.raises(ValueError):
        obj.update_from_dict({"unknown": 42})

    obj.update_from_dict({"unknown": 42}, skip_exceptions=True)
