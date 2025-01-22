from helios.validation import *

import copy
import pytest


class MockCppObject:
    someint = 41
    somestr = "Foobar2"
    somebool = False
    somevec = [0, 1]


class RelatedCppMockObject:
    other = MockCppObject()


def test_validated_cpp_model():
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        someint: int = ValidatedCppManagedProperty("someint", default=42)
        somestr: str = ValidatedCppManagedProperty("somestr", default="Foobar")
        somebool: bool = ValidatedCppManagedProperty("somebool", default=True)

    obj = Obj()

    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj.someint = 43
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.someint = "Foobar"


def test_instantiation():
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        someint: int = ValidatedCppManagedProperty("someint")
        somestr: str = ValidatedCppManagedProperty("somestr")
        somebool: bool = ValidatedCppManagedProperty("somebool", default=True)

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


def test_iterable_property():
    class IterObj(ValidatedCppModel, cpp_class=MockCppObject):
        somevec: list[int] = ValidatedCppManagedProperty(
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
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        someint: int = ValidatedCppManagedProperty("someint", default=0)

    class RelatedObj(ValidatedCppModel, cpp_class=RelatedCppMockObject):
        other: Obj = ValidatedCppManagedProperty("other", Obj, default=Obj())
        otherlist: list[Obj] = ValidatedCppManagedProperty(
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
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        pass

    class RelatedObj(ValidatedCppModel, cpp_class=RelatedCppMockObject):
        other: Obj = ValidatedCppManagedProperty(
            "other", Obj, unique_across_instances=True
        )

    obj = Obj()
    related1 = RelatedObj(obj)

    with pytest.raises(ValueError):
        related2 = RelatedObj(obj)


def test_repr():
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        pass

    x1 = Obj()
    x2 = Obj()

    assert repr(x1) != repr(x2)

    x2._cpp_object = x1._cpp_object
    assert repr(x1) == repr(x2)


def test_cloning_not_implemented():
    class Obj(ValidatedCppModel, cpp_class=MockCppObject):
        pass

    obj = Obj()
    with pytest.raises(NotImplementedError):
        obj.clone()


def test_cloning():
    class CloneableMockCppObject:
        someint = 42

        def clone(self):
            return copy.deepcopy(self)

    class Obj(ValidatedCppModel, cpp_class=CloneableMockCppObject):
        someint: int = ValidatedCppManagedProperty("someint", default=42)

    obj = Obj()
    clone = obj.clone()

    assert repr(obj) != repr(clone)
    assert obj.someint == clone.someint


def test_updateable_mixin():
    class Obj(ValidatedCppModel, UpdateableMixin, cpp_class=MockCppObject):
        someint: int = ValidatedCppManagedProperty("someint", default=42)

    obj = Obj()
    obj.update_from_dict({"someint": 43})
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.update_from_dict({"someint": "Foobar"})

    with pytest.raises(ValueError):
        obj.update_from_dict({"unknown": 42})

    obj.update_from_dict({"unknown": 42}, skip_exceptions=True)
