from helios.validation import *

import copy
import math
import numpy as np
import pytest


class MockCppObject:
    someint = 41
    somestr = "Foobar2"
    somebool = False
    somevec = [0, 1]

    def clone(self):
        return copy.deepcopy(self)


class DerivedMockCppObject(MockCppObject):
    derived = 42


class RelatedCppMockObject:
    other = MockCppObject()


class IterCppMockObject:
    someints = [0, 1]
    somemodels = [MockCppObject(), MockCppObject()]


def test_validated_cpp_properties():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42
        somestr: str = "Foobar"
        somebool: bool = True

    obj = Obj()

    assert obj.someint == 42
    assert obj.somestr == "Foobar"
    assert obj.somebool == True

    obj.someint = 43
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.someint = "Foobar"


def test_validated_noncpp_properties():
    class Obj(Model, cpp_class=MockCppObject):
        foo: int = 42

    obj = Obj()
    assert obj.foo == 42
    assert not hasattr(obj._cpp_object, "foo")


def test_validate_other_model():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

    class RelatedObj(Model, cpp_class=RelatedCppMockObject):
        other: Obj = Obj()

    related = RelatedObj()

    with pytest.raises(ValueError):
        related.other = 42


def test_wrapping_getter_setter():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

    class RelatedObj(Model, cpp_class=RelatedCppMockObject):
        other: Obj = Obj()

    obj = Obj()
    related = RelatedObj(obj)

    assert isinstance(related.other, Obj)
    assert related.other.someint == 42


def test_instantiation():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int
        somestr: str
        somebool: bool = True

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
    class IterObj(Model, cpp_class=MockCppObject):
        somevec: list[int] = [0, 1]

    obj = IterObj()

    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 0
    assert obj.somevec[1] == 1

    obj.somevec = [1, 2]
    assert isinstance(obj.somevec, list)
    assert obj.somevec[0] == 1
    assert obj.somevec[1] == 2


def test_iterable_model():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int

    class IterObj(Model, cpp_class=MockCppObject):
        somevec: list[Model]

    obj1 = Obj(42)
    obj2 = Obj(43)

    iterobj = IterObj([obj1, obj2])

    assert isinstance(iterobj.somevec, list)
    assert isinstance(iterobj.somevec[0], Obj)
    assert isinstance(iterobj.somevec[1], Obj)
    assert iterobj.somevec[0].someint == 42
    assert iterobj.somevec[1].someint == 43


def test_iterable_model_cpp_updates_reflected():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

    class IterObject(Model, cpp_class=IterCppMockObject):
        someints: list[int] = [0, 1]
        somemodels: list[Obj] = []

    obj = IterObject()

    # Length change
    obj._cpp_object.somemodels = [Obj(43)._cpp_object]
    assert len(obj.somemodels) == 1
    assert obj.somemodels[0].someint == 43

    # Only object changed
    obj._cpp_object.somemodels[0] = Obj(44)._cpp_object
    assert obj.somemodels[0].someint == 44


def test_cpp_update_reflected_simple():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

    obj = Obj()
    obj._cpp_object.someint = 43

    assert obj.someint == 43


def test_cpp_update_reflected_other_model():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

    class RelatedObj(Model, cpp_class=RelatedCppMockObject):
        other: Obj = Obj()

    related = RelatedObj()

    assert related.other.someint == 42

    related.other._cpp_object.someint = 43
    assert related.other.someint == 43


def test_repr():
    class Obj(Model, cpp_class=MockCppObject):
        pass

    x1 = Obj()
    x2 = Obj()

    assert repr(x1) != repr(x2)

    x2._cpp_object = x1._cpp_object
    assert repr(x1) == repr(x2)


def test_repr_non_cpp():
    class Obj(Model):
        pass

    x1 = Obj()
    x2 = Obj()

    assert repr(x1) != repr(x2)


def test_updateable_mixin():
    class Obj(Model, UpdateableMixin, cpp_class=MockCppObject):
        someint: int = 42

    obj = Obj()
    obj.update_from_dict({"someint": 43})
    assert obj.someint == 43

    with pytest.raises(ValueError):
        obj.update_from_dict({"someint": "Foobar"})

    with pytest.raises(ValueError):
        obj.update_from_dict({"unknown": 42})

    obj.update_from_dict({"unknown": 42}, skip_exceptions=True)


def test_updateable_mixin_not_mixing_model():
    class Obj(UpdateableMixin):
        someint: int = 42

    class Obj2:
        pass

    with pytest.raises(ValueError):
        Obj().update_from_object(Obj2())


def test_non_cpp_property():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42
        noncpp: int = 42

    obj = Obj()
    assert obj.noncpp == 42

    obj.noncpp = 43
    assert obj.noncpp == 43

    obj2 = Obj()
    assert obj2.noncpp == 42
    assert obj.noncpp == 43


def test_no_cpp_class():
    class Obj(Model):
        noncpp: int = 42

    obj = Obj()
    assert obj.noncpp == 42

    obj.noncpp = 43
    assert obj.noncpp == 43


def test_none_default():
    class Obj(Model, cpp_class=MockCppObject):
        someint: Union[int, None] = None

    obj = Obj()
    assert obj.someint is None


def test_missing_required():
    class Obj(Model, cpp_class=MockCppObject):
        someint: Union[int, None]

    with pytest.raises(ValueError):
        Obj()


def test_enforce_uniqueness_across_instances():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

        def _pre_set(self, field, value):
            if field == "someint":
                self._enforce_uniqueness_across_instances(field, value)

    obj1 = Obj()
    obj2 = Obj(43)

    obj2.someint = 43

    with pytest.raises(ValueError):
        obj1.someint = 43


def test_enforce_uniqueness_across_instances_list():
    class Obj(Model):
        ints: list[int] = [42]

        def _pre_set(self, field, value):
            if field == "ints":
                self._enforce_uniqueness_across_instances(field, value)

    obj1 = Obj()
    obj2 = Obj([43])

    with pytest.raises(ValueError):
        obj1.ints = [43]


def test_post_set_hook():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 42

        def _post_set(self, field):
            if self.someint == 100:
                raise ValueError

    obj = Obj()
    obj.someint = 99

    with pytest.raises(ValueError):
        obj.someint = 100


def test_clone_with_cpp():
    class Obj(Model, cpp_class=MockCppObject):
        somestr: str
        someint: int = 42

    obj = Obj("foo", someint=43)
    obj2 = obj.clone()

    assert obj.someint == obj2.someint
    assert obj.somestr == obj2.somestr
    assert obj is not obj2

    obj2.someint = 44
    assert obj.someint != obj2.someint


def test_clone_with_nonsupporting_cpp_object():
    class Obj(Model, cpp_class=RelatedCppMockObject):
        somestr: str

    obj = Obj("foo")

    with pytest.raises(ValueError):
        obj.clone()


def test_clone_without_cpp():
    class Obj(Model):
        somestr: str
        someint: int = 42

    obj = Obj("foo", someint=43)
    obj2 = obj.clone()

    assert obj.someint == obj2.someint
    assert obj.somestr == obj2.somestr
    assert obj is not obj2

    obj2.someint = 44
    assert obj.someint != obj2.someint


def test_threadcount_annotation():
    @validate_call
    def foo(count: ThreadCount):
        return count

    assert foo(1) == 1
    assert foo(None) > 0

    with pytest.raises(ValueError):
        assert foo(1000000)

    with pytest.raises(ValueError):
        assert foo(0)

    with pytest.raises(ValueError):
        assert foo(-1)


def test_assetpath_annotation():
    @validate_call
    def foo(path: AssetPath):
        return path

    assert isinstance(foo("data/scanners_als.xml"), Path)

    with pytest.raises(FileNotFoundError):
        foo("data/scanners_als2.xml")


def test_created_directory_annotation(tmp_path):
    @validate_call
    def foo(path: CreatedDirectory):
        return path

    foo(tmp_path / "nonexistent")
    assert (tmp_path / "nonexistent").exists()


def test_multiassetpath(assetdir):
    assert (b := assetdir / "b" / "bb" / "other.obj").exists()

    @validate_call
    def from_obj(obj_file: MultiAssetPath):
        return obj_file

    # tmp_dir has different parents, because it gets called from different functions
    assert b.parts[-4:] == from_obj("root/b/*/*.obj")[0].parts[-4:]


def test_angle_annotation():
    class Obj(Model):
        angle: Angle

    # Testing all sorts of valid angle values
    assert Obj(angle=1.0).angle == 1.0
    assert Obj(angle=1 * units.rad).angle == 1.0
    assert Obj(angle="1 rad").angle == 1.0
    assert np.isclose(Obj(angle="180 deg").angle, math.pi)
    assert np.isclose(Obj(angle=180 * units.deg).angle, math.pi)


def test_anglevelocity_annotation():
    class Obj(Model):
        anglevel: Optional[AngleVelocity] = None

    # Testing all sorts of valid angle velocity values
    assert Obj(anglevel=1.0).anglevel == 1.0
    assert Obj(anglevel=1 * units.rad / units.s).anglevel == 1.0
    assert Obj(anglevel="1 rad/s").anglevel == 1.0
    assert np.isclose(Obj(anglevel="180 deg/s").anglevel, math.pi)
    assert np.isclose(Obj(anglevel=180 * units.deg / units.s).anglevel, math.pi)
    assert np.isclose(Obj(anglevel=360 * units.deg / (2.0 * units.s)).anglevel, math.pi)


def test_frequency_annotation():
    class Obj(Model):
        frequency: Frequency

    # Testing all sorts of frequency values
    assert Obj(frequency=1.0).frequency == 1.0
    assert Obj(frequency=1 * units.Hz).frequency == 1.0
    assert Obj(frequency="1 Hz").frequency == 1.0
    assert Obj(frequency="1 kHz").frequency == 1000.0
    assert Obj(frequency=1000 * units.mHz).frequency == 1.0
    assert Obj(frequency=1 * units.kHz).frequency == 1000.0
    assert Obj(frequency=1 * units.MHz).frequency == 1_000_000.0
    assert Obj(frequency=1 * units.GHz).frequency == 1_000_000_000.0

    with pytest.raises(ValueError):
        Obj(frequency=-1.0)

    with pytest.raises(ValueError):
        Obj(frequency="-1 Hz")


def test_length_annotation():
    class Obj(Model):
        length: Length

    # Testing all sorts of valid length values
    assert Obj(length=1.0).length == 1.0
    assert Obj(length=1 * units.m).length == 1.0
    assert Obj(length="1 m").length == 1.0
    assert Obj(length="1 km").length == 1000.0
    assert Obj(length=1000 * units.mm).length == 1.0
    assert Obj(length=1 * units.cm).length == 0.01
    assert Obj(length=1 * units.km).length == 1000.0

    with pytest.raises(ValueError):
        Obj(length=-1.0)

    with pytest.raises(ValueError):
        Obj(length="-1 m")


def test_timeinterval_annotation():
    class Obj(Model):
        timeinterval: TimeInterval

    # Testing all sorts of time interval values
    assert Obj(timeinterval=1.0).timeinterval == 1.0
    assert Obj(timeinterval=1 * units.s).timeinterval == 1.0
    assert Obj(timeinterval="1 s").timeinterval == 1.0
    assert Obj(timeinterval="1 ms").timeinterval == 0.001
    assert Obj(timeinterval=1000 * units.us).timeinterval == 0.001
    assert Obj(timeinterval=1 * units.ms).timeinterval == 0.001
    assert Obj(timeinterval=1 * units.min).timeinterval == 60.0

    with pytest.raises(ValueError):
        Obj(timeinterval=-1.0)

    with pytest.raises(ValueError):
        Obj(timeinterval="-1 s")


def test_is_iterable_annotation():
    from helios.validation import _is_iterable_annotation

    assert _is_iterable_annotation(bool) == False
    assert _is_iterable_annotation(str) == False
    assert _is_iterable_annotation(Union[int, str]) == False
    assert _is_iterable_annotation(list[int]) == True
    assert _is_iterable_annotation(tuple[int]) == True


def test_is_iterable_of_model_annotation():
    from helios.validation import _is_iterable_of_model_annotation

    class Obj(Model):
        pass

    assert _is_iterable_of_model_annotation(Union[int, str]) == False
    assert _is_iterable_of_model_annotation(list[int]) == False
    assert _is_iterable_of_model_annotation(list[Obj]) == True


def test_model_hierarchy():
    class X(Model):
        x: int = 1

    class Y(X):
        y: int = 2

    class Z(Y):
        z: int = 3

    z = Z(x=10, y=20, z=30)
    assert z.x == 10
    assert z.y == 20
    assert z.z == 30


def test_model_instantiated_with_invalid_fields():
    class Obj(Model, cpp_class=MockCppObject):
        someint: int = 41

    with pytest.raises(ValueError):
        Obj(unknown_field=100)
