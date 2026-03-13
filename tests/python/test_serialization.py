import copy
import os
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

import helios.serialization as serialization
from helios.platforms import DynamicPlatformSettings, Platform, TrajectorySettings
from helios.serialization import _MIGRATIONS, register_migration
from helios.scanner import Scanner
from helios.scene import ScenePart, StaticScene
from helios.settings import ExecutionSettings, OutputFormat
from helios.survey import Survey
from helios.utils import add_asset_directory, get_asset_directories, set_rng_seed
from helios.validation import Model


class SerializationLeaf(Model):
    value: int = 0


class SerializationRoot(Model):
    leaf: SerializationLeaf = SerializationLeaf()
    label: str = "default"
    values: tuple[int, ...] = (1, 2, 3)


class HookedFilenameRoot(Model):
    leaf: SerializationLeaf = SerializationLeaf()

    def _serialization_filename(self) -> str:
        return "custom_root.yaml"


class OperationModel(Model):
    value: int = 0

    @classmethod
    def make(cls):
        return cls()


class RecursivePayloadModel(Model):
    payload: object = None


def _build_manual_survey() -> Survey:
    scanner = Scanner.from_xml("data/scanners_tls.xml", scanner_id="riegl_vz400")
    platform = Platform.from_xml("data/platforms.xml", platform_id="tripod")
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    survey = Survey(scanner=scanner, platform=platform, scene=scene, name="manual")
    survey.add_leg(
        x=0.0,
        y=0.0,
        z=0.0,
        pulse_frequency=2000,
        scan_angle="20 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="10 deg",
        scan_frequency=120,
    )
    return survey


def _build_xml_survey() -> Survey:
    return Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")


def _run_survey_npy(survey: Survey):
    execution_settings = ExecutionSettings(num_threads=1)
    set_rng_seed(42)
    return survey.run(format=OutputFormat.NPY, execution_settings=execution_settings)


def _assert_point_cloud_equal(
    points: np.ndarray, loaded_points: np.ndarray, ignore_fields: tuple[str, ...] = ()
):
    if not ignore_fields:
        np.testing.assert_array_equal(points, loaded_points)
        return

    points_cmp = points.copy()
    loaded_points_cmp = loaded_points.copy()
    for field_name in ignore_fields:
        if field_name in points_cmp.dtype.names:
            points_cmp[field_name] = 0
            loaded_points_cmp[field_name] = 0

    np.testing.assert_array_equal(points_cmp, loaded_points_cmp)


def test_roundtrip_yaml_inline(tmp_path):
    root = DynamicPlatformSettings(
        trajectory_settings=TrajectorySettings(
            start_time=1.5, end_time=9.0, teleport_to_start=True
        ),
        speed_m_s=42.0,
        x=1.0,
        y=2.0,
        z=3.0,
    )

    yaml_path = tmp_path / "inline.yaml"
    written = root.to_yaml(yaml_path, shallow=False)
    assert written == yaml_path

    with yaml_path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    assert "model_ref" not in str(document["fields"]["trajectory_settings"])
    assert document["serialization_major_version"] == 0
    assert document["serialization_minor_version"] == 0

    loaded = DynamicPlatformSettings.from_yaml(yaml_path)
    assert loaded.speed_m_s == 42.0
    assert loaded.trajectory_settings.start_time == 1.5
    assert loaded.trajectory_settings.end_time == 9.0
    assert loaded.trajectory_settings.teleport_to_start is True


def test_roundtrip_yaml_shallow_with_references(tmp_path):
    root = DynamicPlatformSettings(
        trajectory_settings=TrajectorySettings(
            start_time=2.0, end_time=8.0, teleport_to_start=False
        ),
        speed_m_s=33.0,
        x=7.0,
        y=8.0,
        z=9.0,
    )

    root_yaml = root.to_yaml(tmp_path, shallow=True)
    assert root_yaml == tmp_path / "dynamicplatformsettings.helios.yaml"

    with root_yaml.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    trajectory_ref = document["fields"]["trajectory_settings"]["model_ref"]
    assert (tmp_path / trajectory_ref).exists()

    loaded = DynamicPlatformSettings.from_yaml(root_yaml)
    assert loaded.speed_m_s == 33.0
    assert loaded.trajectory_settings.start_time == 2.0
    assert loaded.trajectory_settings.end_time == 8.0
    assert loaded.trajectory_settings.teleport_to_start is False


def test_serialization_filename_hook_is_used(tmp_path):
    root = HookedFilenameRoot(leaf=SerializationLeaf(value=3))

    root_yaml = root.to_yaml(tmp_path, shallow=True)
    assert root_yaml == tmp_path / "custom_root.yaml"
    assert root_yaml.exists()


def test_invalid_yaml_fails_schema_validation(tmp_path):
    yaml_path = tmp_path / "invalid.yaml"
    with yaml_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(
            {
                "serialization_major_version": 0,
                "serialization_minor_version": 0,
                "fields": {},
            },
            handle,
        )

    with pytest.raises(ValueError, match="Invalid YAML serialization format"):
        DynamicPlatformSettings.from_yaml(yaml_path)


def test_from_yaml_raises_for_missing_file(tmp_path):
    missing = tmp_path / "does_not_exist.yaml"
    with pytest.raises(FileNotFoundError, match="YAML file not found"):
        DynamicPlatformSettings.from_yaml(missing)


def test_to_yaml_existing_non_yaml_path_gets_yaml_suffix(tmp_path):
    root = DynamicPlatformSettings(
        trajectory_settings=TrajectorySettings(
            start_time=1.0, end_time=2.0, teleport_to_start=False
        ),
        speed_m_s=15.0,
        x=0.0,
        y=0.0,
        z=0.0,
    )
    existing_non_yaml = tmp_path / "target.txt"
    existing_non_yaml.write_text("placeholder", encoding="utf-8")

    written = root.to_yaml(existing_non_yaml, shallow=False)
    assert written == tmp_path / "target.helios.yaml"
    assert written.exists()


def test_to_bundle_errors_if_target_path_is_file(tmp_path):
    bundle_target = tmp_path / "bundle_target"
    bundle_target.write_text("not a directory", encoding="utf-8")

    with pytest.raises(ValueError, match="Bundle path must be a directory"):
        SerializationLeaf(value=5).to_bundle(bundle_target)


def test_register_migration_registers_and_rejects_negative_minor_version():
    with pytest.raises(ValueError, match="must be non-negative"):
        register_migration(-1)

    previous = dict(_MIGRATIONS)
    try:

        @register_migration(9999)
        def _dummy_migration(document):
            migrated = dict(document)
            migrated["serialization_minor_version"] = (
                migrated["serialization_minor_version"] + 1
            )
            return migrated

        assert _MIGRATIONS[9999] is _dummy_migration
    finally:
        _MIGRATIONS.clear()
        _MIGRATIONS.update(previous)


def test_inline_cycle_detection_raises_for_non_shallow_serialization(tmp_path):
    model = RecursivePayloadModel()
    model.payload = model

    with pytest.raises(ValueError, match="Detected cyclic model graph"):
        model.to_yaml(tmp_path / "recursive.yaml", shallow=False)


def test_load_model_from_file_uses_cache(tmp_path):
    yaml_path = DynamicPlatformSettings().to_yaml(
        tmp_path / "settings.yaml", shallow=False
    )
    context = serialization._DeserializationContext()

    first = serialization._load_model_from_file(yaml_path, context)
    second = serialization._load_model_from_file(yaml_path, context)

    assert first is second


def test_from_yaml_raises_for_cyclic_model_references(tmp_path):
    dynamic_yaml = tmp_path / "dynamic.yaml"
    platform_yaml = tmp_path / "platform.yaml"

    platform_doc = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "helios.platforms.Platform",
        "fields": {"platform_settings": {"model_ref": dynamic_yaml.name}},
    }
    dynamic_doc = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "helios.platforms.DynamicPlatformSettings",
        "fields": {"trajectory_settings": {"model_ref": platform_yaml.name}},
    }

    platform_yaml.write_text(
        yaml.safe_dump(platform_doc, sort_keys=False), encoding="utf-8"
    )
    dynamic_yaml.write_text(
        yaml.safe_dump(dynamic_doc, sort_keys=False), encoding="utf-8"
    )

    with pytest.raises(ValueError, match="Cyclic model reference detected"):
        Platform.from_yaml(platform_yaml)


def test_from_yaml_raises_for_non_mapping_yaml_document(tmp_path):
    yaml_path = tmp_path / "list.yaml"
    yaml_path.write_text("- 1\n- 2\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Expected YAML document object"):
        DynamicPlatformSettings.from_yaml(yaml_path)


def test_deserialize_model_from_yaml_rejects_non_instance_result(tmp_path, monkeypatch):
    yaml_path = tmp_path / "dummy.yaml"
    yaml_path.write_text("{}", encoding="utf-8")

    monkeypatch.setattr(
        serialization, "_load_model_from_file", lambda *_args, **_kw: []
    )
    with pytest.raises(
        TypeError,
        match="Expected YAML to deserialize into DynamicPlatformSettings, got list",
    ):
        serialization.deserialize_model_from_yaml(DynamicPlatformSettings, yaml_path)


def test_binary_deserialization_requires_existing_loader(tmp_path):
    yaml_path = tmp_path / "platform_binary.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "serialization_major_version": 0,
                "serialization_minor_version": 0,
                "model_class": "helios.platforms.Platform",
                "fields": {},
                "binary": {"method": "from_binary", "path": "platform.bin"},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Binary method 'from_binary' not found"):
        Platform.from_yaml(yaml_path)


def test_binary_deserialization_rejects_unknown_loader_method(tmp_path):
    yaml_path = tmp_path / "platform_binary_unknown.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "serialization_major_version": 0,
                "serialization_minor_version": 0,
                "model_class": "helios.platforms.Platform",
                "fields": {},
                "binary": {"method": "missing_loader", "path": "platform.bin"},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Binary method 'missing_loader' not found"):
        Platform.from_yaml(yaml_path)


def test_provenance_deserialization_rejects_unknown_constructor(tmp_path):
    yaml_path = tmp_path / "settings_unknown_ctor.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "serialization_major_version": 0,
                "serialization_minor_version": 0,
                "model_class": "helios.platforms.DynamicPlatformSettings",
                "fields": {},
                "provenance": {
                    "constructor": {"method": "does_not_exist", "kwargs": {}}
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(
        ValueError, match="Provenance constructor 'does_not_exist' not found"
    ):
        DynamicPlatformSettings.from_yaml(yaml_path)


def test_provenance_deserialization_rejects_operations_without_constructor(tmp_path):
    yaml_path = tmp_path / "settings_ops_without_ctor.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "serialization_major_version": 0,
                "serialization_minor_version": 0,
                "model_class": "helios.platforms.DynamicPlatformSettings",
                "fields": {},
                "provenance": {"operations": [{"method": "rotate", "kwargs": {}}]},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Cannot replay provenance operations"):
        DynamicPlatformSettings.from_yaml(yaml_path)


def test_deserialize_model_document_rejects_missing_provenance_operation_method(
    tmp_path,
):
    document = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": f"{OperationModel.__module__}.{OperationModel.__qualname__}",
        "fields": {},
        "provenance": {
            "constructor": {"method": "make", "kwargs": {}},
            "operations": [{"method": "missing_method", "kwargs": {}}],
        },
    }

    with pytest.raises(
        ValueError, match="Provenance operation 'missing_method' not found"
    ):
        serialization._deserialize_model_document(
            document=document,
            expected_cls=None,
            source_dir=tmp_path,
            context=serialization._DeserializationContext(),
        )


def test_deserialize_model_document_rejects_invalid_class_identifier(tmp_path):
    document = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "InvalidClassName",
        "fields": {},
    }

    with pytest.raises(ValueError, match="Invalid model class identifier"):
        serialization._deserialize_model_document(
            document=document,
            expected_cls=None,
            source_dir=tmp_path,
            context=serialization._DeserializationContext(),
        )


def test_deserialize_model_document_rejects_unresolvable_class(tmp_path):
    document = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "helios.platforms.DoesNotExist",
        "fields": {},
    }

    with pytest.raises(ValueError, match="Cannot resolve model class"):
        serialization._deserialize_model_document(
            document=document,
            expected_cls=None,
            source_dir=tmp_path,
            context=serialization._DeserializationContext(),
        )


def test_deserialize_model_document_rejects_non_model_class(tmp_path):
    document = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "helios.platforms.Printable",
        "fields": {},
    }

    with pytest.raises(TypeError, match="is not a Model subclass"):
        serialization._deserialize_model_document(
            document=document,
            expected_cls=None,
            source_dir=tmp_path,
            context=serialization._DeserializationContext(),
        )


def test_deserialize_model_document_rejects_incompatible_expected_type(tmp_path):
    document = {
        "serialization_major_version": 0,
        "serialization_minor_version": 0,
        "model_class": "helios.platforms.PlatformSettings",
        "fields": {},
    }

    with pytest.raises(
        TypeError, match="is not compatible with DynamicPlatformSettings"
    ):
        serialization._deserialize_model_document(
            document=document,
            expected_cls=DynamicPlatformSettings,
            source_dir=tmp_path,
            context=serialization._DeserializationContext(),
        )


def test_apply_migrations_raises_if_migration_missing(monkeypatch):
    previous_minor = serialization.SERIALIZATION_FORMAT_MINOR_VERSION
    previous = dict(serialization._MIGRATIONS)
    monkeypatch.setattr(serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", 1)
    serialization._MIGRATIONS.clear()
    try:
        with pytest.raises(
            ValueError, match="No migration available from minor version 0"
        ):
            serialization._apply_migrations(
                {
                    "serialization_major_version": 0,
                    "serialization_minor_version": 0,
                    "model_class": "helios.scanner.Scanner",
                    "fields": {},
                },
                None,
            )
    finally:
        monkeypatch.setattr(
            serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", previous_minor
        )
        serialization._MIGRATIONS.clear()
        serialization._MIGRATIONS.update(previous)


def test_apply_migrations_rejects_invalid_major_version_from_migration(monkeypatch):
    previous_minor = serialization.SERIALIZATION_FORMAT_MINOR_VERSION
    previous = dict(serialization._MIGRATIONS)
    monkeypatch.setattr(serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", 1)
    serialization._MIGRATIONS.clear()

    @register_migration(0)
    def _invalid_major(document):
        migrated = dict(document)
        migrated["serialization_major_version"] = 99
        migrated["serialization_minor_version"] = 1
        return migrated

    try:
        with pytest.raises(ValueError, match="Invalid migration result: expected"):
            serialization._apply_migrations(
                {
                    "serialization_major_version": 0,
                    "serialization_minor_version": 0,
                    "model_class": "helios.scanner.Scanner",
                    "fields": {},
                },
                None,
            )
    finally:
        monkeypatch.setattr(
            serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", previous_minor
        )
        serialization._MIGRATIONS.clear()
        serialization._MIGRATIONS.update(previous)


def test_apply_migrations_rejects_non_increasing_minor_version(monkeypatch):
    previous_minor = serialization.SERIALIZATION_FORMAT_MINOR_VERSION
    previous = dict(serialization._MIGRATIONS)
    monkeypatch.setattr(serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", 1)
    serialization._MIGRATIONS.clear()

    @register_migration(0)
    def _non_increasing_minor(document):
        migrated = dict(document)
        migrated["serialization_minor_version"] = 0
        return migrated

    try:
        with pytest.raises(ValueError, match="serialization_minor_version > 0"):
            serialization._apply_migrations(
                {
                    "serialization_major_version": 0,
                    "serialization_minor_version": 0,
                    "model_class": "helios.scanner.Scanner",
                    "fields": {},
                },
                None,
            )
    finally:
        monkeypatch.setattr(
            serialization, "SERIALIZATION_FORMAT_MINOR_VERSION", previous_minor
        )
        serialization._MIGRATIONS.clear()
        serialization._MIGRATIONS.update(previous)


def test_clone_and_deepcopy_keep_provenance():
    obj = SerializationLeaf(value=9)
    obj._yaml_serializable = False
    obj._provenance = {
        "constructor": {"method": "from_example", "kwargs": {"path": "example.dat"}},
        "operations": [{"method": "shift", "kwargs": {"x": 1}}],
    }

    cloned = obj.clone()
    copied = copy.deepcopy(obj)

    assert cloned._provenance == obj._provenance
    assert copied._provenance == obj._provenance
    assert cloned._provenance is not obj._provenance
    assert copied._provenance is not obj._provenance
    assert cloned._yaml_serializable is False
    assert copied._yaml_serializable is False


def test_scenepart_transform_provenance_roundtrip(tmp_path):
    part = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    part.translate([1.0, 2.0, 3.0]).scale(1.2).rotate(axis=[0, 0, 1], angle=np.pi / 8)

    yaml_path = part.to_yaml(tmp_path / "scenepart.yaml", shallow=False)
    loaded = ScenePart.from_yaml(yaml_path)

    assert loaded._provenance["constructor"]["method"] == "from_obj"
    assert [op["method"] for op in loaded._provenance["operations"]] == [
        "translate",
        "scale",
        "rotate",
    ]
    np.testing.assert_allclose(loaded.bbox.bounds[0], part.bbox.bounds[0], atol=1e-8)
    np.testing.assert_allclose(loaded.bbox.bounds[1], part.bbox.bounds[1], atol=1e-8)


def test_static_scene_yaml_roundtrip(tmp_path):
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )

    root_yaml = scene.to_yaml(tmp_path, shallow=True)
    with root_yaml.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)

    assert "binary" not in document

    loaded = StaticScene.from_yaml(root_yaml)
    scene._finalize()
    loaded._finalize()
    assert len(scene._cpp_object.primitives) == len(loaded._cpp_object.primitives)


def test_from_binary_to_yaml_uses_provenance_reference(tmp_path):
    source = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    binary_path = tmp_path / "scene.bin"
    source.to_binary(binary_path)

    loaded_from_binary = StaticScene.from_binary(binary_path)
    yaml_path = loaded_from_binary.to_yaml(tmp_path / "scene_from_binary.yaml")

    with yaml_path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)

    assert "binary" not in document
    assert document["provenance"]["constructor"]["method"] == "from_binary"
    assert document["fields"] == {}


def test_from_constructors_store_provenance():
    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    platform = Platform.from_xml("data/platforms.xml", platform_id="tripod")
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    assert scanner._provenance["constructor"]["method"] == "from_xml"
    assert scanner._provenance["constructor"]["kwargs"]["scanner_id"] == "leica_als50"
    assert platform._provenance["constructor"]["method"] == "from_xml"
    assert platform._provenance["constructor"]["kwargs"]["platform_id"] == "tripod"
    assert survey._provenance["constructor"]["method"] == "from_xml"
    assert scanner._yaml_serializable is True
    assert platform._yaml_serializable is True
    assert survey._yaml_serializable is True
    assert survey.scene._yaml_serializable is False
    assert survey.scanner._yaml_serializable is False
    assert survey.platform._yaml_serializable is False
    assert all(not leg._yaml_serializable for leg in survey.legs)


def test_xml_loaded_roots_serialize_with_provenance(tmp_path):
    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    platform = Platform.from_xml("data/platforms.xml", platform_id="tripod")
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    scanner_yaml = scanner.to_yaml(tmp_path / "scanner.yaml")
    with scanner_yaml.open("r", encoding="utf-8") as handle:
        scanner_document = yaml.safe_load(handle)
    assert scanner_document["provenance"]["constructor"]["method"] == "from_xml"
    assert (
        scanner_document["provenance"]["constructor"]["kwargs"]["scanner_id"]
        == "leica_als50"
    )
    scanner_file = scanner_document["provenance"]["constructor"]["kwargs"][
        "scanner_file"
    ]
    assert not Path(scanner_file).is_absolute()

    platform_yaml = platform.to_yaml(tmp_path / "platform.yaml")
    with platform_yaml.open("r", encoding="utf-8") as handle:
        platform_document = yaml.safe_load(handle)
    assert platform_document["provenance"]["constructor"]["method"] == "from_xml"
    assert (
        platform_document["provenance"]["constructor"]["kwargs"]["platform_id"]
        == "tripod"
    )
    platform_file = platform_document["provenance"]["constructor"]["kwargs"][
        "platform_file"
    ]
    assert not Path(platform_file).is_absolute()

    survey_yaml = survey.to_yaml(tmp_path / "survey.yaml")
    with survey_yaml.open("r", encoding="utf-8") as handle:
        survey_document = yaml.safe_load(handle)
    assert survey_document["provenance"]["constructor"]["method"] == "from_xml"
    survey_file = survey_document["provenance"]["constructor"]["kwargs"]["survey_file"]
    assert not Path(survey_file).is_absolute()


def test_xml_implicit_children_cannot_serialize_directly(tmp_path):
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")
    assert len(survey.legs) > 0

    with pytest.raises(RuntimeError, match="implicitly constructed"):
        survey.scene.to_yaml(tmp_path / "scene.yaml")

    with pytest.raises(RuntimeError, match="implicitly constructed"):
        survey.scanner.to_yaml(tmp_path / "scanner.yaml")

    with pytest.raises(RuntimeError, match="implicitly constructed"):
        survey.platform.to_yaml(tmp_path / "platform.yaml")

    with pytest.raises(RuntimeError, match="implicitly constructed"):
        survey.legs[0].to_yaml(tmp_path / "leg.yaml")


def test_nested_xml_loaded_components_keep_from_xml_provenance(tmp_path):
    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    platform = Platform.from_xml("data/platforms.xml", platform_id="tripod")
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(x=0.0, y=0.0, z=0.0)

    survey_yaml = survey.to_yaml(tmp_path / "survey_inline.yaml", shallow=False)
    with survey_yaml.open("r", encoding="utf-8") as handle:
        survey_document = yaml.safe_load(handle)

    assert (
        survey_document["fields"]["scanner"]["provenance"]["constructor"]["method"]
        == "from_xml"
    )
    assert (
        survey_document["fields"]["platform"]["provenance"]["constructor"]["method"]
        == "from_xml"
    )


@pytest.mark.parametrize(
    ("constructor", "shallow"),
    [
        ("manual", False),
        ("manual", True),
        ("xml", False),
        ("xml", True),
    ],
)
def test_full_survey_roundtrip_preserves_point_cloud(
    tmp_path, constructor: str, shallow: bool
):
    if constructor == "manual":
        baseline_survey = _build_manual_survey()
        survey_to_serialize = _build_manual_survey()
    else:
        baseline_survey = _build_xml_survey()
        survey_to_serialize = _build_xml_survey()

    points, trajectory = _run_survey_npy(baseline_survey)

    yaml_path = survey_to_serialize.to_yaml(
        tmp_path / f"survey_{constructor}_{shallow}.yaml",
        shallow=shallow,
    )
    assert yaml_path.exists()

    loaded = Survey.from_yaml(yaml_path)

    loaded_points, loaded_trajectory = _run_survey_npy(loaded)

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0
    if constructor == "xml":
        _assert_point_cloud_equal(
            points,
            loaded_points,
            ignore_fields=("channel_id", "hit_object_id"),
        )
    else:
        _assert_point_cloud_equal(points, loaded_points)
    np.testing.assert_array_equal(trajectory, loaded_trajectory)


@pytest.mark.parametrize(
    ("constructor", "binary"),
    [
        ("manual", False),
        ("manual", True),
        ("xml", False),
        ("xml", True),
    ],
)
def test_full_survey_bundle_roundtrip_preserves_point_cloud(
    tmp_path, constructor: str, binary: bool
):
    if constructor == "manual":
        baseline_survey = _build_manual_survey()
        survey_to_serialize = _build_manual_survey()
    else:
        baseline_survey = _build_xml_survey()
        survey_to_serialize = _build_xml_survey()

    points, trajectory = _run_survey_npy(baseline_survey)

    bundle_dir = tmp_path / f"bundle_{constructor}_{binary}"
    root_yaml = survey_to_serialize.to_bundle(bundle_dir, binary=binary)
    assert root_yaml.exists()

    if binary:
        with root_yaml.open("r", encoding="utf-8") as handle:
            root_document = yaml.safe_load(handle)
        scene_ref = root_document["fields"]["scene"]["model_ref"]
        scene_yaml = bundle_dir / scene_ref
        with scene_yaml.open("r", encoding="utf-8") as handle:
            scene_document = yaml.safe_load(handle)
        assert "binary" in scene_document
        assert (bundle_dir / scene_document["binary"]["path"]).exists()

    loaded = Survey.from_bundle(bundle_dir)
    loaded_points, loaded_trajectory = _run_survey_npy(loaded)

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0
    if constructor == "xml":
        _assert_point_cloud_equal(
            points,
            loaded_points,
            ignore_fields=("channel_id", "hit_object_id"),
        )
    else:
        _assert_point_cloud_equal(points, loaded_points)
    np.testing.assert_array_equal(trajectory, loaded_trajectory)


def test_to_bundle_errors_on_non_empty_directory_without_force(tmp_path):
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    stale = bundle_dir / "stale.txt"
    stale.write_text("stale", encoding="utf-8")

    model = SerializationLeaf(value=5)
    with pytest.raises(RuntimeError, match="not empty"):
        model.to_bundle(bundle_dir)

    root_yaml = model.to_bundle(bundle_dir, force=True)
    assert root_yaml.exists()
    assert not stale.exists()


def test_from_bundle_uses_class_default_filename(tmp_path):
    model = DynamicPlatformSettings(speed_m_s=5.0)
    bundle_dir = tmp_path / "bundle"
    model.to_bundle(bundle_dir)

    loaded = DynamicPlatformSettings.from_bundle(bundle_dir)
    assert isinstance(loaded, DynamicPlatformSettings)
    assert loaded.speed_m_s == 5.0


def test_from_bundle_allows_explicit_filename(tmp_path):
    model = DynamicPlatformSettings(speed_m_s=9.0)
    bundle_dir = tmp_path / "bundle"
    root_yaml = model.to_bundle(bundle_dir)

    loaded = DynamicPlatformSettings.from_bundle(bundle_dir, filename=root_yaml.name)
    assert isinstance(loaded, DynamicPlatformSettings)
    assert loaded.speed_m_s == 9.0


def test_from_bundle_uses_custom_default_filename_hook(tmp_path, monkeypatch):
    monkeypatch.setattr(
        DynamicPlatformSettings,
        "_serialization_filename",
        lambda _self: "custom_root.yaml",
    )
    model = DynamicPlatformSettings(speed_m_s=11.0)
    bundle_dir = tmp_path / "bundle"
    model.to_bundle(bundle_dir)

    loaded = DynamicPlatformSettings.from_bundle(bundle_dir)
    assert isinstance(loaded, DynamicPlatformSettings)
    assert loaded.speed_m_s == 11.0


def test_manual_survey_to_bundle_roundtrip_with_assets(tmp_path):
    survey = _build_manual_survey()
    bundle_dir = tmp_path / "bundle"

    root_yaml = survey.to_bundle(bundle_dir, binary=False)
    assert root_yaml.exists()

    with root_yaml.open("r", encoding="utf-8") as handle:
        survey_doc = yaml.safe_load(handle)

    scanner_yaml = bundle_dir / survey_doc["fields"]["scanner"]["model_ref"]
    platform_yaml = bundle_dir / survey_doc["fields"]["platform"]["model_ref"]
    scene_yaml = bundle_dir / survey_doc["fields"]["scene"]["model_ref"]
    assert scanner_yaml.exists()
    assert platform_yaml.exists()
    assert scene_yaml.exists()

    with scanner_yaml.open("r", encoding="utf-8") as handle:
        scanner_doc = yaml.safe_load(handle)
    scanner_file = scanner_doc["provenance"]["constructor"]["kwargs"]["scanner_file"]
    assert len(Path(scanner_file).parts) == 1
    assert (bundle_dir / scanner_file).exists()

    with platform_yaml.open("r", encoding="utf-8") as handle:
        platform_doc = yaml.safe_load(handle)
    platform_file = platform_doc["provenance"]["constructor"]["kwargs"]["platform_file"]
    assert len(Path(platform_file).parts) == 1
    assert (bundle_dir / platform_file).exists()

    with scene_yaml.open("r", encoding="utf-8") as handle:
        scene_doc = yaml.safe_load(handle)
    assert "binary" not in scene_doc

    scene_part_yaml = bundle_dir / scene_doc["fields"]["scene_parts"][0]["model_ref"]
    with scene_part_yaml.open("r", encoding="utf-8") as handle:
        scene_part_doc = yaml.safe_load(handle)
    obj_file = scene_part_doc["provenance"]["constructor"]["kwargs"]["obj_file"]
    assert len(Path(obj_file).parts) == 1
    bundled_obj = bundle_dir / obj_file
    assert bundled_obj.exists()
    source_obj = Path("data/sceneparts/basic/box/box100.obj")
    assert bundled_obj.read_text(encoding="utf-8") == source_obj.read_text(
        encoding="utf-8"
    )
    assert (bundle_dir / Path(obj_file).with_suffix(".mtl")).exists()
    assert not any(entry.is_dir() for entry in bundle_dir.iterdir())

    with add_asset_directory(bundle_dir):
        loaded = Survey.from_yaml(root_yaml)

    baseline_points, baseline_traj = _run_survey_npy(_build_manual_survey())
    loaded_points, loaded_traj = _run_survey_npy(loaded)
    _assert_point_cloud_equal(baseline_points, loaded_points)
    np.testing.assert_array_equal(baseline_traj, loaded_traj)


def test_scene_bundle_copies_material_files_from_loaded_scene_part(tmp_path):
    source_obj = Path("data/sceneparts/basic/box/box100.obj").resolve()
    source_mtl = source_obj.with_suffix(".mtl")
    staged_obj = tmp_path / source_obj.name
    staged_mtl = tmp_path / source_mtl.name
    shutil.copy2(source_obj, staged_obj)
    shutil.copy2(source_mtl, staged_mtl)

    scene_part = ScenePart.from_obj(staged_obj)
    scene = StaticScene(scene_parts=[scene_part])

    original_obj = staged_obj.read_text(encoding="utf-8")
    stripped_lines = [
        line
        for line in original_obj.splitlines()
        if not line.lstrip().lower().startswith("mtllib ")
    ]
    rewritten_obj = "\n".join(stripped_lines)
    if original_obj.endswith("\n"):
        rewritten_obj += "\n"
    staged_obj.write_text(rewritten_obj, encoding="utf-8")
    assert "mtllib " not in rewritten_obj.lower()

    bundle_dir = tmp_path / "scene_bundle"
    root_yaml = scene.to_bundle(bundle_dir, binary=False)
    assert root_yaml.exists()

    with root_yaml.open("r", encoding="utf-8") as handle:
        scene_doc = yaml.safe_load(handle)
    scene_part_yaml = bundle_dir / scene_doc["fields"]["scene_parts"][0]["model_ref"]

    with scene_part_yaml.open("r", encoding="utf-8") as handle:
        scene_part_doc = yaml.safe_load(handle)
    obj_file = scene_part_doc["provenance"]["constructor"]["kwargs"]["obj_file"]
    assert (bundle_dir / obj_file).exists()

    mtl_files = sorted(bundle_dir.glob("*.mtl"))
    assert [path.name for path in mtl_files] == [source_mtl.name]


def test_to_bundle_passes_binary_flag(tmp_path):
    survey = _build_manual_survey()
    bundle_dir = tmp_path / "bundle_binary"
    root_yaml = survey.to_bundle(bundle_dir, binary=True)

    with root_yaml.open("r", encoding="utf-8") as handle:
        survey_doc = yaml.safe_load(handle)

    scene_yaml = bundle_dir / survey_doc["fields"]["scene"]["model_ref"]
    with scene_yaml.open("r", encoding="utf-8") as handle:
        scene_doc = yaml.safe_load(handle)
    assert "binary" in scene_doc
    assert (bundle_dir / scene_doc["binary"]["path"]).exists()


def test_from_binary_stores_provenance(tmp_path):
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    binary_path = tmp_path / "scene.bin"
    scene.to_binary(binary_path)

    loaded = StaticScene.from_binary(binary_path)
    assert loaded._provenance["constructor"]["method"] == "from_binary"
    recorded_binary_file = loaded._provenance["constructor"]["kwargs"]["binary_file"]
    recorded_path = Path(recorded_binary_file)

    asset_root = Path(get_asset_directories()[0]).expanduser().resolve()
    try:
        os.path.relpath(binary_path.resolve(), start=asset_root)
    except ValueError:
        assert recorded_path.is_absolute()
    else:
        assert not recorded_path.is_absolute()

    assert Path(recorded_binary_file).name == binary_path.name
