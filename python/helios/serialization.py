from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import date, datetime, time
from enum import Enum
from importlib import import_module
from pathlib import Path
from typing import Any, Callable, Optional, Type

from jsonschema import Draft202012Validator
from pydantic import TypeAdapter

from helios.utils import (
    add_asset_directory,
    find_file,
    get_asset_directories,
    is_real_iterable,
)
from helios.validation import Model, get_all_annotations

import importlib_resources as resources
import json
import numpy as np
import os
import shlex
import shutil
import yaml


# Bumping this rejects all documents from previous major versions.
# Do this only when there is strong need to, you will invalidate all
# data that users might have serialized already. You can also delete
# all migrations below in that case, as they are no longer relevant.
SERIALIZATION_FORMAT_MAJOR_VERSION = 0

# Bump this every time you change the serialization format. Alongside
# this bump, you should adapt the schema in python/helios/data/serialization_schema.json
# and add a new migration as described below.
SERIALIZATION_FORMAT_MINOR_VERSION = 0

# The constant used to signal that a model field is serialized in a different file
_MODEL_REFERENCE_KEY = "model_ref"

# Storage for for the lazily loaded schema
_MODEL_DOCUMENT_VALIDATOR: Optional[Draft202012Validator] = None

Migration = Callable[[dict[str, Any]], dict[str, Any]]

# Storage for minor version migrations
_MIGRATIONS: dict[int, Migration] = {}


def register_migration(from_minor_version: int):
    """Register a migration function from a given minor version to the next one.

    Parameters
    ----------
    from_minor_version : int
        Source serialization minor version handled by the decorated migration.

    Returns
    -------
    Callable[[Migration], Migration]
        A decorator that stores the migration for upgrade application.
    """

    if from_minor_version < 0:
        raise ValueError("Migration source minor version must be non-negative.")

    def _register(migration: Migration) -> Migration:
        """Store the given migration under its source minor version key."""

        _MIGRATIONS[from_minor_version] = migration
        return migration

    return _register


def _normalize_provenance_value(value: Any):
    """Convert runtime values into YAML-safe primitives."""

    if isinstance(value, (str, int, float, bool)) or value is None:
        return value

    if isinstance(value, Path):
        path = value.expanduser()
        if not path.is_absolute():
            return str(path)

        asset_dirs = [
            Path(directory).expanduser().resolve()
            for directory in get_asset_directories()
        ]
        relative_candidates = []
        resolved_path = path.resolve()
        for asset_dir in asset_dirs:
            try:
                relative_candidates.append(resolved_path.relative_to(asset_dir))
            except ValueError:
                continue

        if relative_candidates:
            shortest = min(
                relative_candidates,
                key=lambda candidate: (len(candidate.parts), len(str(candidate))),
            )
            return str(shortest)

        if asset_dirs:
            try:
                return os.path.relpath(resolved_path, start=asset_dirs[0])
            except ValueError:
                return str(resolved_path)

        try:
            return os.path.relpath(resolved_path, start=Path.cwd())
        except ValueError:
            return str(resolved_path)

    if isinstance(value, Enum):
        return value.value

    if isinstance(value, (datetime, date, time)):
        return value.isoformat()

    if isinstance(value, np.ndarray):
        return value.tolist()

    if isinstance(value, np.generic):
        return value.item()

    if isinstance(value, Mapping):
        return {
            str(key): _normalize_provenance_value(item) for key, item in value.items()
        }

    if is_real_iterable(value):
        return [_normalize_provenance_value(item) for item in value]

    return str(value)


#
# Minor version migrations. Add one per new minor version.
#

# Example first migration:
# @register_migration(from_minor_version=0)
# def _migrate_v0_to_v1(document: dict[str, Any]) -> dict[str, Any]:
#     migrated = dict(document)
#
#     # Do something with migrated
#
#     migrated["serialization_minor_version"] = 1
#     return migrated


@dataclass
class _SerializationContext:
    root_dir: Path
    shallow: bool
    binary: bool
    model_files: dict[int, str] = field(default_factory=dict)
    document_models: dict[str, Model] = field(default_factory=dict)
    used_yaml_filenames: set[str] = field(default_factory=set)
    binary_files: dict[int, str] = field(default_factory=dict)
    used_binary_filenames: set[str] = field(default_factory=set)
    inline_stack: set[int] = field(default_factory=set)


@dataclass
class _DeserializationContext:
    file_cache: dict[Path, Model] = field(default_factory=dict)
    loading_files: set[Path] = field(default_factory=set)


@dataclass
class _BundleCopyContext:
    bundle_dir: Path
    source_to_target: dict[Path, str] = field(default_factory=dict)
    used_filenames: set[str] = field(default_factory=set)
    obj_material_sources: dict[Path, list[Path]] = field(default_factory=dict)


def serialize_model_to_yaml(model: Model, path: Path, shallow: bool = True) -> Path:
    """Serialize a model to YAML.

    Parameters
    ----------
    model : Model
        Model instance to serialize.
    path : Path
        Destination YAML file path or destination directory.
    shallow : bool, optional
        If ``True``, referenced models are serialized into separate YAML files and
        represented via references. If ``False``, nested models are inlined.

    Returns
    -------
    Path
        Absolute path to the written root YAML file.
    """

    root_file, _ = _serialize_model_to_yaml_with_context(
        model=model,
        path=path,
        shallow=shallow,
        binary=False,
    )
    return root_file


def _serialize_model_to_yaml_with_context(
    model: Model, path: Path, shallow: bool = True, binary: bool = False
) -> tuple[Path, _SerializationContext]:
    """Serialize a model instance into YAML and return the serialization context."""

    if binary and not shallow:
        raise ValueError("binary=True requires shallow=True.")

    default_filename = _ensure_suffix(model._serialization_filename(), ".helios.yaml")
    path = path.expanduser()
    if path.exists():
        if path.is_dir():
            root_file = path / default_filename
        else:
            suffix = path.suffix.lower()
            root_file = (
                path
                if suffix in {".yaml", ".yml"}
                else path.with_suffix(".helios.yaml")
            )
    elif path.suffix.lower() in {".yaml", ".yml"}:
        root_file = path
    else:
        root_file = path / default_filename

    root_file.parent.mkdir(parents=True, exist_ok=True)

    context = _SerializationContext(
        root_dir=root_file.parent,
        shallow=shallow,
        binary=binary,
    )

    root_name = root_file.name
    context.used_yaml_filenames.add(root_name)
    _write_model_document(model, root_name, context)
    return root_file, context


def serialize_model_to_bundle(
    model: Model, path: Path, binary: bool = False, force: bool = False
) -> Path:
    """Serialize a model into a relocatable directory bundle.

    Parameters
    ----------
    model : Model
        Model instance to serialize.
    path : Path
        Target directory for bundle contents.
    binary : bool, optional
        If ``True``, serialize the model payload via ``to_binary`` sidecar files
        where available.
    force : bool, optional
        If ``True``, clear an existing non-empty target directory before writing.

    Returns
    -------
    Path
        Path to the root YAML file inside the created bundle.
    """

    bundle_dir = path.expanduser()
    _prepare_bundle_directory(bundle_dir, force=force)

    with add_asset_directory(bundle_dir):
        root_yaml, serialization_context = _serialize_model_to_yaml_with_context(
            model, path=bundle_dir, shallow=True, binary=binary
        )
        _rewrite_bundle_provenance_paths(
            root_yaml,
            bundle_dir,
            serialization_context.document_models,
        )

    return root_yaml


def deserialize_model_from_yaml(cls: Type[Model], path: Path) -> Model:
    """Deserialize a model instance from YAML.

    Parameters
    ----------
    cls : Type[Model]
        Expected model type.
    path : Path
        Path to a YAML model document.

    Returns
    -------
    Model
        Deserialized model instance of type ``cls``.
    """

    source = path.expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"YAML file not found: {source}")

    context = _DeserializationContext()
    model = _load_model_from_file(source, context, expected_cls=cls)
    if not isinstance(model, cls):
        raise TypeError(
            f"Expected YAML to deserialize into {cls.__name__}, got {type(model).__name__}."
        )
    return model


def _write_model_document(model: Model, filename: str, context: _SerializationContext):
    """Build and persist one model YAML document."""

    model_id = id(model)
    context.model_files[model_id] = filename
    context.document_models[filename] = model

    document = _build_model_document(model, context)
    path = context.root_dir / filename
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(document, handle, sort_keys=False, allow_unicode=False)


def _prepare_bundle_directory(path: Path, force: bool) -> None:
    """Create a writable bundle directory, optionally clearing prior contents."""

    if path.exists() and not path.is_dir():
        raise ValueError(f"Bundle path must be a directory: {path}")

    path.mkdir(parents=True, exist_ok=True)
    has_contents = any(path.iterdir())
    if not has_contents:
        return

    if not force:
        raise RuntimeError(
            f"Bundle directory '{path}' is not empty. Set force=True to clean it."
        )

    for item in path.iterdir():
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()


def _rewrite_bundle_provenance_paths(
    root_yaml: Path,
    bundle_dir: Path,
    model_documents: Optional[dict[str, Model]] = None,
) -> None:
    """Rewrite provenance file references to point at copied bundle assets."""

    context = _BundleCopyContext(
        bundle_dir=bundle_dir.resolve(),
        used_filenames={
            entry.name for entry in bundle_dir.iterdir() if entry.is_file()
        },
    )

    loaded_docs = _load_bundle_documents(root_yaml)
    for yaml_path, document in loaded_docs.items():
        source_model = None
        if model_documents is not None:
            source_model = model_documents.get(yaml_path.name)
        changed = _copy_referenced_files_in_provenance(
            document,
            context,
            source_model,
        )
        if changed:
            with yaml_path.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(document, handle, sort_keys=False, allow_unicode=False)


def _load_bundle_documents(root_yaml: Path) -> dict[Path, dict[str, Any]]:
    """Load all YAML documents reachable from the root model reference graph."""

    docs: dict[Path, dict[str, Any]] = {}
    queue = [root_yaml.resolve()]
    seen: set[Path] = set()

    while queue:
        current = queue.pop()
        if current in seen:
            continue
        seen.add(current)

        with current.open("r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle)
        if not isinstance(document, dict):
            continue

        docs[current] = document

        for ref in _iter_model_references(document):
            queue.append((current.parent / ref).resolve())

    return docs


def _iter_model_references(value: Any):
    """Yield nested model reference filenames from a serialized value tree."""

    if (
        isinstance(value, Mapping)
        and set(value.keys()) == {_MODEL_REFERENCE_KEY}
        and isinstance(value[_MODEL_REFERENCE_KEY], str)
    ):
        yield value[_MODEL_REFERENCE_KEY]
        return

    if isinstance(value, Mapping):
        for item in value.values():
            yield from _iter_model_references(item)
        return

    if isinstance(value, list):
        for item in value:
            yield from _iter_model_references(item)


def _copy_referenced_files_in_provenance(
    document: dict[str, Any],
    context: _BundleCopyContext,
    source_model: Optional[Model] = None,
) -> bool:
    """Copy provenance file references into the bundle and rewrite paths in place."""

    provenance = document.get("provenance")
    if not isinstance(provenance, Mapping):
        return False

    changed = False

    constructor = provenance.get("constructor")
    if isinstance(constructor, Mapping):
        method = constructor.get("method")
        kwargs = constructor.get("kwargs")
        if isinstance(kwargs, Mapping):
            if method == "from_obj":
                _register_obj_material_sources(kwargs, source_model, context)
            changed |= _rewrite_provenance_value(kwargs, context)

    operations = provenance.get("operations")
    if isinstance(operations, list):
        for operation in operations:
            if not isinstance(operation, Mapping):
                continue
            kwargs = operation.get("kwargs")
            if isinstance(kwargs, Mapping):
                changed |= _rewrite_provenance_value(kwargs, context)

    return changed


def _register_obj_material_sources(
    kwargs: Mapping[str, Any],
    source_model: Optional[Model],
    context: _BundleCopyContext,
) -> None:
    """Record OBJ-linked material files that must keep their original names."""

    if source_model is None:
        return

    obj_reference = kwargs.get("obj_file")
    if not isinstance(obj_reference, str) or not obj_reference:
        return

    obj_source = _resolve_reference_source(Path(obj_reference), context.bundle_dir)
    if obj_source is None:
        return

    material_sources = _collect_scene_part_material_sources(
        source_model,
        obj_source.parent,
        context.bundle_dir,
    )
    if material_sources:
        context.obj_material_sources[obj_source.resolve()] = material_sources


def _collect_scene_part_material_sources(
    source_model: Model,
    obj_dir: Path,
    bundle_dir: Path,
) -> list[Path]:
    """Collect MTL sources referenced by scene-part materials loaded from OBJ."""

    materials = getattr(source_model, "materials", None)
    if materials is None or not hasattr(materials, "values"):
        return []

    sources: list[Path] = []
    seen: set[Path] = set()
    for material in materials.values():
        cpp_material = getattr(material, "_cpp_object", None)
        mat_file_path = getattr(cpp_material, "mat_file_path", None)
        if not isinstance(mat_file_path, str) or not mat_file_path:
            continue

        source = _resolve_dependency_reference(mat_file_path, obj_dir, bundle_dir)
        if source is None or source.suffix.lower() != ".mtl":
            continue

        source = source.resolve()
        if source in seen:
            continue
        seen.add(source)
        sources.append(source)

    return sources


def _rewrite_provenance_value(value: Any, context: _BundleCopyContext) -> bool:
    """Rewrite nested provenance strings to bundled filenames where possible."""

    changed = False

    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(item, str):
                rewritten = _copy_file_reference_to_bundle(item, context)
                if rewritten != item:
                    value[key] = rewritten
                    changed = True
                continue
            changed |= _rewrite_provenance_value(item, context)
        return changed

    if isinstance(value, list):
        for index, item in enumerate(value):
            if isinstance(item, str):
                rewritten = _copy_file_reference_to_bundle(item, context)
                if rewritten != item:
                    value[index] = rewritten
                    changed = True
                continue
            changed |= _rewrite_provenance_value(item, context)
        return changed

    return False


def _copy_file_reference_to_bundle(value: str, context: _BundleCopyContext) -> str:
    """Copy one file reference into the bundle and return its bundled filename."""

    if not value:
        return value

    reference = Path(value).expanduser()
    if reference.suffix == "":
        return value

    source = _resolve_reference_source(reference, context.bundle_dir)
    if source is None:
        return value

    if source.is_relative_to(context.bundle_dir):
        source_name = source.name
        context.source_to_target.setdefault(source, source_name)
        context.used_filenames.add(source_name)
        return source_name

    if source in context.source_to_target:
        return context.source_to_target[source]

    target_name = _reserve_bundle_filename(source, context)
    _copy_asset_with_dependencies(
        source=source,
        target_name=target_name,
        context=context,
        copied_sources=set(),
    )
    return target_name


def _copy_asset_with_dependencies(
    source: Path,
    target_name: str,
    context: _BundleCopyContext,
    copied_sources: set[Path],
) -> None:
    """Copy an asset into the bundle, recursively including known dependencies."""

    source = source.resolve()
    if source in copied_sources:
        return
    copied_sources.add(source)

    target_name = _reserve_bundle_filename(source, context, preferred=target_name)
    target_path = context.bundle_dir / target_name

    suffix = source.suffix.lower()
    if suffix == ".obj":
        _copy_obj_with_dependencies(source, target_path, context, copied_sources)
        return

    if suffix == ".mtl":
        _copy_mtl_with_dependencies(source, target_path, context, copied_sources)
        return

    target_path.parent.mkdir(parents=True, exist_ok=True)
    if source != target_path.resolve():
        shutil.copy2(source, target_path)


def _resolve_reference_source(reference: Path, bundle_dir: Path) -> Optional[Path]:
    """Resolve a referenced file from absolute paths, bundle paths, or assets."""

    source: Optional[Path] = None
    if reference.is_absolute():
        if reference.exists() and reference.is_file():
            source = reference.resolve()
    else:
        bundle_candidate = (bundle_dir / reference).resolve()
        if bundle_candidate.exists() and bundle_candidate.is_file():
            source = bundle_candidate
        else:
            found = find_file(reference, fatal=False)
            if found is not None and found.is_file():
                source = found.resolve()
    return source


def _reserve_bundle_filename(
    source: Path,
    context: _BundleCopyContext,
    preferred: Optional[str] = None,
) -> str:
    """Reserve and memoize a collision-free filename for a bundled source file."""

    source = source.resolve()
    existing = context.source_to_target.get(source)
    if existing is not None:
        return existing

    base_name = Path(preferred if preferred is not None else source.name).name
    base_name = _sanitize_bundle_filename(base_name)
    reserved = _reserve_unique_filename(base_name, context.used_filenames)
    context.source_to_target[source] = reserved
    return reserved


def _sanitize_bundle_filename(filename: str) -> str:
    """Normalize a filename for safe bundle-local usage."""

    candidate = filename.replace("\\", "_").replace("/", "_")
    candidate = "_".join(candidate.split())
    return candidate or "asset"


def _resolve_dependency_reference(
    reference: str, base_dir: Path, bundle_dir: Path
) -> Optional[Path]:
    """Resolve dependent assets relative to source, bundle, or asset directories."""

    ref_path = Path(reference).expanduser()
    if ref_path.is_absolute():
        if ref_path.exists() and ref_path.is_file():
            return ref_path.resolve()
        return None

    source_candidate = (base_dir / ref_path).resolve()
    if source_candidate.exists() and source_candidate.is_file():
        return source_candidate

    bundle_candidate = (bundle_dir / ref_path).resolve()
    if bundle_candidate.exists() and bundle_candidate.is_file():
        return bundle_candidate

    found = find_file(ref_path, fatal=False)
    if found is not None and found.is_file():
        return found.resolve()
    return None


def _copy_obj_with_dependencies(
    source: Path,
    target: Path,
    context: _BundleCopyContext,
    copied_sources: set[Path],
) -> None:
    """Copy an OBJ and required material dependencies into the bundle."""

    target.parent.mkdir(parents=True, exist_ok=True)
    if source != target.resolve():
        shutil.copy2(source, target)

    material_sources = context.obj_material_sources.get(source.resolve(), [])
    if not material_sources:
        return

    for material_source in material_sources:
        dep_name = _reserve_bundle_filename(
            material_source,
            context,
            preferred=material_source.name,
        )
        if dep_name != material_source.name:
            raise RuntimeError(
                "Cannot keep OBJ file unchanged while bundling materials: "
                f"filename conflict for '{material_source.name}'."
            )
        _copy_asset_with_dependencies(
            material_source, dep_name, context, copied_sources
        )


def _copy_mtl_with_dependencies(
    source: Path,
    target: Path,
    context: _BundleCopyContext,
    copied_sources: set[Path],
) -> None:
    """Copy an MTL file while rewriting texture references to bundled assets."""

    content = source.read_text(encoding="utf-8", errors="ignore")
    lines = content.splitlines()
    trailing_newline = content.endswith("\n")
    rewritten = [
        _rewrite_mtl_line(line, source.parent, context, copied_sources)
        for line in lines
    ]
    _write_text_asset(target, rewritten, trailing_newline)


def _rewrite_mtl_line(
    line: str,
    base_dir: Path,
    context: _BundleCopyContext,
    copied_sources: set[Path],
) -> str:
    """Rewrite one MTL directive line so copied textures use bundle-local names."""

    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return line

    parts = stripped.split(maxsplit=1)
    if len(parts) != 2:
        return line

    directive = parts[0]
    if directive.lower() not in _mtl_texture_directives():
        return line

    try:
        tokens = shlex.split(parts[1])
    except ValueError:
        return line

    if not tokens:
        return line

    dep_source = _resolve_dependency_reference(tokens[-1], base_dir, context.bundle_dir)
    if dep_source is None:
        return line

    dep_name = _reserve_bundle_filename(dep_source, context)
    _copy_asset_with_dependencies(dep_source, dep_name, context, copied_sources)
    tokens[-1] = dep_name
    return f"{directive} {' '.join(tokens)}"


def _write_text_asset(target: Path, lines: list[str], trailing_newline: bool) -> None:
    """Write plain-text asset lines, preserving trailing newline semantics."""

    target.parent.mkdir(parents=True, exist_ok=True)
    content = "\n".join(lines)
    if trailing_newline:
        content += "\n"
    target.write_text(content, encoding="utf-8")


def _mtl_texture_directives() -> set[str]:
    """Return lowercase MTL directives that point to texture map files."""

    texture_directives = {
        "map_ka",
        "map_kd",
        "map_ks",
        "map_ke",
        "map_ns",
        "map_d",
        "map_bump",
        "bump",
        "disp",
        "decal",
        "refl",
        "norm",
    }
    return texture_directives


def _build_model_document(
    model: Model, context: _SerializationContext
) -> dict[str, Any]:
    """Create the serialized document payload for one model."""

    binary_info = None
    include_fields = not _uses_from_binary_constructor_provenance(model)
    if context.binary and callable(getattr(model, "to_binary", None)):
        model_id = id(model)
        if model_id in context.binary_files:
            binary_filename = context.binary_files[model_id]
        else:
            preferred = model._serialization_binary_filename()
            preferred = _ensure_suffix(Path(preferred).name, ".bin")
            binary_filename = _reserve_unique_filename(
                preferred, context.used_binary_filenames
            )
            model.to_binary(context.root_dir / binary_filename)
            context.binary_files[model_id] = binary_filename
        binary_info = {
            "method": "from_binary",
            "path": binary_filename,
        }
        include_fields = False

    fields: dict[str, Any] = {}
    if include_fields:
        for field_name, annotation in get_all_annotations(model.__class__).items():
            value = getattr(model, field_name)
            if _contains_model_instance(value):
                fields[field_name] = _serialize_runtime_value(value, context)
            else:
                fields[field_name] = TypeAdapter(annotation).dump_python(
                    value, mode="json"
                )

    document: dict[str, Any] = {
        "serialization_major_version": SERIALIZATION_FORMAT_MAJOR_VERSION,
        "serialization_minor_version": SERIALIZATION_FORMAT_MINOR_VERSION,
        "model_class": _class_identifier(model.__class__),
        "fields": fields,
    }

    if hasattr(model, "_provenance"):
        document["provenance"] = _normalize_provenance_value(model._provenance)

    if binary_info is not None:
        document["binary"] = binary_info

    return document


def _uses_from_binary_constructor_provenance(model: Model) -> bool:
    """Return whether model provenance already records a ``from_binary`` constructor."""

    provenance = getattr(model, "_provenance", None)
    if not isinstance(provenance, Mapping):
        return False

    constructor = provenance.get("constructor")
    if not isinstance(constructor, Mapping):
        return False

    return constructor.get("method") == "from_binary"


def _serialize_runtime_value(value: Any, context: _SerializationContext):
    """Serialize arbitrary runtime values recursively."""

    if isinstance(value, Model):
        if context.shallow:
            model_id = id(value)
            if model_id in context.model_files:
                filename = context.model_files[model_id]
            else:
                preferred = value._serialization_filename()
                preferred = _ensure_suffix(Path(preferred).name, ".helios.yaml")
                filename = _reserve_unique_filename(
                    preferred, context.used_yaml_filenames
                )
                _write_model_document(value, filename, context)
            return {_MODEL_REFERENCE_KEY: filename}

        value_id = id(value)
        if value_id in context.inline_stack:
            raise ValueError(
                "Detected cyclic model graph. Use shallow=True for this object."
            )

        context.inline_stack.add(value_id)
        try:
            return _build_model_document(value, context)
        finally:
            context.inline_stack.remove(value_id)

    if isinstance(value, Mapping):
        return {
            str(key): _serialize_runtime_value(item, context)
            for key, item in value.items()
        }

    if is_real_iterable(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        return [_serialize_runtime_value(item, context) for item in value]

    return _normalize_provenance_value(value)


def _contains_model_instance(value: Any) -> bool:
    """Return whether a value tree contains any Model instances."""

    if isinstance(value, Model):
        return True

    if isinstance(value, Mapping):
        return any(_contains_model_instance(item) for item in value.values())

    if is_real_iterable(value):
        if isinstance(value, np.ndarray):
            return False
        return any(_contains_model_instance(item) for item in value)

    return False


def _contains_serialized_model(value: Any) -> bool:
    """Return whether a serialized value tree contains model documents/references."""

    if (
        isinstance(value, Mapping)
        and set(value.keys()) == {_MODEL_REFERENCE_KEY}
        and isinstance(value[_MODEL_REFERENCE_KEY], str)
    ):
        return True

    if (
        isinstance(value, Mapping)
        and "serialization_major_version" in value
        and "serialization_minor_version" in value
        and "model_class" in value
        and "fields" in value
    ):
        return True

    if isinstance(value, Mapping):
        return any(_contains_serialized_model(item) for item in value.values())

    if isinstance(value, list):
        return any(_contains_serialized_model(item) for item in value)

    return False


def _reserve_unique_filename(preferred: str, used: set[str]) -> str:
    """Reserve a unique filename within the given used-name set."""

    candidate = Path(preferred).name
    stem = Path(candidate).stem
    suffix = Path(candidate).suffix

    if candidate not in used:
        used.add(candidate)
        return candidate

    index = 2
    while True:
        candidate = f"{stem}_{index}{suffix}"
        if candidate not in used:
            used.add(candidate)
            return candidate
        index += 1


def _ensure_suffix(filename: str, suffix: str) -> str:
    """Append a suffix if the filename has none."""

    if Path(filename).suffix:
        return filename
    return f"{filename}{suffix}"


def _load_model_from_file(
    path: Path,
    context: _DeserializationContext,
    expected_cls: Optional[Type[Model]] = None,
) -> Model:
    """Load and cache a model document from disk."""

    resolved = path.resolve()
    if resolved in context.file_cache:
        return context.file_cache[resolved]

    if resolved in context.loading_files:
        raise ValueError(f"Cyclic model reference detected while loading {resolved}.")

    context.loading_files.add(resolved)
    try:
        with path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle)
        if not isinstance(loaded, dict):
            raise ValueError(
                f"Expected YAML document object at {path}, got {type(loaded).__name__}."
            )

        _validate_document_schema(loaded, path)
        document = _apply_migrations(loaded, path)
        model = _deserialize_model_document(
            document=document,
            expected_cls=expected_cls,
            source_dir=resolved.parent,
            context=context,
        )
        context.file_cache[resolved] = model
        return model
    finally:
        context.loading_files.remove(resolved)


def _validate_document_schema(document: dict[str, Any], path: Optional[Path]) -> None:
    """Validate a serialized document against the JSON schema."""

    global _MODEL_DOCUMENT_VALIDATOR
    if _MODEL_DOCUMENT_VALIDATOR is None:
        schema_resource = (
            resources.files("helios") / "data" / "serialization_schema.json"
        )
        with schema_resource.open("r", encoding="utf-8") as handle:
            schema = json.load(handle)
        _MODEL_DOCUMENT_VALIDATOR = Draft202012Validator(schema)

    errors = sorted(
        _MODEL_DOCUMENT_VALIDATOR.iter_errors(document),
        key=lambda error: error.path,
    )
    if not errors:
        return

    details = "; ".join(error.message for error in errors)
    location = str(path) if path is not None else "<inline>"
    raise ValueError(f"Invalid YAML serialization format in {location}: {details}")


def _apply_migrations(document: dict[str, Any], path: Optional[Path]) -> dict[str, Any]:
    """Migrate a document forward to the current format minor version."""

    major_version = document.get("serialization_major_version")
    if not isinstance(major_version, int):
        location = str(path) if path is not None else "<inline>"
        raise ValueError(
            "Invalid serialization_major_version " f"in {location}: {major_version!r}"
        )

    minor_version = document.get("serialization_minor_version")
    if not isinstance(minor_version, int):
        location = str(path) if path is not None else "<inline>"
        raise ValueError(
            "Invalid serialization_minor_version " f"in {location}: {minor_version!r}"
        )

    if major_version != SERIALIZATION_FORMAT_MAJOR_VERSION:
        location = str(path) if path is not None else "<inline>"
        raise ValueError(
            f"Unsupported serialization major version {major_version} in {location}. "
            f"Current major version is {SERIALIZATION_FORMAT_MAJOR_VERSION}."
        )

    if minor_version > SERIALIZATION_FORMAT_MINOR_VERSION:
        location = str(path) if path is not None else "<inline>"
        raise ValueError(
            f"Unsupported serialization minor version {minor_version} in {location}. "
            f"Current minor version is {SERIALIZATION_FORMAT_MINOR_VERSION}."
        )

    migrated = document
    while minor_version < SERIALIZATION_FORMAT_MINOR_VERSION:
        migration = _MIGRATIONS.get(minor_version)
        if migration is None:
            location = str(path) if path is not None else "<inline>"
            raise ValueError(
                f"No migration available from minor version {minor_version} "
                f"to {minor_version + 1} "
                f"for {location}."
            )
        migrated = migration(migrated)
        next_major_version = migrated.get("serialization_major_version")
        next_minor_version = migrated.get("serialization_minor_version")
        if (
            not isinstance(next_major_version, int)
            or next_major_version != SERIALIZATION_FORMAT_MAJOR_VERSION
        ):
            raise ValueError(
                "Invalid migration result: expected "
                f"serialization_major_version={SERIALIZATION_FORMAT_MAJOR_VERSION}, "
                f"got {next_major_version!r}."
            )
        if (
            not isinstance(next_minor_version, int)
            or next_minor_version <= minor_version
        ):
            raise ValueError(
                "Invalid migration result: expected "
                f"serialization_minor_version > {minor_version}, "
                f"got {next_minor_version!r}."
            )
        minor_version = next_minor_version

    _validate_document_schema(migrated, path)
    return migrated


def _deserialize_model_document(
    document: dict[str, Any],
    expected_cls: Optional[Type[Model]],
    source_dir: Path,
    context: _DeserializationContext,
) -> Model:
    """Reconstruct a model object from one serialized document."""

    class_identifier = document["model_class"]
    if expected_cls is not None and class_identifier == _class_identifier(expected_cls):
        model_cls = expected_cls
    else:
        module_name, _, class_name = class_identifier.rpartition(".")
        if not module_name:
            raise ValueError(f"Invalid model class identifier: {class_identifier!r}")

        module = import_module(module_name)
        model_cls = module
        for part in class_name.split("."):
            model_cls = getattr(model_cls, part, None)
            if model_cls is None:
                raise ValueError(f"Cannot resolve model class '{class_identifier}'.")

    if not isinstance(model_cls, type) or not issubclass(model_cls, Model):
        raise TypeError(f"Resolved class {model_cls!r} is not a Model subclass.")

    if expected_cls is not None and not issubclass(model_cls, expected_cls):
        raise TypeError(
            f"Serialized model type {model_cls.__name__} is not compatible with {expected_cls.__name__}."
        )

    model_obj: Optional[Model] = None
    if "binary" in document:
        binary_info = document["binary"]
        method_name = binary_info["method"]
        binary_path = (source_dir / binary_info["path"]).resolve()

        if method_name == "from_binary":
            method = getattr(model_cls, "from_binary", None)
            if method is None or not callable(method):
                raise ValueError(
                    f"Binary method 'from_binary' not found on {model_cls.__name__}."
                )
            model_obj = method(binary_path)
        else:
            method = getattr(model_cls, method_name, None)
            if method is None or not callable(method):
                raise ValueError(
                    f"Binary method '{method_name}' not found on {model_cls.__name__}."
                )
            model_obj = method(binary_path)
    elif "provenance" in document:
        provenance = document["provenance"]
        constructor_info = provenance.get("constructor")
        operations = provenance.get("operations", [])

        if constructor_info is not None:
            method_name = constructor_info["method"]
            kwargs = _expand_serialized_value(
                constructor_info["kwargs"], source_dir, context
            )

            constructor = getattr(model_cls, method_name, None)
            if constructor is None or not callable(constructor):
                raise ValueError(
                    f"Provenance constructor '{method_name}' not found on {model_cls.__name__}."
                )
            model_obj = constructor(**kwargs)

        if model_obj is None:
            if operations:
                raise ValueError(
                    f"Cannot replay provenance operations for {model_cls.__name__} without constructor."
                )
        else:
            for operation in operations:
                method_name = operation["method"]
                kwargs = _expand_serialized_value(
                    operation["kwargs"], source_dir, context
                )
                method = getattr(model_obj, method_name, None)
                if method is None or not callable(method):
                    raise ValueError(
                        f"Provenance operation '{method_name}' not found on {model_cls.__name__}."
                    )
                method(**kwargs)

    parsed_fields: dict[str, Any] = {}
    for field_name, annotation in get_all_annotations(model_cls).items():
        if field_name not in document["fields"]:
            continue

        serialized_value = document["fields"][field_name]
        if model_obj is not None and _contains_serialized_model(serialized_value):
            # If we already reconstructed via provenance, keep constructor-provided
            # model graph for model-typed fields.
            continue

        expanded_value = _expand_serialized_value(
            serialized_value, source_dir=source_dir, context=context
        )
        parsed_fields[field_name] = TypeAdapter(annotation).validate_python(
            expanded_value
        )

    if model_obj is None:
        model_obj = model_cls(**parsed_fields)
    else:
        for field_name, value in parsed_fields.items():
            if _contains_model_instance(value):
                continue
            try:
                setattr(model_obj, field_name, value)
            except RuntimeError:
                # XML-backed objects can have immutable fields; keep constructor result.
                continue

    if "provenance" in document:
        model_obj._provenance = _normalize_provenance_value(document["provenance"])

    return model_obj


def _expand_serialized_value(
    value: Any, source_dir: Path, context: _DeserializationContext
) -> Any:
    """Expand references and inline docs recursively into runtime values."""

    if (
        isinstance(value, Mapping)
        and set(value.keys()) == {_MODEL_REFERENCE_KEY}
        and isinstance(value[_MODEL_REFERENCE_KEY], str)
    ):
        reference = value[_MODEL_REFERENCE_KEY]
        reference_path = (source_dir / reference).resolve()
        return _load_model_from_file(reference_path, context)

    if (
        isinstance(value, Mapping)
        and "serialization_major_version" in value
        and "serialization_minor_version" in value
        and "model_class" in value
        and "fields" in value
    ):
        _validate_document_schema(value, None)
        migrated = _apply_migrations(value, None)
        return _deserialize_model_document(
            document=migrated,
            expected_cls=None,
            source_dir=source_dir,
            context=context,
        )

    if isinstance(value, list):
        return [_expand_serialized_value(item, source_dir, context) for item in value]

    if isinstance(value, Mapping):
        return {
            key: _expand_serialized_value(item, source_dir, context)
            for key, item in value.items()
        }

    return value


def _class_identifier(cls: Type[Model]) -> str:
    """Return the fully qualified identifier for a model class."""

    return f"{cls.__module__}.{cls.__qualname__}"
