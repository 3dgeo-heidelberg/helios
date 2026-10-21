# Pluggable `Model` constructors

## Objective

Allow Helios and downstream distributions to add class-bound `from_*` constructors to specific `Model` subclasses through Python entry points. Constructor availability must be discovered early enough that the method is present in the class dictionary, while importing the provider implementation must remain lazy to avoid circular imports during class and package initialization.

All runtime infrastructure for discovery, provider loading, descriptor installation, and invocation belongs in `python/helios/validation.py`. Serialization should continue using ordinary `getattr(model_cls, method_name)` and should not gain a separate plugin system.

## Public plugin contract

### Entry-point group and names

Define one entry-point group:

```text
helios.model_constructors.v1
```

The `.v1` suffix is the constructor-provider API version. Additive, backward-compatible changes remain in this group. A future breaking provider API will use a new group such as `helios.model_constructors.v2`; the entry-point group is the sole API-version declaration, so provider classes do not also declare a version.

Each entry-point name identifies one scope-root class and one constructor:

```text
<fully-qualified-model-class>:<constructor-name>
```

For example, a downstream `pyproject.toml` would contain:

```toml
[project.entry-points."helios.model_constructors.v1"]
"helios.scene.ScenePart:from_doi" = "my_project.constructors:ScenePartFromDoi"
"helios.scene.ScenePart:from_repository" = "my_project.constructors:ScenePartFromRepository"
```

The target is an inheritance scope root, not an exact-only applicability rule. The descriptor is installed once on the named class and is inherited normally by its subclasses. Consequently, a constructor intended for every model can target `Model`, while one intended for a family of models can target their common model base:

```toml
[project.entry-points."helios.model_constructors.v1"]
"helios.validation.Model:from_msgpack" = "helios_msgpack.constructors:FromMsgpack"
"helios.scene._Scene:from_scene_archive" = "scene_archive.constructors:FromSceneArchive"
```

`Survey.from_msgpack(...)` receives `Survey` as `model_cls`, even though the descriptor was registered and installed on `Model`. Likewise, `StaticScene.from_scene_archive(...)` receives `StaticScene`. Providers are responsible for returning an object appropriate for the supplied class; the infrastructure does not validate the result.

Encoding the method name in entry-point metadata is essential: Helios can attach the descriptor without calling `EntryPoint.load()` and therefore without importing the downstream project during target-class construction. It mirrors the provider's `name()` result because the provider remains unloaded during discovery.

Entry-point names follow the documented `<fully-qualified-model-class>:<from_method>` convention. The infrastructure does not validate this metadata; ordinary parsing and attribute errors propagate.

### Provider base class

Add a small public `ConstructorProvider` base class to `validation.py`. Each provider represents one constructor and implements two static methods: `name()` returns its public `from_*` name, and `construct(model_cls, ...)` implements it. The base implementations raise `NotImplementedError`. Provider classes are never inserted into a `Model` class's MRO.

The base class defines the provider interface and is a stable extension point for future provider-wide utilities. Constructor resolution remains in the lazy descriptor. The initial infrastructure does not validate inheritance, compare `name()` with the entry-point name, or inspect the method declarations; ordinary attribute and call errors propagate.

The initial base class should remain deliberately small. In particular, it must not import target model classes, scan/install itself in `__init_subclass__`, or maintain a second global registration mechanism.

Example provider:

```python
from typing import TYPE_CHECKING

from helios.validation import ConstructorProvider

if TYPE_CHECKING:
    from helios.scene import ScenePart


class ScenePartFromDoi(ConstructorProvider):
    @staticmethod
    def name() -> str:
        return "from_doi"

    @staticmethod
    def construct(model_cls: type["ScenePart"], doi: str) -> "ScenePart":
        path = download_doi(doi)
        result = model_cls.from_obj(path)
        result._set_constructor_provenance("from_doi", doi=doi)
        return result
```

Provider modules may use target classes under `TYPE_CHECKING`, but must not need to import the concrete target at runtime. The actual target class is supplied as `model_cls` when the implementation is invoked.

## Infrastructure in `validation.py`

### Entry-point metadata index

Add an internal, process-local index populated with `importlib.metadata.entry_points(group="helios.model_constructors.v1")`.

The index should:

- Be initialized lazily while `Model` itself is constructed, so universal constructors can be installed on `Model`; initialization still reads metadata only and never imports provider modules.
- Cache entry-point metadata for the remainder of the process.
- Never call `EntryPoint.load()` while building the index.
- Map `(target_class_identifier, constructor_name)` to an entry point. If metadata contains duplicates, the later record replaces the earlier record.
- Cache the index with `functools.lru_cache`; tests clear that cache through an autouse pytest fixture rather than application-level reset infrastructure.

Installing a new distribution into a running Python process is out of scope. A process restart is required for newly installed entry points to appear.

The initial implementation scans only the `.v1` group because no older API exists. If a future breaking change introduces `.v2`, that implementation should use `.v2` for active providers and additionally inspect known older groups to report installed outdated plugins clearly rather than making their constructors appear inexplicably absent. No legacy-group discovery machinery is needed for v1.

### Lazy constructor descriptor

Implement a private descriptor, for example `_LazyConstructorDescriptor`, containing:

- The constructor name.
- The target identifier.
- The matching entry-point record.
- A cached loaded implementation.

Descriptor behavior:

1. Access loads the provider on first use and resolves its `construct` implementation.
2. Cache the resolved implementation and return a normal callable bound to the class used for lookup.
3. Copy the implementation docstring onto the bound callable.
4. Invoke it as `implementation(model_cls, *args, **kwargs)` and return its result unchanged.

The bound callable exposes the public constructor `__name__` and `__qualname__` and copies `__doc__` from `construct`. Binding the callable loads the provider; target-class construction remains import-free.

### Error handling

The initial implementation adds no plugin-specific error checking or translation. Errors from entry-point parsing, provider loading, attribute lookup, and provider invocation propagate unchanged. Validation and improved diagnostics can be introduced incrementally when concrete needs are established.

### Metaclass integration

Extend `ValidatedModelMetaClass.__new__` so constructor descriptors are part of the class from creation time:

1. Apply constructor discovery to every class created by `ValidatedModelMetaClass`, including `Model` itself. Definitions needed for indexing and lazy descriptors must therefore appear before the `Model` declaration in `validation.py`.
2. Form the target identifier from `dct["__module__"]` and `dct["__qualname__"]`; use the same `module.qualname` convention as serialization's class identifier.
3. Query the cached entry-point metadata index for registrations whose scope root is that exact class. Install only those descriptors directly; registrations on base classes propagate through normal inheritance.
4. Add matching lazy descriptors to the class namespace before passing it to `super().__new__`. This lets Python perform normal descriptor setup and makes the methods visible in the resulting class dictionary.
5. Continue with the existing annotation/default/property and generated-`__init__` processing unchanged.

Constructor descriptors are not annotated fields and must never be included in `get_all_annotations()` or `_defaults`.

Apply these collision rules:

- A method explicitly defined by the target class wins over an entry-point constructor.
- A descriptor registered for an exact subclass or intermediate model base replaces an inherited constructor. This permits narrower scopes to specialize a constructor inherited from `Model` or another model base, while subclasses without their own registration continue to inherit the broader provider.
- Duplicate metadata records use the entry point encountered last during indexing.

A descriptor inherited from a base model must bind to the class through which it was accessed. Thus `SpecialScenePart.from_doi(...)` receives `SpecialScenePart` as `model_cls` even when the entry point was registered for `ScenePart`.

This produces the following precedence, from strongest to weakest:

1. A constructor explicitly implemented by the concrete class.
2. A plugin descriptor registered directly for the concrete class.
3. A constructor or plugin descriptor inherited from the nearest intermediate base.
4. A universal plugin descriptor registered on `Model`.

## Constructor composition

Provider implementations call other constructors through `model_cls`:

```python
class ScenePartFromDoi(ConstructorProvider):
    @staticmethod
    def name():
        return "from_doi"

    @staticmethod
    def construct(model_cls, doi):
        path = download_doi(doi)
        result = model_cls.from_obj(path)
        result._set_constructor_provenance("from_doi", doi=doi)
        return result
```

This preserves subclass dispatch, built-in constructor behavior, and lazy loading of other plugin constructors. Calling another provider class directly is discouraged because it bypasses target binding and overrides.

If one plugin requires a constructor supplied by another distribution, that distribution must be declared as a package dependency. A missing optional constructor produces Python's normal attribute error at the delegation point.

Circular constructor calls such as `from_a -> from_b -> from_a` are programming errors. Explicit cycle detection is not required initially; normal Python recursion behavior is sufficient. It can be added later with a `ContextVar` call stack if diagnostics prove inadequate.

## Provenance and serialization

Do not make the descriptor automatically create provenance. Existing constructors have method-specific decisions about which arguments are stable and serializable, and some constructors delegate to lower-level constructors.

Establish this provider rule:

- The outer public constructor owns final constructor provenance.
- When it delegates, it sets its provenance after the delegated constructor returns, replacing the inner constructor's provenance when appropriate.

No constructor-plugin changes should be required in `serialization.py`. Deserialization already resolves provenance constructors through `getattr(model_cls, method_name)`. Because lazy descriptors participate in ordinary attribute lookup, replay will load the required provider automatically.

Serialized data continues to store the constructor method and arguments, not executable plugin code. Consequently, replay requires a compatible provider distribution to be installed. Recording provider distribution/version metadata in provenance is a possible future format change, not part of the initial implementation.

## Built-in constructors and packaging

Keep existing Helios constructors implemented directly on their model classes in the first iteration. A constructor defined directly on a class takes precedence over a registration for that same class; a narrower subclass registration may replace an inherited constructor.

Use the new entry-point system initially for new optional or downstream constructors. Migrating existing constructors into providers can be considered separately after the infrastructure is stable; it is not required to validate the design.

Add the entry-point group and provider authoring example to the developer documentation. Document that:

- Entry points are discovered from installed distribution metadata.
- Provider implementation import is deferred until constructor attribute access.
- Providers must inherit `ConstructorProvider`.
- Each provider implements static `name()` and `construct()` methods, with `construct()` accepting `model_cls` first.
- Target imports should be guarded by `TYPE_CHECKING`.
- Runtime method injection is not visible to static type checkers unless the plugin ships appropriate stubs.

## Test plan

Add focused tests to `tests/python/test_validation.py`. Tests should protect observable behavior and plugin-author contracts, not mirror every architectural statement or internal data structure. Use fake entry-point objects so the suite does not require building/installing a separate wheel, and isolate them with an autouse fixture that calls `cache_clear()` on the cached index function.

Avoid assertions about the concrete descriptor class, cache representation, or the exact point inside metaclass execution at which installation occurs. Assert through normal class lookup, invocation, and returned values instead. Acceptance criteria and design guardrails do not imply a one-to-one set of unit tests.

### Discovery and installation

- A matching entry point installs the constructor descriptor in the class dictionary without calling `EntryPoint.load()`.

### Lazy loading

- Accessing the constructor loads the provider exactly once and subsequent accesses reuse it.

### Resolution and inheritance

- Use a compact precedence matrix covering universal registration on `Model`, family registration on an intermediate base, exact-class registration, explicitly defined constructors, and replacement of inherited constructors.
- Verify that an inherited constructor receives the subclass through which it was called.

### Composition and provenance

- One composition test verifies that a lazy constructor can invoke another constructor through `model_cls`, including lazy loading when the delegated constructor is also plugin-provided.
- The outer constructor can replace delegated provenance so the recorded method is the user-facing constructor.

### Serialization integration

- Serialize a model whose provenance names a plugin constructor, then deserialize it and verify ordinary `getattr` triggers lazy loading and reconstruction.

Subclass binding is already covered by focused tests and does not need to be repeated through serialization unless implementation work reveals that replay bypasses the normal descriptor path.

## Design and review guardrails

The following constraints should be checked during implementation and review rather than receiving dedicated structural unit tests:

- All constructor-plugin infrastructure remains in `validation.py`.
- Provider classes are never added to a `Model` MRO, so provider annotations cannot become model fields.
- Class construction reads entry-point metadata but does not import provider modules.
- Serialization continues to use normal attribute lookup and does not introduce a second constructor-plugin registry.
- API compatibility is represented by the versioned entry-point group, currently `helios.model_constructors.v1`, rather than a provider class version field.
- Global applicability uses registration on `Model`; no wildcard registration mechanism is introduced.
- Existing built-in constructors remain implemented directly on their model classes for this iteration.
- Newly installed entry points require a process restart; hot reloading is not implemented.
- Static type-checker discovery of injected constructors remains outside the runtime infrastructure.

## Implementation order

1. Add entry-point parsing and an `lru_cache`-backed metadata index to `validation.py`.
2. Add the two-method `ConstructorProvider` interface.
3. Add `_LazyConstructorDescriptor` and unit tests for lazy loading and caching.
4. Integrate descriptor installation into `ValidatedModelMetaClass.__new__` and add the compact precedence/inheritance matrix test.
5. Add constructor-composition and provenance tests.
6. Add a serialization replay test.
7. Document downstream packaging and provider implementation, including typing and runtime-import constraints.
8. Run the full Python test suite and formatting/lint checks used by the repository.

## Acceptance criteria

- An installed downstream distribution can make `TargetModel.from_example` available using only entry-point metadata and a `ConstructorProvider` subclass.
- A distribution can make `from_example` available to every model by targeting `helios.validation.Model`, without wildcard syntax or per-subclass registration.
- A distribution can scope a constructor to a model family by targeting its common model base, and a narrower registration can specialize an inherited plugin constructor.
- The method is present through normal class attribute lookup immediately after target-class creation.
- Neither entry-point discovery nor target-class creation imports the provider module.
- The provider is imported once, on first constructor attribute access.
- Constructors can safely delegate through `model_cls.from_*`.
- Existing constructors retain their behavior unless a narrower subclass registration replaces an inherited constructor.
- Provenance replay discovers and calls plugin constructors without a serialization-specific registry.

## Non-goals for the initial implementation

- Hot-plugging newly installed distributions into an already running process.
- Automatically bundling or installing the provider distribution with serialized output.
- Runtime modification of `Model` base classes or MROs.
- Automatic provenance inference for arbitrary constructor arguments.
- Static type-checker discovery of dynamically installed methods.
- Migrating all existing built-in `from_*` methods into provider classes.
