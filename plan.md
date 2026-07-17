# DynamicScene Python API implementation plan

## Goal

Expose XML-defined C++ dynamic scenes through the high-level Python API while
keeping the first public surface deliberately small:

```python
from helios import DynamicScene, Survey

scene = DynamicScene.from_xml("path/to/dynamic_scene.xml")
survey = Survey(scanner=scanner, platform=platform, scene=scene)
survey.add_leg(...)
points, trajectory = survey.run()
```

Loading an entire dynamic survey must also work and preserve the scene type:

```python
survey = Survey.from_xml("path/to/dynamic_survey.xml")
assert isinstance(survey.scene, DynamicScene)
survey.run()
```

The C++ simulation remains responsible for dynamic timing and stepping. Python
must pass the polymorphic scene into the existing `SurveyPlayback`; it must not
reimplement or manually drive dynamic behavior.

## Agreed public contract

- `DynamicScene` is exported as `helios.DynamicScene`.
- Its only scene-specific public constructor is
  `DynamicScene.from_xml(scene_file: AssetPath) -> DynamicScene`.
- Direct construction raises a clear `TypeError` that directs users to
  `from_xml()`.
- `DynamicScene.from_xml()` rejects XML that does not produce a C++ `DynScene`.
- `StaticScene.from_xml()` rejects XML that produces a C++ `DynScene` and tells
  the user to use `DynamicScene.from_xml()`.
- `Survey.from_xml()` dispatches automatically and exposes either a
  `StaticScene` or `DynamicScene` as appropriate.
- No high-level dynamic scene-part access or mutation is exposed. In
  particular, the first API has no `scene_parts`, `add_scene_part`, material
  editing, transformation, dynamic-step editing, `show()`, binary loading, or
  binary writing for `DynamicScene`.
- XML remains the authority for motions, `dynStep`, `dynTimeStep`, object step
  intervals, and dynamic KD-tree update intervals.
- `survey.run(live=True)` is rejected for dynamic scenes until the viewer can
  update moving geometry correctly.
- One `DynamicScene` instance cannot be owned by multiple surveys.
- Direct `DynamicScene.clone()`/`deepcopy()` is unsupported. Dynamic
  `Survey.clone()`/`deepcopy()` remains unsupported unless an independence test
  proves that the C++ dynamic graph is genuinely deep-copied.
- Serialization is provenance-only: YAML stores an empty field mapping plus a
  `from_xml` constructor provenance record. There is no dynamic scene binary
  serialization in this increment.
- Before reset support lands, a dynamic scene is one-shot. Any second
  `survey.run()` attempt using a scene whose playback was started raises a clear
  error. An interrupted or failed started playback also consumes the scene.
- The last milestone replaces the one-shot restriction with a complete,
  state-efficient C++ reset. It must not retain a second full copy of scene
  geometry.

## Existing behavior that the implementation must preserve

- `XmlSceneLoader::createSceneFromXml()` begins with a `StaticScene`, upgrades
  it to `DynScene` when it encounters `<dmotion>`, finalizes the scene, builds
  the grove, and applies dynamic scene attributes.
- `Simulation::prepareSimulation()` calls virtual `Scene::prepareSimulation()`.
- `Simulation::doSimStep()` calls virtual `Scene::doSimStep()` after the scanner
  step. Therefore a correctly typed `DynScene` already moves and updates its
  observer KD trees during ordinary survey playback.
- Dynamic moving objects use internal IDs such as `DSMO_2`. C++ strips the
  `DSMO_` prefix when generating measurement hit-object IDs. Python must not
  force those internal IDs through the integer-only `ScenePart.id` wrapper.
- XML loading assigns numeric source part IDs when they are omitted. Dynamic
  scenes do not require Python's static-scene missing-ID assignment pass.
- Direct scene loading does not know scanner wavelength, so `Survey.run()` must
  continue applying reflectances after scanner and scene have been combined.

## Work-package dependency graph

```text
WP1 C++ type/loader exposure
  -> WP2 Python DynamicScene wrapper
       -> WP3 Survey integration and one-shot lifecycle
            -> WP4 policy guards
            -> WP5 provenance-only YAML
            -> WP6 end-to-end behavior and documentation
                 -> WP7 state-efficient reset and repeatable runs
```

Each work package below is intended to be implementable in fresh context. Its
file list is the expected primary surface; avoid unrelated cleanup.

---

## WP1 — Register the C++ dynamic type and add a checked XML loader

### Objective

Make pybind preserve the runtime `DynScene` type and provide one checked helper
that can only return a dynamic scene.

### Primary files

- `python/helios/helios_python.cpp`
- `src/python/PyXMLReader.h`
- `src/python/PyXMLReader.cpp`
- Binding-focused tests in `tests/python/test_bindings.py` or
  `tests/python/test_scene.py`

### Implementation

1. Register `DynScene` after the existing `StaticScene` pybind registration:

   ```cpp
   py::class_<DynScene, StaticScene, std::shared_ptr<DynScene>>(
     m, "DynamicScene");
   ```

   Do not bind constructors, scene mutation, dynamic-object access, step
   setters, or serialization methods.

2. Add:

   ```cpp
   std::shared_ptr<DynScene>
   readDynamicSceneFromXml(std::string filePath,
                           std::vector<std::string> assetsPath,
                           bool buildKDGrove = true);
   ```

   The implementation must reuse `readSceneFromXml()`, reject a null result,
   perform `std::dynamic_pointer_cast<DynScene>`, and throw a descriptive
   `HeliosException` if the XML produced a static scene.

3. Bind it as `_helios.read_dynamic_scene_from_xml` with named arguments.

4. Keep `_helios.read_scene_from_xml` generic for existing callers. Once the
   derived pybind type is registered, a generic `shared_ptr<Scene>` containing
   a `DynScene` should arrive in Python as `_helios.DynamicScene`.

### Acceptance criteria

- A known static XML file returns `_helios.StaticScene` from the generic loader.
- A known dynamic XML file returns `_helios.DynamicScene` from both the generic
  and checked dynamic loaders.
- The checked loader rejects static XML with an actionable message.
- No public `_helios.DynamicScene()` constructor exists.
- Existing binding and static scene tests pass.

### Risk notes

- Preserve `shared_ptr` ownership and pybind inheritance exactly; do not cast a
  `DynScene` into a new `StaticScene`, which would slice dynamic state.
- Do not duplicate any XML parsing or finalization logic.

---

## WP2 — Introduce the high-level `DynamicScene` and scene type dispatch

### Objective

Add the minimal high-level class without inheriting the static mutation API,
and provide a single internal dispatch point for C++ scenes.

### Primary files

- `python/helios/scene.py`
- `python/helios/__init__.py`
- `tests/python/test_scene.py`

### Implementation

1. Add a private, no-field common scene wrapper, for example `_Scene`:

   ```python
   class _Scene(Model, cpp_class=_helios.Scene):
       ...
   ```

   `StaticScene` and `DynamicScene` both inherit from `_Scene`; the high-level
   `DynamicScene` must not inherit from the high-level `StaticScene`.

2. Put common internal behavior on `_Scene`:

   - `_set_reflectances(wavelength)` using the existing
     `_helios.set_scene_reflectances` helper.
   - `_from_cpp()` dispatch that returns `DynamicScene` for
     `_helios.DynamicScene`, `StaticScene` for `_helios.StaticScene`, and fails
     clearly for an unsupported bare `_helios.Scene`.

   Keep the dispatch in this private wrapper rather than teaching the generic
   model metaclass how to select arbitrary union members.

3. Change `StaticScene` to inherit `_Scene`; otherwise preserve its fields and
   behavior.

4. Add `DynamicScene(_Scene, cpp_class=_helios.DynamicScene)` with no model
   fields and no scene mutation methods.

5. Implement an explicit direct-construction guard. Internal construction from
   an existing `_cpp_object` must remain possible, while `DynamicScene()` must
   raise:

   ```text
   DynamicScene cannot be constructed directly; use DynamicScene.from_xml(...)
   ```

6. Implement `DynamicScene.from_xml(scene_file)` using the existing
   `AssetPath` validation, scene XSD validation, asset directories, and the
   checked C++ loader from WP1. Set `_is_loaded_from_xml`, disable direct YAML
   serialization for descendants, and record constructor provenance exactly as
   other XML-backed models do.

7. Make `StaticScene.from_xml()` check the returned C++ type and reject a
   dynamic result with a message directing the user to `DynamicScene`.

8. Export `DynamicScene` from `helios.__init__`.

### Acceptance criteria

- `helios.DynamicScene` is importable.
- Direct construction fails with the documented message.
- Dynamic XML constructs a high-level `DynamicScene` without wrapping any
  `ScenePart`; in particular, no `DSMO_*` ID conversion occurs.
- Static/dynamic wrong-loader cases fail clearly.
- `is_xml_loaded(dynamic_scene)` is true.
- Existing `StaticScene`, scene-part, binary, and visualization tests pass.

### Risk notes

- `Model._from_cpp()` normally traverses annotated fields. `DynamicScene` must
  remain fieldless so it never accesses the inherited C++ `scene_parts` vector.
- Avoid changing `validation.py` unless the private dispatcher proves
  impossible; such a change would widen the regression surface substantially.

---

## WP3 — Accept dynamic scenes in `Survey` and implement one-shot execution

### Objective

Allow both manual survey composition and `Survey.from_xml()` to use the new
wrapper, while preventing reuse of mutated dynamic state until WP7.

### Primary files

- `python/helios/survey.py`
- `python/helios/scene.py` for small private lifecycle helpers only
- `tests/python/test_survey.py`
- `tests/python/test_survey_interrupt.py`

### Implementation

1. Type `Survey.scene` through the private common scene base. Document the
   public type as `StaticScene | DynamicScene`.

2. Confirm that the generated property getter uses `_Scene._from_cpp()` so
   `Survey.from_xml(dynamic_survey_xml).scene` is a `DynamicScene` and static
   surveys remain unchanged.

3. In `Survey.run()`:

   - Finalize only non-XML `StaticScene` instances.
   - Continue applying reflectances to both scene types.
   - Continue survey/leg integration and scene shifting for a programmatically
     assembled survey containing an XML-loaded dynamic scene.
   - Run automatic scene-part ID assignment only for `StaticScene`.
   - Pass the original polymorphic C++ scene into `SurveyPlayback`; do not copy,
     cast, step, or rebuild it in Python.

4. Add private lifecycle state to `DynamicScene`, with at least `fresh`,
   `running`, and `consumed` semantics. This is runtime state and must not be
   serialized.

5. At the start of `Survey.run()`, reject a `running` or `consumed` dynamic
   scene before creating output or otherwise causing side effects.

6. Claim the scene immediately before playback is started. In a `finally`
   block mark it consumed. Once playback start has been attempted, interruption,
   callback failure, or simulation failure must not make the scene reusable.

7. Do not mark the scene consumed for errors that occur entirely during
   parameter validation or pre-playback setup.

### Acceptance criteria

- A programmatically composed survey runs once with `DynamicScene`.
- `Survey.from_xml()` with a dynamic survey runs once and exposes the correct
  scene type.
- A second `survey.run()` raises a deterministic error explaining that reset
  support is planned but not yet available.
- A run interrupted after playback starts also causes the next run to fail.
- A validation error before playback starts does not consume the scene.
- Dynamic output contains numeric hit-object IDs; Python never observes or
  rewrites `DSMO_*` IDs.
- Static survey rerun behavior is unchanged.

### Risk notes

- Place the lifecycle transition next to the actual playback start rather than
  at method entry, but perform the consumed check at method entry.
- The guard belongs to the scene, not only the survey, so it also protects
  against accidental reuse through another reference.

---

## WP4 — Enforce the initial ownership, clone, and live-view boundaries

### Objective

Turn currently unsafe or misleading operations into explicit errors without
changing the default `survey.run()` path.

### Primary files

- `python/helios/scene.py`
- `python/helios/survey.py`
- `python/helios/live.py`
- Focused tests in `tests/python/test_scene.py`, `test_survey.py`, and
  `test_live.py`

### Implementation

1. Give `DynamicScene` a weak owner reference and private claim/release helpers.
   `Survey` must claim a dynamic scene when assigned. A second live survey
   owner must be rejected. Static scene sharing remains supported.

2. Correctly handle reassignment and garbage collection so a dead Python
   survey does not permanently poison an otherwise fresh scene. Do not use the
   existing global uniqueness dictionary for this because it retains objects
   strongly and static scene sharing is intentionally supported.

3. Override direct `DynamicScene.clone()` and `__deepcopy__()` with a clear
   unsupported-operation error.

4. Before invoking the existing C++ survey clone path, reject
   `Survey.clone()`/`deepcopy()` when the survey owns a dynamic scene. Preserve
   existing static survey copy behavior.

5. Reject `live=True` or an explicit `LiveViewer` in `Survey.run()` when the
   scene is dynamic. Raise `NotImplementedError` before viewer attachment.

6. Update type hints that only read from a scene, such as force-on-ground
   helpers, where accepting both scene types is valid. Do not expose dynamic
   scene parts to make the current viewer work.

### Acceptance criteria

- Sharing one dynamic scene between two surveys fails; sharing a static scene
  remains covered by existing tests and succeeds.
- Direct dynamic clone/deepcopy and dynamic survey clone/deepcopy fail clearly.
- Static clone/deepcopy tests remain unchanged.
- Dynamic `run(live=True)` fails before opening a viewer or starting playback.

### Exit rule for future clone support

Do not remove clone restrictions merely because a C++ `DynScene` copy can be
constructed. Enable them only after tests demonstrate independent primitives,
dynamic objects, motion sequencers, loop counters, observer groves, and reset
state under interleaved execution.

---

## WP5 — Implement provenance-only YAML serialization

### Objective

Serialize the recipe for rebuilding a dynamic scene, never its runtime geometry
or progressed simulation state.

### Primary files

- `python/helios/scene.py`
- `python/helios/serialization.py` only if generic behavior needs a narrow fix
- `python/helios/data/serialization_schema.json`
- `tests/python/test_serialization.py`

### Required representation

Conceptually, a dynamic scene document is:

```yaml
model_class: helios.scene.DynamicScene
fields: {}
provenance:
  constructor:
    method: from_xml
    kwargs:
      scene_file: path/to/dynamic_scene.xml
  operations: []
```

Normal format version and `$schema` keys remain present as produced by the
generic serializer.

### Implementation

1. Keep `DynamicScene` fieldless so `fields` is always empty.

2. Register `helios.scene.DynamicScene` and its empty fields definition in the
   serialization schema. Do not bump the format version unless the schema
   policy requires it; adding a model class should remain backward compatible
   for existing documents.

3. Ensure `DynamicScene.to_yaml()` emits constructor provenance and no binary
   sidecar.

4. Ensure `DynamicScene.from_yaml()` reconstructs exclusively by replaying
   `DynamicScene.from_xml()`. Runtime lifecycle state must be fresh after
   reconstruction.

5. Cover a `DynamicScene` nested inside a programmatically composed `Survey`
   in shallow and inline YAML forms.

6. Cover a dynamic `Survey.from_xml()` root. Its root survey provenance should
   continue to rebuild the entire survey from the survey XML; implicitly
   constructed descendants must retain the existing serialization restrictions.

7. Ensure `to_binary`, `from_binary`, and binary bundle modes are absent or
   fail explicitly for `DynamicScene`; do not silently serialize it as a base
   `Scene` or `StaticScene`.

8. For a non-binary `DynamicScene.to_bundle()`, use the existing generic
   provenance bundling behavior. Do not add DynamicScene-specific traversal of
   references contained inside XML. The generic bundler currently copies files
   named directly by YAML provenance (including the scene XML), but dependency
   closure for files referenced by XML is a separate, project-wide concern
   shared by static scenes, surveys, scanners, and platforms.

### Acceptance criteria

- YAML contains an empty dynamic-scene field map and `from_xml` provenance.
- YAML round-trip returns a fresh `DynamicScene` backed by a C++ `DynScene`.
- A round-tripped scene can be placed in a survey and run once.
- No consumed/running state appears in YAML.
- Dynamic binary serialization is unavailable with an actionable error.
- Existing serialization documents and tests remain valid.

---

## WP6 — End-to-end dynamic behavior tests and user documentation

### Objective

Prove that the integration produces dynamic behavior, not merely that it can
finish a playback, and document the intentionally narrow first release.

### Primary files

- `tests/python/test_scene.py`
- `tests/python/test_survey.py`
- A small test fixture under `data/test/` or a temporary fixture generated by
  the test
- Python API documentation/notebook location used by the project

### Implementation

1. Create a minimal deterministic dynamic scene and short survey. Avoid the
   existing high-pulse production examples in the regular test suite.

2. Test both construction routes:

   - `DynamicScene.from_xml()` plus a programmatically composed survey.
   - `Survey.from_xml()` referencing a dynamic scene.

3. Demonstrate actual motion. Use a fixture with a deliberately visible
   translation/rotation and assert a stable property of returns over time or a
   difference from an equivalent static control. Completion alone is
   insufficient.

4. Test NPY output and at least one LAS/LAZ path. Assert numeric hit-object IDs
   from dynamic objects to cover `DSMO_` prefix handling and `std::stoi` in the
   LAS writer.

5. Add regression coverage for callbacks and interruption sufficient to prove
   that polymorphic scene stepping and the one-shot lifecycle guard coexist.

6. Document:

   - The exact `DynamicScene.from_xml()` API.
   - How to use it in a manually composed survey.
   - Automatic dispatch through `Survey.from_xml()`.
   - XML-controlled dynamic timing.
   - One-shot execution before WP7.
   - Unsupported mutation, binary serialization, cloning, sharing, and live
     visualization.
   - Provenance-only YAML and the existing generic bundle behavior, including
     its current project-wide XML dependency limitation.

### Acceptance criteria

- Tests fail if the scene is accidentally sliced to `StaticScene`.
- Tests fail if `Scene::doSimStep()` is not invoked.
- Tests remain fast enough for the regular Python suite.
- Public examples use only the supported high-level API.

---

## WP7 — Replace one-shot execution with state-efficient C++ reset

### Objective

Make repeated `survey.run()` calls equivalent to running a freshly loaded copy
of the same initially loaded asset snapshot, without reloading XML and without
storing a second full geometry copy.

This is deliberately the final milestone. Until all acceptance criteria pass,
retain the WP3 one-shot error.

### Equivalence contract

Given identical survey settings and RNG seed, the following must produce the
same measurements and trajectories:

1. Run an already used dynamic scene after reset.
2. Run a newly XML-loaded dynamic scene from the same original asset snapshot.

Reset should be preferred over reloading because it avoids reparsing, remains
stable if source files later change, and does not rerun randomized loading with
an advanced RNG state.

### Primary files

- `src/scene/dynamic/DynScene.h/.cpp`
- `src/scene/dynamic/DynObject.h` and concrete dynamic object implementations
- `src/scene/dynamic/DynMovingObject.h/.cpp`
- `src/scene/dynamic/DynSequentiableMovingObject.h/.cpp`
- `src/scene/dynamic/DynSequencer.h/.tpp`
- `src/scene/dynamic/DynSequence.h/.tpp`
- Dynamic grove/KD-tree code only where forced refresh is required
- `src/assetloading/SwapOnRepeatHandler.h/.cpp` if mixed swap/motion scenes are
  supported by the equivalence contract
- `python/helios/scene.py` and `survey.py` only to remove the one-shot guard
- C++ unit tests plus Python repeated-run tests

### Baseline-state constraint

Do not retain a duplicate `Scene`, duplicate primitive collection, or duplicate
vertex-position array solely for reset.

Reuse existing canonical state:

- `Scene::finalizeLoading()` already stores every vertex's pre-centering
  position in `Vertex::posOrigin`.
- The original CRS bounding box supplies the original scene shift.
- Motion definitions and sequence topology remain resident.
- Swap-on-repeat already retains baseline geometry where its semantics require
  it.

Small constant-sized state per dynamic object is allowed, for example an
accumulated rigid transform or initial centroid. It is not a redundant scene
copy.

### Implementation

1. Add internal reset methods, with names such as:

   ```cpp
   DynScene::resetSimulationState();
   DynObject::resetSimulationState();
   DynSequentiableMovingObject::resetSimulationState();
   DynSequencer::restart();
   ```

   Keep them off the high-level Python API.

2. Restore dynamic geometry positions from `Vertex::posOrigin` using the
   original CRS shift. Do not call `reset_to_canonical_position()` if doing so
   would discard baseline information needed for later resets.

3. Restore normals correctly for every supported primitive type:

   - Recompute triangle face normals and primitive caches from restored
     vertices.
   - Preserve custom vertex/voxel normals without storing a second normal per
     vertex if possible. The preferred approach is a compact accumulated rigid
     transform per moving object and inverse/reset of its linear component.
   - If that approach cannot exactly represent all XML dynamic motions,
     document the counterexample before selecting a different minimal baseline
     representation.

4. Recompute object centroids, primitive AABBs, and other derived caches after
   geometry restoration.

5. Reset all dynamic control state:

   - Scene step-loop current step.
   - Object step-loop current steps.
   - Observer/KD update loop current steps.
   - Position and normal motion queues.
   - `updated` flags.
   - Sequencer `current` pointer back to `start`.
   - Iteration count of every dynamic sequence, including sequences not current
     at the end of the prior run.

6. Force the dynamic KD grove to represent restored geometry immediately.
   Rebuild the grove or synchronously update every dynamic subject; merely
   resetting observer counters is insufficient because an observer may have
   intentionally lagged geometry at the end of the prior run.

7. Handle mixed dynamic-motion and swap-on-repeat scenes. The current swap
   handler consumes filter and TTL queues and deletes filters as swaps execute.
   To make reset equivalent to reload without reparsing XML, retain immutable
   swap recipes/configuration and rebuild runtime queues on reset. Reuse its
   existing baseline scene part; do not introduce another geometry baseline.
   If this refactor is deferred, continue rejecting subsequent runs for any
   scene containing swap-on-repeat handlers and state that limitation
   explicitly.

8. Invoke reset at the beginning of `DynScene::prepareSimulation()`, not only
   at the end of successful playback. This makes recovery from interruption or
   failure reliable. The first call must be idempotent and leave freshly loaded
   state unchanged.

9. Ensure reset occurs once per new `SurveyPlayback`, not between the internal
   plays managed by `SimulationPlayer`; internal swap/replay behavior must
   remain unchanged.

10. After C++ and Python equivalence tests pass, remove the consumed-state
    rejection from WP3. Retain a running/reentrancy guard and the prohibition on
    sharing one scene between concurrently active surveys.

11. Reassess dynamic survey clone support separately. Reset capability alone
    does not prove that two copied surveys own independent dynamic graphs. Keep
    clone restrictions unless the WP4 exit rule is satisfied.

### Acceptance criteria

- With the RNG reset to the same seed, run 2 on one dynamic survey is identical
  to run 1 and identical to a fresh XML-loaded control.
- The same equivalence holds after an interrupted run and after a playback that
  raises from a callback.
- Tests cover translation, rotation with custom normals, finite chained
  sequences, and an infinite (`loop=0`) sequence.
- Scene/object/observer step intervals begin at the same phase on every run.
- Dynamic KD-tree results are correct on the first pulse after reset.
- Internal multi-play swap behavior remains unchanged.
- If mixed swap/motion reset is supported, it matches a fresh load after all
  swap queues were previously consumed.
- Memory tests demonstrate that reset did not add a second full copy of scene
  geometry.
- Provenance YAML remains independent of runtime/reset state.

### Suggested verification strategy

For each reset fixture:

1. Load scene A and run it.
2. Reset the RNG, run scene A again.
3. Load scene B freshly from the same XML, reset the RNG, and run it.
4. Compare structured measurement and trajectory arrays exactly where current
   deterministic tests permit, otherwise use a documented numerical tolerance.
5. Inspect the first dynamic update and first ray intersection after reset to
   catch stale grove state that may not affect aggregate counts.

---

## Final completion criteria

The project is complete when:

- The minimal `DynamicScene.from_xml()` API is stable and documented.
- Manual and XML-loaded surveys execute dynamic behavior through ordinary
  `survey.run()`.
- Static scene behavior and tests are unaffected.
- Wrong loaders, mutation attempts, sharing, cloning, and live viewing fail
  according to the agreed contract.
- Provenance-only YAML works, and non-binary bundles follow the existing
  generic provenance behavior without adding XML dependency traversal.
- Before WP7, subsequent runs fail safely and clearly.
- After WP7, repeated runs reset efficiently and are equivalent to fresh loads
  without redundant full-scene storage.
