Serialization Format
====================

HELIOS++ models can be serialized to YAML for interchange and to relocatable
bundle directories for portable storage. The public entry points are the model
methods ``to_yaml()``, ``from_yaml()``, ``to_bundle()`` and ``from_bundle()``.

The serialized YAML documents are validated against the JSON schema in
``python/helios/data/serialization_schema.json`` before they are loaded.

Document Structure
------------------

Each serialized model document stores four core fields:

- ``serialization_major_version`` for incompatible format changes
- ``serialization_minor_version`` for backwards-compatible format revisions
- ``model_class`` as the fully qualified Python class name
- ``fields`` for the serialized model payload

Two optional top-level sections can also appear:

- ``provenance`` records how an object was created and which mutating
  operations were applied afterwards
- ``binary`` points to a binary sidecar file for models that support
  ``from_binary()``

A typical inline YAML document looks like this:

.. code-block:: yaml

   serialization_major_version: 0
   serialization_minor_version: 0
   model_class: helios.platforms.DynamicPlatformSettings
   fields:
     x: 7.0
     y: 8.0
     z: 9.0
     trajectory_settings:
       serialization_major_version: 0
       serialization_minor_version: 0
       model_class: helios.platforms.TrajectorySettings
       fields:
         start_time: 2.0
         end_time: 8.0
         teleport_to_start: false
     speed_m_s: 33.0

Inline And Shallow YAML
-----------------------

``to_yaml(..., shallow=False)`` inlines nested model documents directly into the
parent ``fields`` mapping. This is the default mode and produces a single YAML
file.

``to_yaml(..., shallow=True)`` writes nested models into separate YAML files and
replaces them with ``model_ref`` objects:

.. code-block:: yaml

   serialization_major_version: 0
   serialization_minor_version: 0
   model_class: helios.platforms.DynamicPlatformSettings
   fields:
     x: 7.0
     y: 8.0
     z: 9.0
     trajectory_settings:
       model_ref: trajectorysettings.helios.yaml
     speed_m_s: 33.0

The referenced file then contains the nested model:

.. code-block:: yaml

   serialization_major_version: 0
   serialization_minor_version: 0
   model_class: helios.platforms.TrajectorySettings
   fields:
     start_time: 2.0
     end_time: 8.0
     teleport_to_start: false

The default filenames come from ``_serialization_filename()`` and currently use
``<classname>.helios.yaml``. Models can override this hook to choose a custom
root or child filename.

Bundle Layout
-------------

``to_bundle(path)`` always creates a directory bundle and stores the root model
as shallow YAML. Compared to plain shallow YAML, bundles additionally rewrite
provenance file references so that copied assets live inside the bundle.

For example, a small bundle may contain:

.. code-block:: text

   bundle/
     dynamicplatformsettings.helios.yaml
     trajectorysettings.helios.yaml

Bundles created from scene-based models may also contain copied assets such as
``.obj`` files, linked ``.mtl`` files and referenced textures. ``from_bundle()``
loads the class default root filename automatically, or an explicit relative
``filename=...`` can be provided.

Binary Sidecars
---------------

``to_bundle(path, binary=True)`` uses binary sidecar files for models that
implement ``to_binary()`` and ``from_binary()``. In that case the YAML document
stores metadata plus a pointer to the sidecar, while ``fields`` stays empty.

Example:

.. code-block:: yaml

   serialization_major_version: 0
   serialization_minor_version: 0
   model_class: helios.scene.StaticScene
   fields: {}
   binary:
     method: from_binary
     path: staticscene.helios.bin

Currently this is mainly relevant for large scene objects where binary storage
is much more compact and faster to load than a fully expanded YAML payload.

Provenance
----------

When a model is created via constructors such as ``from_xml()``, ``from_obj()``
or ``from_binary()``, HELIOS++ stores enough provenance information to recreate
the object during deserialization. Transforming methods such as ``translate()``,
``scale()`` and ``rotate()`` append entries to the ``operations`` list.

Representative example:

.. code-block:: yaml

   serialization_major_version: 0
   serialization_minor_version: 0
   model_class: helios.scene.ScenePart
   fields:
     force_on_ground: 0
   provenance:
     constructor:
       method: from_obj
       kwargs:
         obj_file: data/sceneparts/basic/box/box100.obj
         up_axis: z
     operations:
     - method: translate
       kwargs:
         offset:
         - 1.0
         - 2.0
         - 3.0
     - method: scale
       kwargs:
         factor: 1.2

When a bundle is written, file paths recorded inside provenance are rewritten to
bundle-local relative paths so that the bundle stays relocatable. Path
separators follow the local platform.

Versioning And Validation
-------------------------

The format uses a major/minor version pair:

- Increase ``serialization_major_version`` only for incompatible changes
- Increase ``serialization_minor_version`` for compatible revisions and add a
  migration in ``python/helios/serialization.py``

On load, HELIOS++ performs the following steps:

1. Parse the YAML document.
2. Validate it against ``python/helios/data/serialization_schema.json``.
3. Apply any registered minor-version migrations.
4. Reconstruct the model from ``binary``, ``provenance`` and/or ``fields``.

Notes And Limits
----------------

- Inline serialization rejects cyclic model graphs. Use shallow serialization
  if nested models need to be split across files.
- Only model classes listed in the generated JSON schema are accepted.
- ``binary=True`` is available through ``to_bundle()``; plain ``to_yaml()``
  always writes YAML payloads.
- Models that were only implicitly created as children of a parent ``from_xml()``
  object are intentionally not serializable on their own.
