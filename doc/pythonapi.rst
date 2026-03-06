Python API reference
====================

This is the complete reference of the Python API of HELIOS++.

Surveys and legs
------------------


.. autoclass:: helios.Survey
   :members:

.. autoclass:: helios.Leg
   :members:

Scanner and scanner settings
----------------------------

.. autoclass:: helios.Scanner
   :members:

.. autoclass:: helios.ScannerSettings
   :members:

.. autofunction:: helios.list_scanners

.. autofunction:: helios.scanner_from_name

Platform, platform settings and trajectories
--------------------------------------------

.. autoclass:: helios.Platform
   :members:

.. autoclass:: helios.PlatformSettings
   :members:

.. autoclass:: helios.StaticPlatformSettings
   :members:

.. autoclass:: helios.DynamicPlatformSettings
   :members:

.. autoclass:: helios.TrajectorySettings
   :members:

.. autofunction:: helios.load_traj_csv

.. autofunction:: helios.list_platforms

.. autofunction:: helios.platform_from_name

Scene
------

.. autoclass:: helios.StaticScene
   :members:

.. autoclass:: helios.ScenePart
   :members:

.. autoclass:: helios.scene.Material
   :members:

.. autoclass:: helios.ForceOnGroundStrategy
   :members:

.. autoclass:: helios.scene.MaterialDict
   :members:

.. autoclass:: helios.scene.BoundingBox
   :members:

Settings
--------

.. autoclass:: helios.ExecutionSettings
   :members:

.. autoclass:: helios.OutputSettings
   :members:

.. autoclass:: helios.OutputFormat
   :members:

.. autoclass:: helios.FullWaveformSettings
   :members:

.. autoclass:: helios.LogVerbosity
   :members:

.. autoclass:: helios.ParallelizationStrategy
   :members:

.. autoclass:: helios.KDTreeFactoryType
   :members:

Utils
------

.. autofunction:: helios.add_asset_directory

.. autofunction:: helios.combine_parameters