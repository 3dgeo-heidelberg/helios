Python API reference
====================

This is the complete reference of the Python API of HELIOS++.

Surveys and legs
------------------


.. autoapiclass:: helios.Survey
   :members:

.. autoapiclass:: helios.Leg
   :members:

Scanner and scanner settings
----------------------------

.. autoapiclass:: helios.Scanner
   :members:

.. autoapiclass:: helios.ScannerSettings
   :members:

.. autoapifunction:: helios.list_scanners

.. autoapifunction:: helios.scanner_from_name

Platform, platform settings and trajectories
--------------------------------------------

.. autoapiclass:: helios.Platform
   :members:

.. autoapiclass:: helios.PlatformSettings
   :members:

.. autoapiclass:: helios.StaticPlatformSettings
   :members:

.. autoapiclass:: helios.DynamicPlatformSettings
   :members:

.. autoapiclass:: helios.TrajectorySettings
   :members:

.. autoapifunction:: helios.load_traj_csv

.. autoapifunction:: helios.list_platforms

.. autoapifunction:: helios.platform_from_name

Scene
------

.. autoapiclass:: helios.StaticScene
   :members:

.. autoapiclass:: helios.ScenePart
   :members:

.. autoapiclass:: helios.scene.Material
   :members:

.. autoapiclass:: helios.ForceOnGroundStrategy
   :members:

.. autoapiclass:: helios.scene.MaterialDict
   :members:

.. autoapiclass:: helios.scene.BoundingBox
   :members:

Settings
--------

.. autoapiclass:: helios.ExecutionSettings
   :members:

.. autoapiclass:: helios.OutputSettings
   :members:

.. autoapiclass:: helios.OutputFormat
   :members:

.. autoapiclass:: helios.FullWaveformSettings
   :members:

.. autoapiclass:: helios.LogVerbosity
   :members:

.. autoapiclass:: helios.ParallelizationStrategy
   :members:

.. autoapiclass:: helios.KDTreeFactoryType
   :members:

Utils
------

.. autoapifunction:: helios.add_asset_directory

.. autoapifunction:: helios.combine_parameters

.. autoapifunction:: helios.utils.set_rng_seed