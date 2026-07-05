C++ API reference
=================

This is the generated reference for core C++ APIs of HELIOS++.

Scanner
------------------

.. doxygenfile:: scanner/Scanner.h
.. doxygenfile:: scanner/ScannerHead.h
.. doxygenfile:: scanner/ScannerSettings.h
.. doxygenfile:: scanner/ScanningDevice.h
.. doxygenfile:: scanner/Trajectory.h
.. doxygenfile:: scanner/beamDeflector/AbstractBeamDeflector.h
.. doxygenfile:: scanner/beamDeflector/ConicBeamDeflector.h
.. doxygenfile:: scanner/beamDeflector/FiberArrayBeamDeflector.h
.. doxygenfile:: scanner/beamDeflector/OscillatingBeamDeflector.h
.. doxygenfile:: scanner/beamDeflector/PolygonMirrorBeamDeflector.h
.. doxygenfile:: scanner/beamDeflector/RisleyBeamDeflector.h
.. doxygenfile:: scanner/detector/AbstractDetector.h
.. doxygenfile:: scanner/detector/AbstractPulseRunnable.h
.. doxygenfile:: scanner/detector/FullWaveform.h
.. doxygenfile:: scanner/detector/FullWaveformPulseDetector.h
.. doxygenfile:: scanner/detector/FullWaveformPulseRunnable.h

Platform
------------------

.. doxygenfile:: platform/Platform.h
.. doxygenfile:: platform/PlatformSettings.h
.. doxygenfile:: platform/LinearPathPlatform.h
.. doxygenfile:: platform/SimplePhysicsPlatform.h
.. doxygenfile:: platform/GroundVehiclePlatform.h
.. doxygenfile:: platform/HelicopterPlatform.h
.. doxygenfile:: platform/InterpolatedMovingPlatform.h

Scene
------------------

.. doxygenfile:: scene/Scene.h
.. doxygenfile:: scene/Material.h
.. doxygenfile:: scene/StaticScene.h
.. doxygenfile:: scene/IntersectionHandlingResult.h
.. doxygenfile:: scene/RaySceneIntersection.h
.. doxygenfile:: scene/primitives/Primitive.h
.. doxygenfile:: scene/primitives/Triangle.h
.. doxygenfile:: scene/primitives/Vertex.h
.. doxygenfile:: scene/primitives/Voxel.h
.. doxygenfile:: scene/primitives/DetailedVoxel.h
.. doxygenfile:: scene/primitives/AABB.h
.. doxygenfile:: scene/dynamic/DynObject.h
.. doxygenfile:: scene/dynamic/DynMovingObject.h
.. doxygenfile:: scene/dynamic/DynSequentiableMovingObject.h
.. doxygenfile:: scene/dynamic/DynMotion.h
.. doxygenfile:: scene/dynamic/DynMotionEngine.h
.. doxygenfile:: scene/dynamic/DynSequence.h
.. doxygenfile:: scene/dynamic/DynScene.h

Survey and Simulation
------------------
.. doxygenfile:: main/LidarSim.h

.. doxygenfile:: sim/comps/Survey.h
.. doxygenfile:: sim/comps/Leg.h
.. doxygenfile:: sim/comps/ScanningStrip.h
.. doxygenfile:: sim/comps/SimulationPlayer.h
.. doxygenfile:: sim/comps/SimulationReporter.h
.. doxygenfile:: sim/core/Simulation.h
.. doxygenfile:: sim/core/SurveyHooks.h
.. doxygenfile:: sim/core/SurveyPlayback.h
