Command line interface and XML configuration
********************************************

HELIOS++ can be run via a command line interface (CLI).
This has been the main interface for HELIOS++ before version 3.0 and is still available for users who prefer it over the Python API.
In this interface, all components of the simulation (scanners, platforms, scenes, surveys) are defined via configuration files in XML format.
In the HELIOS++ repo, you will find a ``data`` folder with a number of example surveys and scenes.

.. figure:: /img/HELIOS_figure.png
   :alt: Typical workflow of a simulation with HELIOS++ (using XML files as input)
   :width: 100%
   :align: center

   Typical workflow of a simulation with HELIOS++ (using XML files as input)

CLI usage
=========

The CLI is available via the ``helios`` command, which is installed with the HELIOS++ Python package.

To print the HELIOS++ help message, type:

.. code-block:: bash

    helios --help

This gives you an overview of additional, optional arguments.

To run a survey, use the following command:

.. code-block:: bash

    helios <survey-file>

<survey-file> is the absolute or relative path to a survey XML file. 
In this XML file, paths pointing to the scene XML, the platform XML and the scanner XML are provided.
If those paths are relative (recommended for sharing), their base directory needs to be in the list of HELIOS++ ``assetsPaths``. 
This way, it can be sufficient to only specify filepaths in the XML survey and scene files and then adding search folders to the ``assetPath`` list. To do so, add one or multiple file paths via the ``--assets`` argument, e.g.,

.. code-block:: bash

     helios <survey-file> --assets <path/to/my/scenes> --assets <path/to/my/sceneparts>

If you have cloned the HELIOS++ repository, you can run the demos provided in the ``data`` folder. For example, to run the ``simple_survey.xml`` survey, use the following command:

.. code-block:: bash

    helios data/surveys/demo/tls_arbaro_demo.xml

This will simulate terrestrial laser scanning (TLS) scans of two trees from two scan positions.
The output will be created in the ``output/arbaro_demo_tls``-folder under the timestamp of the simulation start. This point cloud can be visualized e.g. using Cloud Compare.

To modify the format of the output (default: xyz), you can use the ``--format`` argument, e.g.,

.. code-block:: bash

    helios data/surveys/demo/tls_arbaro_demo.xml --format laz

XML configuration
==================

Survey XML
----------

The survey XML file contains references to the components needed to build a simulation: The scene, the platform and the scanner (see also `:doc:`Scanners and platforms <scanners_platforms>). 
It also contains waypoint information needed to define the scan positions or trajectory.

Linking to the different components is done by specifying the absolute or relative path of the respective XML file in the ``<survey>`` tag, followed by a hashtag (#) and the ID of the entry:

.. code-block:: xml

    <survey name="toyblocks_als" 
        platform="data/platforms.xml#sr22"
        scanner="data/scanners_als.xml#riegl_vq-880g"
        scene="data/scenes/toyblocks/toyblocks_scene.xml#toyblocks_scene">

The ``platform``  attribute of the ``<survey>`` tag can also be set to "interpolated".
In this case, the platform behavior will be defined by a function obtained through interpolation of given trajectory data, see :ref:`interpolated-trajectories`

Scanner settings and platform settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Scanner settings and platform settings are defined at the document level before the ``<survey>``-tag and/or within the ``<leg>``-tags.
Setting one or more templates with defined scanner settings at the beginning of the file enables the reuse of these settings for the different legs.
Within the ``<scannerSettings>`` or ``<platformSettings>`` in each ``<leg>``, the globally defined settings can be referenced by an ID using the template parameter.
Additionally, all settings can be given explicitly in the ``<scannerSettings>`` and ``platformSettings`` of the ``<leg>``.
If the same setting is defined in a template and in a leg which uses the template, the value that is given in the leg is taken. So the priority is always: Leg -> Template -> Default.

.. code-block:: xml

    <?xml version="1.0"?>
    <document>
            <platformSettings id="platform1" z="35.000" onGround="false" movePerSec_m="5" stopAndTurn="true"/>
            <scannerSettings id="scanner1" active="true" pulseFreq_hz="100000" scanAngle_deg="90" scanFreq_hz="50" trajectoryTimeInterval_s="0.05"/>
        <survey name="toyblocks_uls_stopturn" platform="data/platforms.xml#quadcopter" scanner="data/scanners_als.xml#riegl_vux-1uav" scene="data/scenes/toyblocks/toyblocks_scene.xml#toyblocks_scene">
        <!-- platform: quadcopter, deflector: rotating, stop and turn-mode
            OPTIONAL: detectorSettings and FWFSettings, examples below
                <detectorSettings accuracy_m="0.001" rangeMin_m="1.5" rangeMax_m="200"/>
                <FWFSettings winSize_ns="1.5" beamSampleQuality="3"/> -->
                    <!-- leg000 -->
            <leg>
                <platformSettings template="platform1" x="-80.0" y="-50.0"/>
                <scannerSettings template="scanner1"/>
            </leg>
                    <!-- leg001 -->
            <leg>
                <platformSettings template="platform1" x="80.0" y="-50.0"/>
                <scannerSettings template="scanner1"/>
            </leg>
                    <!-- leg002 -->
            <leg>
                <platformSettings template="platform1" x="-80.0" y="50.0"/>
                <scannerSettings template="scanner1" pulseFreq_hz="300000"/>
            </leg>
                    <!-- leg003 -->
            <leg>
                <platformSettings template="platform1" x="80.0" y="50.0"/>
                <scannerSettings template="scanner1" active="false"/>
            </leg>
        </survey>
    </document>

In this example, the ``platformSettings`` template "platform1" is used for all flight lines (legs), so only the x and y position have to be specified for each leg.
Furthermore, all legs use the template "scanner1" for the ``scannerSettings``. For leg002, the pulse frequency is increased to 300 kHz.
Hence, the pulse prequency defined in the template is overridden, all other settings (scan frequency, scan angle, etc.) remain as defined in the template.

Detector settings and Fullwave settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the ``<survey>`` tag and typically before the legs, it is possible (but optional) to define ``<detectorSettings>`` and ``<FWFSettings>`` which will overwrite the default settings.

In the ``<detectorSettings>`` tag, the device accuracy, minimum range and maximum range of the detector can be specified.

.. code-block:: xml

    <detectorSettings accuracy_m="0.001" rangeMin_m="5" rangeMax_m="600"/>

The FWFSettings-tag is used to configure the discretization of the full waveform in space and time and the window size for the peak detection. The parameters are explained in detail on the pages Scanners and Fullwave processing.
This would be the confugration for the current default settings: 

.. code-block:: xml

    <FWFSettings beamSampleQuality="3" binSize_ns="0.25" winSize_ns="1"/>

Leg definition 
^^^^^^^^^^^^^^

For each scan position, waypoint or trajectory snippet, a ``<leg>`` is defined.
Optionally, a ``stripId`` can be assigned to a leg. Legs with the same ``stripId`` will be grouped and their simulated point cloud is written to a single output file, named by the ``stripId``.
Legs with no ``stripId`` are considered as separate strips and their outputs are numbered consecutively (leg000, leg001, etc.).

For moving platforms, legs are typically finished when the platform reaches the next waypoint.
Static rotating platforms are usually finished when they finish the specified rotation.
Some systems will not move or rotate at all, e.g., static Risley scanners such as the Livox.
To control the integration time for such systems, a maximum duration in seconds (``maxDuration_s``) can be specified, after which the simulation terminates.
``maxDuration_s`` will supersede any other stopping criteria also for moving/rotating platforms.
Within the ``<leg>`` tag, the platform position is provided in the tag ``<platformSettings>``. For dynamic platforms, at least two legs have to be defined corresponding to the start and stop waypoints and the speed between two waypoints is set with the parameter ``movePerSec_m``.
The direction of movement is determined by the position of the next waypoint.
The onGround parameter is useful for terrestrial surveys. It is "false" by default, but when set to "true", the platform is automatically placed onto the ground regardless of the specified z-coordinate.
The ground is determined as the lowest z-coordinate in the data at the xy-position of the leg.

Scanner settings for each leg are defined in the ``<scannerSettings>``-tag. This includes the scanner activity, pulse frequency, scan angle, scan frequency, head rotation and trajectory output.
If a ``template`` is specified, the settings are taken from the template with the given ID, which must be defined at the beginning of the XML document before the ``<survey>``-tag.
If a setting is specified both in the template and in the leg, the value given in the leg is taken. 

.. _interpolated-trajectories:

Interpolated trajectories
^^^^^^^^^^^^^^^^^^^^^^^^^^

To replicate previous real-world surveys, a trajectory file can be supplied as input to a leg instead of setting waypoints manually.
This has the added advantage that the platform's attitude can be considered, which may, e.g., come from flight planning tools. To enable the interpolation, set the platform in the ``<survey>`` tag to "interpolated".

When the platform is defined from trajectory data, a basePlatform can be defined in the ``<survey>`` tag from which the scanner mount and other important platform attributes are retrieved.

It is also possible to configure the scanner mount directly on the survey XML. For this, define a ``<scannerMount>`` element as a direct child of the ``<survey>`` element. The child ``<scannerMount>`` syntax is the same as usual.
In the example below, the scanner is mounted with an offset of 0.2 m in the vertical coordinate. Besides, its attitude is defined by a 180-degree rotation on the x-axis and another 175-degree rotation on the z-axis.

.. code-block:: xml

    <survey ...>
	    <scannerMount x="0" y="0" z="0.2">
		    <rot axis="x" angle_deg="180" />
		    <rot axis="z" angle_deg="175" />
	    </scannerMount>        
    </survey>


The following XML snippet defines a platfrom from trajectory data. The first leg follows the entire trajectory.
The second leg teleports to the position at t=4.78 and moves until the position at t=6.94 is reached. Note that it requires the ``teleportToStart`` flag to be true, because it needs to start at a previously passed position.
The third leg goes from the position at t=8.02 until the end. In this case, it is not necessary to enable the ``teleportToStart`` flag because the start time comes after the previous end time.
The fourth leg starts again from the very beginning. Since all legs are defined from the same trajectory data, the column specification of the first leg is assumed for all legs.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <document>
        <!-- Default scanner settings: -->
        <scannerSettings id="scaset" active="true" pulseFreq_hz="70000" scanAngle_deg="60" scanFreq_hz="50" />
        <survey name="interpolated_trajectory_als" scene="data/scenes/demo/interpolated_trajectory.xml#interpolated_trajectory_demo" platform="interpolated" basePlatform="data/platforms.xml#sr22" scanner="data/scanners_als.xml#leica_als50-ii">
        <FWFSettings beamSampleQuality="3" binSize_ns="0.25" winSize_ns="1"/>
            <!-- Leg which interpolates the full trajectory -->
        <leg>
                <platformSettings 
                    trajectory="data/trajectories/cycloid.trj"
                    tIndex="0" xIndex="4" yIndex="5" zIndex="6" rollIndex="1" pitchIndex="2" yawIndex="3"
                    slopeFilterThreshold="0.0" toRadians="true" syncGPSTime="false"
                />
                <scannerSettings template="scaset" trajectoryTimeInterval_s="0.054"/>
        </leg>
            <!-- Leg which interpolates the trajectory for all t in [4.78, 6.94] -->
        <leg>
                <platformSettings trajectory="data/trajectories/cycloid.trj" tStart="4.78" tEnd="6.94" teleportToStart="true"/>
                <scannerSettings template="scaset" trajectoryTimeInterval_s="0.054"/>
        </leg>
                <!-- Leg which interpolates the trajectory for all t in [8.02, tb] where tb is the final time -->
        <leg>
                <platformSettings trajectory="data/trajectories/cycloid.trj" tStart="8.02"/>
                <scannerSettings template="scaset" trajectoryTimeInterval_s="0.054"/>
        </leg>
                <!-- Leg which interpolates the full trajectory again from the start (3.7 is the time of the first point)  -->
        <leg>
                <platformSettings trajectory="data/trajectories/cycloid.trj" tStart="3.7" teleportToStart="true"/>
                <scannerSettings template="scaset" trajectoryTimeInterval_s="0.054"/>
        </leg>
        </survey>
    </document>

The ``<platformSettings>`` tag inside a ``<leg>`` tag in the survey file can be used to configure the interpolation.
The first attribute is the trajectory, which is the only mandatory one for interpolated platforms. It can be used to define the path to the trajectory file.
A trajectory file can be a simple CSV with neither comments nor header. By default it is expected to have columns with (time, roll, pitch, yaw, x, y, z) format.
It is also possible to specify an arbitrary data format using the comment ``#HEADER: "t", "roll", "pitch", "yaw", "x", "y", "z"`` in the trajectory file.
The aforementioned header definition matches the default column order, but it can be arbitrarily changed to specify any column order as long as the names are correctly spelled.
Anything that is specified in the survey XML file will overwrite any specification in the trajectory file:
Here, the column order can be specified simply by adding index attributes to the ``<platformSettings>`` tag: ``tIndex``, ``xIndex``, ``yIndex``, ``zIndex``, ``rollIndex``, ``pitchIndex``, and ``yawIndex``.

If angles should not be considered, the interpolationDomain should be set to "position" (default is "position_and_attitude").

By default, it is assumed that angles are given in degrees. Nonetheless, it is possible to give them already in radians. In this last case, the attribute ``toRadians`` must be explicitly set to "false" to prevent an unexpected conversion.
Furthermore, the attribute ``trajectory_separator`` can be used to specify the column separator for input trajectory files.

It is also possible to define ``tStart`` and ``tEnd`` attributes for the ``<platformSettings>`` tag of a ``<leg>``. They define the time of the start and the end of the leg, respectively.
This means that only the part of the trajectory between these two time points will be interpolated for the leg.

If ``tStart`` is not specified, the leg will start at the first point in time. If ``tEnd`` is not specified, the leg will end at the last point in time.
When the attribute ``teleportToStart`` is set to true, the platform is forced to teleport to the position of the starting point (either the first point in time in the trajectory or ``tStart``).

The same trajectory file can be repeated among different legs.
However, it is only possible to define its column order once.
When using the same trajectory file for different legs and explicitly setting the ``tStart`` to skip parts of the trajectory or to repeat parts of the trajectory, make sure to also set ``teleportToStart`` to true.
Furthermore, if you have multiple legs using different trajectory files (the later ones having higher time values), keep in mind that using ``teleportToStart`` without setting a ``tStart`` will use the start time of the first trajectory, because internally, all trajectories are merged to a single trajectory.
So either specify neither ``tStart`` nor ``teleportToStart`` or specify both.

It is also possible to synchronize the starting GPS time of the simulation with the minimum time value from the input trajectory data. For this, set the attribute ``syncGPSTime`` to true.

Since trajectory files are translated to a matrix where each row represents the behavior of the platform at a certain time, it can be interesting to reduce the size of this matrix while minimizing the information loss.
This can help to digest big trajectory files, besides facilitating interpolation. That is what ``slopeFilterThreshold`` attribute can be useful for. `
When it is configured to a value greater than zero, forward finite differences are used to approximate the derivative for each column, which is then understood as the slope of a linear interpolation by pieces.
Whenever the accumulated deviation with respect to the first slope does not exceed the ``slopeFilterThreshold``, it is assumed that all intermediary data points can be approximate well enough by a linear interpolation.
Therefore, all points but the first and the last one are discarded.

Scene XML
---------

The scene is referenced in the ``<survey>`` tag of the survey.xml together with the survey name, the platform, the scanner and optionally a seed.
Referencing the different components is done by specifying the absolute or relative path of the XML files, followed by a hashtag (#) and the ID of the entry.

.. code-block:: xml

    <survey name="toyblocks_als" 
            platform="data/platforms.xml#sr22"
            scanner="data/scanners_als.xml#riegl_vq-880g"
            scene="data/scenes/toyblocks/toyblocks_scene.xml#toyblocks_scene">

A scene can be defined in a separate XML file or in the same XML file as the survey (as shown in ``data/surveys/toyblocks/uls_toyblocks_survey_scene_combo.xml``).

The scene XML links to separate files, which hold the actual geometry data.
The XML file also defines how the raw data that is read from the files should be preprocessed and arranged to build the scene.

A scene XML file starts with a ``<scene>`` tag, containing the id and name of the scene.
Any number of ``<part>`` tags can be specified inside the ``<scene>`` tag. Each of them contains one or more ``<filter>`` tags.
There are four loaders for different geometry types and three filters for coordinate transformations.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <document>
        <scene id="toyblocks_scene" name="ToyblocksScene">
            <part>
                <filter type="objloader">
                    <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
                    <param type="string" key="up" value="z" />  <!-- z is the default -->
                </filter>
                <filter type="scale">
                    <param type="double" key="scale" value="70" />
                </filter>
            <filter type="translate">
            <param type="vec3" key="offset" value="20.0;0;0" />
            </filter>
            </part>
            <part>
                <filter type="objloader">
                    <param type="string" key="filepath" value="data/sceneparts/toyblocks/cube.obj" />
                    <param type="string" key="up" value="y" /> <!-- this would be the required for the default export e.g. in Blender -->
                </filter>
            </part>
    <!-- ... -->
        </scene>
    </document>

Loaders
^^^^^^^^

The loader filters are distinguished by their type. Within each loader filter, a parameter specifying the path to the file has to be specified.

.. code-block:: xml

    <filter type="objloader">
        <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
    </filter>

HELIOS++ uses either polygon mesh objects or voxels as geometry, which are loaded in different ways.

To load multiple files with the exact same settings, the files can be specified with regular expressions using the "efilepath" key.

.. code-block:: xml

    <param type="string" key="efilepath" value="data/sceneparts/toyblocks/.*.obj" />

This way, the geometry from each file will receive a unique incremental ID. Alternatively, a single custom ID can be specified in the ``<part>``-tag, which is applied to all geometries loaded with the efilepath parameter:
When using ``efilepath``, all files are loaded to one scenepart and subsequently split into one subpart per file.
Origin, rotation and scale information are applied before splitting and thus, each scene part coming from the original one still considers the origin and transformation specifications of the original (merged) scene part.

.. code-block:: xml

    <part id="1">
        <filter type="objloader">
            <param type="string" key="efilepath" value="data/sceneparts/buildings/.*.obj"/>
        </filter>
        <filter type="rotate">
            <param type="rotation" key="rotation">
                <rot axis="x" angle_deg="90" />
            </param>
        </filter>
    </part>

Wavefront Object Mesh Loader
""""""""""""""""""""""""""""

.. code-block:: xml

    <filter type=“objloader”>

This loader reads polygon meshes with associated material definitions from a Wavefront Object (.obj) file.

The orientation of the mesh can be specified with the ``up`` parameter within a separate param-tag. Often, the default value="z" will be suitable.
However, some software (e.g., Blender) exports meshes with the y-axis pointing upwards per default, so here we would specify value="y".

.. code-block:: xml

    <param type="string" key="up" value="y" /> <!-- the default is 'z', but if it is not set explicitly, we will print an info message -->

Within the obj file, the line mtllib file.mat links to a material file. Multiple materials can be defined in one material file.
All faces, which are preceeded by the line usemtl material1 will ``use material1`` , all faces preceeded by ``usemtl material2`` will use material2 and so on.


GeoTIFF Loader
""""""""""""""

.. code-block:: xml

    <filter type="geotiffloader">

This loader reads a terrain elevation map in the GeoTIFF format and converts it into a triangle mesh. The centers of the pixels are used as data points.
If all points are valid (following the invalid value definition from the GeoTiff header), two triangles are built up: Current point - right neighbor - upper right neighbor and current point - upper neighbor - upper right neighbor.

If any of the three points has an invalid value, the triangle is omitted.

XYZ Point Cloud Loader
""""""""""""""""""""""

.. code-block:: xml

    <filter type="xyzloader">

This loader reads an XYZ point cloud from a text file. It subdivides the space into a grid of cubic cells ("voxels") and checks whether a cell contains at least one point of the point cloud.
If this is the case, the cell is defined as "solid" and an axis-aligned bounding box primitive with the extent of the cell is created, providing a surface to be virtually scanned in HELIOS++.
HELIOS++ expects the columns to have the following order: ``x, y, z, Nx, Ny, Nz``.
Only x, y and z are mandatory. For files with different format, column indices can be given explicitly. 

Several parameters can be added:

* The ``separator`` parameter defines the separator used in the ASCII file, e.g. space or comma.
* The ``voxel_size`` parameter defines the size of the cubic cells. The smaller the voxel size, the more detailed the resulting geometry, but also the higher the memory usage and runtime. **Note**: If using a scale filter in combination with the xyzloader, ``voxelSize`` corresponds to the ``voxelSize`` before scaling.
* The ``sparse`` parameter controls whether a sparse voxel grid is used for the conversion of the XYZ point cloud to a 3D model. The default and recommended setting is "true". If false, a dense voxel grid will be used, which can lead to higher memory usage. 
* Normal indices can be explicitly defined with the parameters ``normalXIndex``, ``normalYIndex``, and ``normalZIndex``.
* Estimation of normals, in case they are not provided, can be controlled with the ``estimateNormals`` parameter. Additional parameters ``defaultNormal`` and ``snapNeighborNormals`` can be used to control the normal estimation (see below).

To obtain ray incidence angles for voxels for the calculation of return intensity, HELIOS++ uses normals. For point cloud based voxel models, there are different options to determine these normals:

- If the point cloud does not contain point normals, normals will always be calculated using Singular Value Decomposition (SVD). This is also possible if the input file contains normals by explicitly specifiying ``<param type="int" key="estimateNormals" value="1" />`` or ``<param type="bool" key="estimateNormals" value="2" />``. In mode 1, all normal computations are performed at once in memory, which is working well for small point clouds. Mode 2 handles the computations in separate batches with a batch size of 10,000,000 points. This is slower than mode 1 but recommended for large point clouds.
- For voxels containing less than three points, no normal can be estimated. These voxels are either discarded or they are assigned a ``defaultNormal`` if it is specified as a vector like this: ``<param type="vec3" key="defaultNormal" value="0;0;1" />``
- The default behavior for point clouds with normals is to average the normals of each point within the voxel to derived the voxel normal. As an alternative, the normal of the point closest to the voxel center can be assigned as point normal by setting: ``<param type="boolean" key="snapNeighborNormal" value="true" />``.
- If the command line flag ``--fixedIncidenceAngle`` is provided, a fixed incidence angle of exactly 0.0 will be considered for all intersections (also for other object types).


DetailedVoxels
""""""""""""""

.. code-block:: xml

    <filter type="detailedvoxels">

This loader reads voxel models in a text format with .vox extension, inspired by the format used in the software `AMAPVox`_ software (`Vincent et al. 2017`_).
The primary purpose of DetailedVoxels is to model vegetation with given leaf properties.

.. _AMAPVox: https://amap-dev.cirad.fr/projects/amapvox
.. _Vincent et al. 2017: http://dx.doi.org/10.1016/j.rse.2017.05.034

There are three modes available for handling ray intersections with DetailedVoxels.

1. Transmittive mode (``<param type="string" key="intersectionMode" value="transmittive" />``)
2. Scaled mode (``<param type="string" key="intersectionMode" value="scaled" />`` with a scaling factor defined like ``<param type="double" key="intersectionArgument" value="0.5" />``)
3. Fixed mode (``<param type="string" key="intersectionMode" value="fixed" />``)

For detailed information about the VOX file format and the different intersection modes, refer to the Python documentation: `Scene - VOX files <scene.ipynb#4)-VOX-files>`_


Coordinate transformations
^^^^^^^^^^^^^^^^^^^^^^^^^^

Coordinate transformation filters can be passed to every scene geometry in order to translate, rotate or scale the geometry.

Rotations and scaling are always about the origin, i.e. (0,0,0), of the object's coordinate system. The translation transforms the origin of the object coordinate system.

Translate
""""""""""

This filter applies a translation to the geometry which is given in the form of a 3D vector, separated by semicolons. The following tag shifts the geometry in x-direction by 5 units and in y-direction by -12.5 units.

.. code-block:: xml

    <filter type="translate">  
        <param type="vec3" key="offset" value="5.0;-12.5;0" />  
    </filter>

In many cases, objects should be placed on the ground. HELIOS++ can perform this translation automatically with the ``onGround``-parameter.

For this parameter to work, an object in the scene has to be labelled as ground in a corresponding material file by inserting the following line: helios_isGround true (see Materials / Intensity Modelling). The ``onGround`` parameter can take on the following values:

* ``0``: No usage.
* ``-1``: Find optimal ground translation.
* ``1``: Find quick ground translation.
* ``>1``: Specify a given depth for the search process.

Rotate
""""""

This filter applies a rotation to the geometry scenepart, using the axis-angle representation. The following example performs a rotation of 90° around the x-axis or unit vector (1,0,0).

.. code-block:: xml

    <filter type="rotate">
        <param key="rotation" type="rotation">  
            <rot angle_deg="90" axis="x"/>  
            <rot angle_deg="0" axis="y"/>  
            <rot angle_deg="0" axis="z"/>  
        </param>  
    </filter>

Scale
"""""""

This filter scales the geometry by a given factor.

.. code-block:: xml
    
    <filter type="scale">
        <param type="double" key="scale" value="0.5" />
    </filter>

.. _materials_cli:

Materials
^^^^^^^^^

HELIOS++ reads material properties from `MTL material library files`_. Following the standard, these files and their materials are linked to mesh faces using the ``mtllib`` and ``usemtl`` statements in the .OBJ file.

For ``xyzloader``, ``geotiffloader``, and ``detailedVoxels``, materials can be specified in the XML file for each scenepart. The ``matfile`` parameter links to a specific material file, similar to the ``mtllib`` line in .obj files.
The ``matname`` parameter specifies a specific material (here: "leaves") in the material file. It is the equivalent to the ``usemtl`` line in .obj files:

.. code-block:: xml

    <param type="string" key="matfile" value="data/sceneparts/arbaro/tree.mtl" />
    <param type="string" key="matname" value="leaves" />

Furthermore, voxel material also supports uniform randomization. For this, a number of random materials and a random range has to be specified:

.. code-block:: xml

    <param type="integer" key="randomMaterials" value="5" />
    <param type="double" key="randomRange" value="0.1" />

Read more about materials including custom HELIOS parameters on the page :doc:`Full waveform and intensity modelling <../intensity_fwf>`.

.. _MTL material library files: http://paulbourke.net/dataformats/mtl/


Dynamic scenes (rigid motions)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A dynamic scene is a scene that experiences some type of update during the simulation, i.e., a scene with dynamic moving updates.
HELIOS++ currently supports rigidly moving dynamic objects which are scene parts which experience at least one type of rigid motion during the simulation.

Dynamic moving objects
""""""""""""""""""""""

In the current version, dynamic scenes can only be specified using an XML file.
For each dynamic moving object, this requires defining the sequence of rigid motions that the object should perform.
HELIOS++ supports all possible rigid motions in 3D, which are illustrated in the following table.

.. list-table::
   :widths: 15 10 15 60
   :header-rows: 1

   * - Rigid motion
     - XML type
     - XML attributes
     - Description
   * - Translation
     - translation
     - vec, autoCRS
     - Add a translation vector
   * - Reflection
     - reflection
     - ortho
     - Reflection with respect to a plane defined by its orthonormal vector
   * - Glide plane
     - glideplane
     - ortho, shift
     - Combination of reflection and translation
   * - Rotation
     - rotation
     - axis, angle, center, autoCRS, selfMode
     - Rotation around given rotation axis (in degrees) around a rotation center (if specified) or around the origin O(0, 0, 0)
   * - Helical motion
     - helical
     - axis, angle, glide
     - Combination of rotation and translation (in degrees)
   * - Rotational symmetry
     - rotsym
     - axis, angle, center
     - Combination of rotation and reflection (in degrees)

Rigid motions that are defined with respect to an associated linear variety (e.g. rotations, which are associated with a rotation axis, a 1D linear variety), also support a ``selfMode`` XML attribute which specifies that the rigid motion must be applied centered with respect to the object.
For example, a rotation which is specified with selfMode="true" implies that the object will rotate around its own centre.

Any scene part that is meant to be a dynamic moving object must be defined using the following general structure (example for a rotation):

.. code-block:: xml

    <part>
    ...
    <dmotion id="my_motion" loop="0">
        <motion type="rotation" axis="0;0;1" angle="1" selfMode="true"/>
    </dmotion>
    </part>

The ``<dmotion>`` element defines a dynamic motion sequence that consists of the computation of all ``<motion>`` elements contained inside in the exact order that they are specified.
In case multiple dynamic motions are defined for the same scene parts to create a motion sequence, these have to be chained together explicitly by defining a ``next="another motion"`` attribute 
so the dynamic motion ``<dmotion id="another_motion" ...> ... </dmotion>`` will be executed after the current one has finished.

Note that the above case defines "my_motion" as the identifier for the dynamic motion sequence.
Furthermore, it will be executed during the entire simulation because the ``loop="0"`` attribute means that the motion must be applied as an infinite loop.
Specifying ``loop="n"`` for any n greater than 0 will lead to the dynamic motion being executed n times before finishing.

Motion definition
"""""""""""""""""

The different rigid motions that can be specified are detailed in this section, in the same order as they appear in the table above.

A **translation** can be defined as

.. code-block:: xml

    <motion type="translation" vec="x;y;z"/>

The vector components specify how many meters for axis ``x``, ``y`` and ``z`` will be added to the object position at each iteration. That is, at each loop iteration the new position of a point ``p`` can be defined as ``p' = p + (x, y, z)``.

For the sake of simplicity, translations can use the ``autoCRS`` attribute to automatically align the translation with respect to the internal coordinate reference system of the simulator, e.g.:

.. code-block:: xml

    <motion type="translation" vec="x;y;z" autoCRS="1" />


A **reflection** can be defined as

.. code-block:: xml

    <motion type="reflection" ortho="x;y;z"/>

The vector ``ortho`` is understood as the orthonormal vector defining the reflection plane. For example, if the orthonormal vector is ``(0, 0, 1)``, then it is the ``e3`` vector of the canonical basis for the 3D space (``z`` axis). Thus, the reflection plane would have ``e1`` and ``e2`` (``x`` and ``y`` axis) as its basis.


A **glide plane** can be defined as

.. code-block:: xml

    <motion type="glideplane" ortho="u;v;w" shift="x;y;z"/>

The ``ortho`` vector is understood as the orthonormal vector defining the reflection plane. The ``shift`` vector specifies how many meters for axis ``x``, ``y`` and ``z`` will be added to the object position at each iteration.


A **rotation** can be defined as:

.. code-block:: xml

    <motion type="rotation" axis="x;y;z" angle="a" />

The ``axis`` vector specifies the rotation axis. The ``angle`` scalar specifies how many degrees of rotation must be applied at each iteration.

Optionally, a rotation ``center`` can be specified to rotate around a known point:

.. code-block:: xml

    <motion type="rotation" axis="x;y;z" angle="a" center="x;y;z"/>

Moreover, the ``autoCRS`` attribute can be used to automatically translate the rotation center to the internal reference system of the simulation:

.. code-block:: xml

    <motion type="rotation" axis="x;y;z" angle="a" center="x;y;z" autoCRS="1" />


To position the rotation center to the object center, use the ``selfMode`` argument:

.. code-block:: xml

    <motion type="rotation" axis="x;y;z" angle="a" selfMode="true" />

A **helical motion** can be defined as:

.. code-block:: xml

    <motion type="helical" axis="x;y;z" angle="a" glide="k"/>

The ``axis`` vector specifies the rotation axis which is the translation direction too. The ``angle`` scalar specifies how many degrees of rotation must be applied at each iteration. The ``glide`` scalar defines the magnitude of the translation so that after the rotation the ``p' = p + k (x,y,z)`` translation is applied to complete the helical motion.


A **rotational symmetry** can be defined as:

.. code-block:: xml

    <motion type="rotsym" axis="x;y;z" angle="a" center="p;q;r" />

The ``axis`` vector specifies the rotation axis which is also the orthogonal vector of the reflection plane. The ``angle`` scalar specifies how many degrees of rotation must be applied at each iteration. The ``center`` point specifies the center of the rotational symmetry, it is the point where the rotation axis and the reflection plane intersect.


Sequence definition
"""""""""""""""""""

As stated before, each rigid motion can be defined as a set of ``<motion ... />`` elements contained inside a ``<dmotion ...> ... </dmotion>``. Let us now look at how to define two different rigid movements.

The first rigid motion is a sequence of 4 motions describing a planetary like motion.

.. code-block:: xml

    <dmotion id ="planetary_motion" loop="0">
    <motion type="rotation" axis="0;0;1" angle="2" selfMode="true" />
    <motion type="translation" vec="-120;-100;0" />
    <motion type="rotation" axis="0;0;1" angle="1" />
    <motion type="translation" vec="120;100;0" />
    </dmotion>

The first rotation is configured with ``selfMode="true"`` to represent the rotation of a planet around the `z` axis centered in the object. It is followed by a translation, then a rotation and finally another translation that reverses the first. This last set of three motions is used to represent the translation of a planet which rotates around a given point. Thus, this dynamic motion defines a behavior similar to the one described by planet Earth, which rotates around itself at the same time that it moves around the sun. The entire sequence is computed inside an infinite loop, so it will never end.


The second rigid motion consists of two chained sequences of rigid motions. It represents a particle describing an ascendant helical motion followed by its corresponding descendant helical motion.

.. code-block:: xml

    <dmotion id="sphere_helical_up" loop="300" next="sphere_helical_down">
    <motion type="translation" vec="-129;-120;0" />
    <motion type="helical" axis="0;0;1" angle="4" glide="0.3"/>
    <motion type="translation" vec="129;120;0" />
    </dmotion>
    <dmotion id="sphere_helical_down" loop="300" next="sphere_helical_up">
    <motion type="translation" vec="-129;-120;0" />
    <motion type="helical" axis="0;0;1" angle="-4" glide="-0.3"/>
    <motion type="translation" vec="129;120;0" />
    </dmotion>


The ``"sphere_helical_down"`` sequence is repeated 300 times before proceeding to the ``"sphere_helical_up"`` motion. Once the second has been applied 300 times, the first one is executed again.
The aforementioned process is repeated until the simulation ends. Each motion starts with a translation and ends with another translation which reverts the first one. Thus, the translations can be seen as a way to centering the helical motion around an arbitrary point. The first helical motion is the ascending one, while the second helical motion is identical to the first but in the opposite direction.




Dynamic motion frequency
""""""""""""""""""""""""

By default, all rigid motions operate at the same frequency than the simulation. 
The operating frequency ``F`` for the simulation is defined by the pulse frequency of the scanner. This leads to rigid motions being computed ``F`` times per virtual second (seconds in simulated time, not real time seconds).
For comparison, typical FPS values for cameras easily range from ``25`` FPS for typical videos to ``960`` FPS for super slow motion videos.
This means that when using a pulse frequency of ``100000`` Hz and not setting a lower dynamic motion frequency explicitly, the scene movement is simulated at a rate that is 4000 and 100 times that of typical and super slow motion videos, respectively.

Thus, users are advised to individually control the frequency at which dynamic motions are computed, to find a suitable trade-off between temporal resolution and performance.
Both the frequency of the dynamic motion update and that of the KDTree update can be specified separately, specifying either a frequency (``dynStep``, ``kdtDynStep``) or a time step (``dynTimeStep``, ``kdtDynTimeStep``).

The first attribute is the ``dynStep="x"`` which means that the dynamic motion will be applied each ``x`` simulation steps.
Thus, if ``F=100000`` and ``dynStep="1000"`` it means that the dynamic motion will be applied ``F/dynStep = 100`` times per second.
Note that this might require to update values such as the magnitude of a translation because adding ``100`` times a translation vector ``(0,0,1)`` leads to an accumulated translation vector ``(0,0,100)`` while adding ``100000`` times the same translation vector leads to an accumulated translation vector ``(0,0,100000)``.
The dynamic time step exploits the inverse relationship between time and frequency to provide an alternative specification.
More concretely, the time step (inside :math:(0, 1]``) between iterations can be given instead of the discrete dynamic step, which is independent of the simulation frequency. This means that when changing the dynamic time step, rigid motion magnitudes do not need to be updated.

The second attribute is the ``kdtDynStep="y"`` which means that the KDGrove (a datastructure which handles multiple KDTrees for efficient ray casting on dynamic scenes) will update the KDTree for the dynamic object after ``y`` consecutive updates of the object itself. For instance, having ``kdtDynStep="10"`` means that the KDTree will be updated after computing ``10`` iterations of the dynamic motion sequence. 
The alternative time-based counterpart ``kdtDynTimeStep="t"`` means that the KDTree will be updated after t seconds.

These frequency/temporal parameters can either be given at the scene level, which changes the default frequency (``dynStep="1"``) for all scene parts,
or at the scene part level, which changes the frequency for the specific scene part by multiplying the value with the scene default.
This means that the ``dynStep`` of the object is relative to the ``dynStep`` of the scene, and the ``kdtDynStep`` of the KDT is relative to the ``dynStep`` of the object.

Having ``dynStep="z"`` at the scene element and ``dynStep="x"`` at the scene part element means that the dynamic object will be updated each ``xz`` iterations.
Also, having ``dynStep="z"`` at the scene element with default ``dynStep="1"`` for all dynamic objects works similar to having ``dynStep="z"`` defined for each dynamic object while having default ``dynStep="1"`` for the scene.

An example of how to configure the different frequencies for scene is presented below:

.. code-block:: xml

    <scene id="dyn_cube_scene" name="DynCubeScene" dynStep="20">

    <!--  Ground plane as a static object -->
    <part>
        <filter type="objloader">
        <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
        </filter>
        <filter type="scale">
        <param type="double" key="scale" value="120" />
        </filter>
        <filter type="translate">
        <param type="vec3" key="offset" value="50.0;0;0" />
        </filter>
    </part>

    <!--  Cube as a dynamic object -->
    <part dynStep="5" kdtDynStep="10">
        <filter type="objloader">
        <param type="string" key="filepath" value="data/sceneparts/toyblocks/cube.obj" />
        </filter>
        <filter type="rotate">
        <param key="rotation" type="rotation">
            <rot angle_deg="45" axis="z"/>
        </param>
        </filter>
        <filter type="scale">
        <param type="double" key="scale" value="0.75" />
        </filter>
        <filter type="translate">
        <param type="vec3" key="offset" value="-40.0;-5.0;0" />
        </filter>
        <!-- The dynamic motion sequence for the cube -->
        <dmotion id="cube_translation" loop="0">
        <motion type="translation" vec="0.001;-0.003;0"/>
        </dmotion>
    </part>

    </scene>

In the above example, the frequency for the dynamic scene is set to ``20`` while the frequency for the dynamic object is set to ``5``, which means that the dynamic object is updated each ``20 x 5 = 100`` simulation steps.
The frequency for KDTree updates is ``10``, which means the KDTree is updated after ``10`` dynamic object updates.
In this case, the dynamic object has an infinite sequence that repeats the same translation. Thus, it is clear that the KDTree will be updated each ``1000`` simulation steps.
Assuming a pulse frequency of ``100000``, the dynamic moving object will apply the translation vector ``1000`` times per virtual second while the KDTree is updated ``100`` times per virtual second.

The XML below yields the same simulation as the previous example but uses a time-based specification instead.

.. code-block:: xml

    <scene id="dyn_cube_scene" name="DynCubeScene" dynTimeStep="0.0002">

    <!--  Ground plane as a static object -->
    <part>
        <filter type="objloader">
        <param type="string" key="filepath" value="data/sceneparts/basic/groundplane/groundplane.obj" />
        </filter>
        <filter type="scale">
        <param type="double" key="scale" value="120" />
        </filter>
        <filter type="translate">
        <param type="vec3" key="offset" value="50.0;0;0" />
        </filter>
    </part>

    <!--  Cube as a dynamic object -->
    <part dynTimeStep="0.001" kdtDynTimeStep="0.01">
        <filter type="objloader">
        <param type="string" key="filepath" value="data/sceneparts/toyblocks/cube.obj" />
        </filter>
        <filter type="rotate">
        <param key="rotation" type="rotation">
            <rot angle_deg="45" axis="z"/>
        </param>
        </filter>
        <filter type="scale">
        <param type="double" key="scale" value="0.75" />
        </filter>
        <filter type="translate">
        <param type="vec3" key="offset" value="-40.0;-5.0;0" />
        </filter>
        <!-- The dynamic motion sequence for the cube -->
        <dmotion id="cube_translation" loop="0">
        <motion type="translation" vec="0.001;-0.003;0"/>
        </dmotion>
    </part>

    </scene>

As stated before, the ``dynTimeStep`` and ``kdtDynTimeStep`` are given as direct time measurements.
Thus, when using the time-based specification, the ``dynTimeStep`` of the scene must be smaller than or equal to the ``dynTimeStep`` of each scene part (object), and the ``kdtDynTimeStep`` should be greater than or equal to the ``dynTimeStep`` of the associated object.
The reason is that they represent nested components. Intuitively, updating a KDT representing an object 100 times per second does not make sense when the object is only updated five times per second. Furthermore, values greater than one are not expected.
The reason is that the dynamic time steps are the inverse of a frequency that represents the number of iterations per second in the main simulation loop. 

The dynamic time steps are straightforward to interpret. A dynamic scene with ``dynTimeStep="0.0002"`` will compute its dynamic simulation logic with a time step of 0.2 ms, a dynamic object with ``dynTimeStep="0.001"`` will run its logic with a time step of 1 ms, and a dynamic KDT with ``dynTimeStep="0.01"`` will be updated each 10 ms. When not given, the dynamic time step of the KDT is automatically equal to the dynamic time step of its associated object.
Similarly, when the dynamic time step of an object is not given, it is automatically equal to the dynamic time step of the scene. Dynamic steps (discrete) and dynamic time steps (continuous) cannot be mixed in the same specification.


Examples
""""""""

Multiple examples for different rigid motions can be found in the `example scene for moving toyblocks`_ and in the `dyn scene folder`_.

See also our notebooks:

- :doc:`14 - MLS Dynamic Scene Demo <14-urban_mls_dynamic>`
- :doc:`15 - TLS Rigid Motion Demo <15-tls_tree_dynamic>`

.. _example scene for moving toyblocks: https://github.com/3dgeo-heidelberg/helios/blob/main/data/scenes/toyblocks/moving_toyblocks_scene.xml
.. _dyn scene folder: https://github.com/3dgeo-heidelberg/helios/tree/main/data/scenes/dyn
