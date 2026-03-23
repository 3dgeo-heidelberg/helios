Error sources and randomness control
====================================

Error sources
--------------

HELIOS++ supports different sources of error in geometry and radiometry.
They correspond to simple models of reality, where noise is introduced at *platform*, *leg* and *ranging* levels. These noise types influence the geometry of the resulting point cloud in systematic or random fashion.

Additionally, *fullwave* noise adds noise to the bins of the fullwave measurements prior to export. It can be enabled by using the ``--fullwaveNoise`` CLI parameter.

Ranging noise
^^^^^^^^^^^^^^

Ranging measurements are subject to noise due to detector accuracy, resolution and sensitivity.
Ranging error is applied by sampling noise from a normal distribution with a mean of 0 and a standard deviation defined by the ``accuracy_m`` attribute in the scanner XML file.
The sampled noise is then added to the true range value to obtain the final range measurement.
If no ``accuracy_m`` is specified, a default value of 0.003 m is used.

For more complex noise models that depend on range and incidence angle, measurement error can also be defined using a univariate expression.
The univariate expression is defined in the survey XML in a ``distanceMeasurementError`` attribute within the ``detectorSettings`` tag.
These expressions are expected as infix expressions (infix notation).


Fullwave noise
^^^^^^^^^^^^^^


Platform noise
^^^^^^^^^^^^^^

Platform noise concerns the position and the attitude, and is specified in the platform XML file.

.. code-block:: xml

    <positionXNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.02"/>
	<positionYNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.02"/>
	<positionZNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.02"/>
	<attitudeXNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.001"/>
	<attitudeYNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.001"/>
	<attitudeZNoise
		clipMin="0.0" clipMax="0.0" clipEnabled="false" fixedLifespan="1"
		type="NORMAL" mean="0.0" stdev="0.001"/>

It can be sampled from a normal distribution (``type="NORMAL"``) with a given ``mean`` and standard deviation (``stdev``) or from a uniform distribution (``type="UNIFORM"``) with a given ``min`` and ``max``.

It is possible to clip sampled noise values by setting ``clipEnabled="true"`` inside the noise tag. The clip values can then be specified using ``clipMin`` and ``clipMax``.

If ``fixedLifespan=="0"``, a computed noise value will be computed one time and used for consecutive noise values.
If ``fixedLifespan="1"``, the value does not behave like a fixed value and will be renewed between each pair of consecutive uses.
Having fixed enabled (``fixedLifespan > "1"``) means a noise value will be computed one time and whenever the next noise value is requested the previously computed fixed value will be returned, as long as its lifespan has not expired yet.

To disable platform noise even if specified in the XML file, use the ``--disablePlatformNoise`` flag when running a simulation with the CLI.

Controlling randomness
----------------------

In order to reproduce simulation runs, (pseudo)random numbers are controlled using seeds.

A seed can be specified using the function ``helios.util.set_rng_seed()`` or with the CLI argument: ``--seed <seed>``.
If no seed is specified, the simulation is using a randomly computed seed.

The seed can be an integer or a datetime.

Despite controlling the seed for random numbers, the concurrent nature of the simulation still generates output with no guaranteed order. 
By **limiting the number of jobs to 1**, the order is guaranteed and the output should be exactly the same for different executions. 
The number of jobs can be specified using ``-j`` or the equivalents ``--njobs`` or ``--nthreads``. If no ``-j`` argument is specified or if it is set to 0, all possible threads are used.