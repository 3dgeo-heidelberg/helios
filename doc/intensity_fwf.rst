
Full waveform and intensity modelling
=====================================


Radiometry calculation API
-------------------------

``EnergyModel`` and scanner calculation methods use incidence cosine and ranges in metres. ``computeReceivedPower`` and ``computeEmittedPower`` return watts. ``computePowerRatio`` returns received power divided by whole-beam emitted peak power, including the absolute subray share. ``WithSigma`` variants accept the existing external scattering value, including the LadLut convention. ``computeTargetArea`` returns the transverse subray patch area in square metres. Python exposes the corresponding snake_case names and keyword arguments::

   power_W = model.compute_received_power(
       incidence_cosine=math.cos(angle_rad), target_range_m=100.0,
       material=material, subray_index=0)
   area_m2 = model.compute_target_area(target_range_m=100.0, subray_index=0)

Migration from the previous API requires replacing argument objects with direct arguments, passing cosine instead of incidence angle, and using ordinary range instead of squared range for target area. The old intensity/transfer names and ``compute_cross_section`` are removed. Received-power values are now watts, so their numerical values are the former values divided by ``1e9``.

Waveform and point-cloud output scaling remains unchanged. Conversion to the existing nanowatt scale occurs at waveform and intersection-handling boundaries. Analytics records retain their schema and watt-valued power columns. The BRDF column contains the angular reflectance response, including incidence cosine where applicable.


Fullwave processing
-------------------

HELIOS++ supports simulation of full waveform data by simulating a laser beam cone of finite divergence via sampled subrays.

Beam divergence
^^^^^^^^^^^^^^^

The scanner XML attribute ``beamDivergence_rad`` defines the full beam divergence in radians. Individual ``<channel>`` elements can override it. In Python, change the divergence before running a simulation:

   scanner.beam_divergence = 0.003  # radians, channel 0
   scanner.beam_divergence = "0.03 deg"  # angles with units are supported
   scanner.set_beam_divergence(0.0005, index=1)  # a multi-channel scanner
   angle = scanner.get_beam_divergence(index=1)

These setters refresh the derived beam state automatically. Divergence must be finite and strictly between zero and pi radians.

Subray quantification
^^^^^^^^^^^^^^^^^^^^^

The quality of the simulation can be adjusted by the ``beamSampleQuality`` attribute of the ``<FWFSettings [...]/>`` tag in the survey XML.
If no information is given there, the scanner definition may have a set of ``FWFSettings`` instead. If this is also not present, hardcoded default values are used, as shown in the following table:

.. list-table:: HELIOS++ Pulse Sampling and Timing Parameters
   :widths: 25 20 55
   :header-rows: 1

   * - **Attribute**
     - **Default Value**
     - **Comment**

   * - ``beamSampleQuality``
     - 3
     - 3 concentric circles of subrays, 19 subrays total; discretization in space.

   * - ``beamSamplingFactor``
     - 2
     - Sampling radius in units of the Gaussian 1/e^2 radius; captures 99.9665% of emitted power.

   * - ``sampleSpacing_m``
     - 0.03747405725
     - Fixed sample spacing in metres (equivalent to 0.25 ns round-trip time).

   * - ``maxRecordingSpan_m``
     - 0
     - Maximum recording span from its first sample; zero means unlimited.

   * - ``peakWindow_m``
     - 0.149896229
     - Peak validation neighborhood in metres; at least twice the sample spacing.

   * - ``pulseShape``
     - gamma
     - Analytic unit-peak shape: ``gamma`` or ``gaussian``.

   * - ``correctPeakOffset``
     - true
     - Subtract the shape's peak offset when reporting ranges.

   * - ``recordingTailTolerance``
     - 0.0001
     - Relative unit-peak amplitude for automatic recording bounds only.


The ``beamSampleQuality`` :math:`bSQ` is the number of concentric circles from which subrays are sampled. The angular distance of circle :math:`i` (counting from the center) depends on the beam divergence :math:`\beta` and is calculated as:

.. math::
   \theta_s = \arctan\left(k\tan(\beta/2)\right), \qquad
   \alpha_i = \frac{i\theta_s}{bSQ-1/2}, \quad 0 \le i < bSQ

Here :math:`\beta` is the full 1/e^2 divergence and :math:`k` is ``beamSamplingFactor``. Annular boundaries lie halfway between ring directions, starting at zero and ending exactly at :math:`\theta_s`. A single central ray represents the entire sampled disk when ``beamSampleQuality = 1``. Increasing coverage at fixed quality makes spatial sampling coarser.

On each circle, :math:`k` subrays are sampled, where:

.. math::
   N_0 = 1, \qquad N_i = \left\lfloor 2\pi i \right\rfloor \quad (i > 0)

The subrays are distributed evenly around the circle. The total number of subrays for a given ``beamSampleQuality`` :math:`bSQ` is:

.. math::
   n_{\text{Rays}} = 1 + \sum_{i=1}^{bSQ-1} \left\lfloor 2\pi i \right\rfloor

For example, with ``beamSampleQuality = 3``, the subray distribution appears as follows (color represents relative amplitude, see next section):

.. figure:: /img/polar_subsampling_grid.png
   :alt: Subray distribution for beamSampleQuality = 3
   :width: 60%
   :align: center

   Subray distribution.


.. _amplitude_evaluation:

Amplitude Evaluation
--------------------

For each subray, the returned waveform is computed by intersecting the ray with the scene. If a hit occurs, the ray does not propagate further.

The received amplitude is derived from the LiDAR equation, considering the following components:

1. **Transmitted power**
   Geometry and weights use one circular far-field Gaussian. At axial distance :math:`z`, the radius is :math:`w=z\tan(\beta/2)`. The power share :math:`s_{p,i}` assigned to each subray of annulus :math:`i` is its integrated fraction of total emitted power :math:`P_0`:

   .. math::
      s_{p,i} = \frac{1}{N_i}\left[\exp{\left(-\frac{2\tan^2(\theta_i)}{\tan^2(\beta/2)}\right)} - \exp{\left(-\frac{2\tan^2(\theta_{i+1})}{\tan^2(\beta/2)}\right)}\right].

   The shares sum to :math:`1-e^{-2k^2}`, independently of range and sample quality. No power from the omitted Gaussian tail is shifted into the sampled cone.

2. **Material reflectance**  
   The surface reflectance is modeled using Phong's Bidirectional Reflectance Distribution Function (BRDF) :cite:p:`Phong.1975`.

3. **Target cross section**  
   The effective cross section is computed based on the area illuminated by the subray and the local incidence angle.

4. **Atmospheric and system efficiency**  
   Includes atmospheric attenuation and system transmission losses.

See also: :ref:`intensity-modelling`.

Incidence evaluation uses the cosine directly during ray tracing. For normalized ray direction and surface normal, the smaller incidence angle has cosine :math:`c = |\mathbf{n}\cdot\mathbf{d}|`. The Phong angular reflectance response is computed without recovering the angle:

.. math::
   f = \rho\left[(1-K_s)c + K_s |2c^2-1|^{N_s}\right].

Angle-based APIs remain available. The cosine path also supports normal-based voxels and their closest-face fallback. Detailed analytics convert the cosine back to radians only when recording the incidence angle.

Analytic pulses in range coordinates
-----------------------------------

The implementation uses the abstract ``AnalyticPulse`` interface with ``GammaPulse`` and ``GaussianPulse`` implementations. Each scanning device holds an immutable prepared pulse selected by the factory. Waveform assembly uses the same interface for either shape.

Reconstruction and peak extraction use metres. The scanner's ``pulseLength_ns`` remains a temporal FWHM specification and is converted once using :math:`x=ct/2`. Subray returns retain their exact geometric ranges :math:`r_i` and dimensionless received/whole-beam peak-power ratios :math:`A_i`.

The default gamma pulse is

.. math::
   g(u)=\begin{cases}
   \frac{e^2}{4}(u/\rho)^2\exp(-u/\rho), & u\geq0,\\
   0, & u<0,
   \end{cases}
   \qquad \rho=\frac{cT_\mathrm{FWHM}}{2\times3.394680670846504}.

Its reference is the onset and its peak is at :math:`2\rho`. The alternative Gaussian is

.. math::
   g(u)=\exp\left[-\frac{(u-3\sigma_r)^2}{2\sigma_r^2}\right],
   \qquad \sigma_r=\frac{cT_\mathrm{FWHM}}{4\sqrt{2\ln2}}.

The Gaussian reference lies three standard deviations before its peak. It has no exact onset: its negative-offset tail is retained. Both shapes have unit analytic peak; equal FWHM and peak power do not imply equal pulse energy.

.. figure:: img/outgoing_waveform.png
   :alt: Time-domain pulse shape
   :align: center

Calculating the detector signal
------------------------------

At each range sample :math:`x_k`, HELIOS++ directly evaluates

.. math::
   S(x_k)=P_0\sum_i A_i g(x_k-r_i).

There is no histogram of return distances, pre-sampled template, integer-bin placement, or renormalization of sampled maxima. Every return contributes its full analytic shape throughout the recording interval, including contributions below the recording tolerance and beyond the former :math:`7\tau` cutoff. Samples retain the legacy nanowatt intensity scale.

For gamma, automatic bounds extend from the nearest return to the furthest return plus the falling-flank tolerance crossing. For Gaussian, the pulse-relative bounds are :math:`3\sigma_r\pm\sigma_r\sqrt{-2\ln\epsilon}`. These bounds are combined with the nearest and furthest return. The tolerance controls the recording interval, not an absolute error bound for the summed signal.

The grid is anchored at range zero; automatic bounds round outwards to grid points. ``maxRecordingSpan_m``, if positive, caps the interval from its first sample without including a sample beyond the cap. Distant returns increase sample count and memory, never the sample spacing. Allocation/index overflow raises an error rather than silently reducing resolution.


.. figure:: img/onevpeak.png
   :alt: Full waveform example with one peak
   :width: 100%
   :align: center

Peak extraction and timing reference
-----------------------------------

The existing neighborhood and amplitude-threshold checks operate on the summed samples. With ``correctPeakOffset=true``, a detected peak at :math:`x` produces range :math:`x-2\rho` for gamma or :math:`x-3\sigma_r` for Gaussian. Disabling correction reports :math:`x` directly. Correction precedes object attribution and measurement-error processing and never changes the waveform itself. Overlapping returns can shift the combined maximum even after this correction.

Waveform export retains its existing layout. Endpoint metadata is physical round-trip time in nanoseconds, calculated from the recorded range coordinates using :math:`t=2x/c`. It is unaffected by range correction. As before, export requires an accepted return, and optional waveform noise is added after point extraction.

.. figure:: img/threevpeaks.png
   :alt: Full waveform example with three peaks
   :width: 100%
   :align: center

Configuration migration
-----------------------

Python uses ``sample_spacing``, ``peak_window``, ``max_recording_span``,
``pulse_shape``, ``correct_peak_offset``, and ``recording_tail_tolerance``::

   settings = helios.FullWaveformSettings(
       sample_spacing="2 cm", peak_window="10 cm",
       pulse_shape="gaussian", correct_peak_offset=True,
       recording_tail_tolerance=1e-4,
   )

Old XML ``binSize_ns``, ``winSize_ns``, and ``maxFullwaveRange_ns`` inputs and Python constructor ``bin_size``, ``win_size``, and ``max_fullwave_range`` inputs are deprecated. They convert to metres using :math:`c/2`; specifying both names for one setting is an error. Old YAML documents migrate to these settings and new serialization writes only the range fields.

This replaces the former one-way time coordinate, peak-aligned templates, and pulse cutoff. Simulation outputs therefore intentionally change. The old ``prepare_discretization``, template-array, sample-count, peak-index, and ``calc_time_propagation`` APIs are removed. Use ``prepare_pulse``, ``get_pulse_model`` (evaluation and peak offset), and ``recording_grid`` instead. The historical ``receivedEnergyMin_W`` setting still compares directly with legacy intensity samples; its unit naming is not changed in this refactor.


Echo width determination
------------------------

If the command line parameter ``--calcEchoWidth`` was provided, a Gaussian curve is fitted to the waveform. The existing fitter reports the width parameter of ``exp(-((x-centre)/width)^2)``, equal to sqrt(2) times the Gaussian standard deviation. As shown in the following figure, the maximum of the Gaussian does not align with the maximum of the pulse shape. The detected sample maximum, with the optional shape offset correction, determines range. The fitted Gaussian centre does not replace it. For an isolated ideal return, sampling uncertainty is bounded by one sample spacing (about 3.75 cm at the default). The fitted sample-index width is converted to metres. The reported ``echo_width`` retains nanoseconds through conversion with :math:`2/c`.

.. figure:: img/echo_width.png
    :alt: 


.. _intensity-modelling:

Intensity modelling
--------------------

The laser return intensity values are calculated using the laser radar equation.

Currently, for its calculation HELIOS++ takes the following factors into account (wide table, use the scroll bar):

.. list-table:: Factors used for intensity modelling
   :widths: 20 35 20 25
   :header-rows: 1

   * - **Factor**
     - **Description**
     - **Provided by**
     - **Default Value**

   * - Pulse amplitude
     - Whole-beam peak optical power before footprint truncation (watts)
     - User: ``pulseAmplitude_W`` in **scanner XML**
     - 1.0 W (reference amplitude)

   * - Aperture size
     - Receiver aperture diameter (meters)
     - User: ``receiverDiameter_m`` in **scanner XML**
     - 0.15 m

   * - Atmosphere transmission
     - Visibility (km)
     - User: ``atmosphericVisibility_km`` in **scanner XML**
     - 23.0 km

   * - System transmission
     - Energy passed after system inefficiency (dimensionless)
     - User: ``opticalEfficiency`` in **scanner XML**
     - 0.99

   * - Beam divergence
     - Increase of the beam diameter with increasing range (radians)
     - User: ``beamDivergence_rad`` in **scanner XML**
     - 0.0003 rad

   * - Range
     - Distance to the target (meters)
     - ``HELIOS++``
     - —

   * - Angle of incidence
     - Angle of incidence to the target (radians)
     - ``HELIOS++``
     - —

   * - Area
     - Area of the target illuminated by the beam (square meters)
     - ``HELIOS++``
     - —

   * - Reflectance
     - Reflectance of the target surface (dimensionless)
     - User: ``helios_reflectance`` in material file (`.mtl`) or ASTER
     - 50% (dimensionless)

   * - Specularity
     - Specularity of the target surface.
       Calculated as: :math:`\sum k_d / \sum (k_d + k_s)`
     - User: ``ks`` and ``kd`` in material file (`.mtl`)
     - —

   * - Specular exponent
     - Controls sharpness of specular highlights (material property)
     - User: ``Ns`` in material file (`.mtl`)
     - —

.. _pulse_amplitude:

Whole-beam pulse amplitude
--------------------------

``pulseAmplitude_W`` (Python ``scanner.pulse_amplitude``) specifies the effective peak optical power of the **entire emitted beam**, not an individual subray. Users supply or calibrate this value for their operating conditions. HELIOS++ does not infer it from manufacturer average power, repetition rate, or pulse length. The default 1 W is a reference amplitude, not a scanner specification.

For amplitude :math:`P_0` and a dimensionless unit-peak temporal shape :math:`h`:

.. math::

   P_\mathrm{emit}(t) = P_0 h(t), \qquad E_\mathrm{pulse} = P_0 \int h(t)\,dt.

Time in the energy integral is in seconds. The current shape is :math:`h(t)=(e^2/4)(t/\tau)^2 e^{-t/\tau}`, with analytic maximum one at :math:`t=2\tau`. Sampling can miss this peak. Samples are not renormalized. Pulse duration and shape therefore affect pulse energy at fixed amplitude. The existing finite sampling window and timing convention remain unchanged.

Each subray receives peak power :math:`P_0 s_i`, where :math:`s_i` is its absolute Gaussian power fraction. At fixed footprint extent, the sum of these fractions is independent of subray count. With ``beamSamplingFactor`` :math:`f`, the captured fraction is :math:`1-e^{-2f^2}`. Power outside the sampling cone is not redistributed among the retained rays.

The energy model computes dimensionless transfer coefficients :math:`H_i`, including these subray fractions. Waveform construction then evaluates:

.. math::

   P_\mathrm{received}(t) = P_0 \sum_i H_i h(t-t_i).

Amplitude is applied before detection and fitting.

.. versionchanged:: 3.0

   ``averagePower_w`` is rejected. Use ``pulseAmplitude_W``. Python ``average_power`` becomes ``pulse_amplitude`` and C++ accessors become ``getPulseAmplitude`` / ``setPulseAmplitude`` (including indexed variants). To preserve a legacy waveform, set :math:`P_0=A_\mathrm{old}4/e^2`. The old default coefficient 4.0 corresponds to approximately 2.1653645318 W. Shipped scanner definitions explicitly use this migrated value. New definitions without an amplitude use 1 W. This conversion preserves simulated waveforms, not manufacturer average-power physics. Public emitted/received peak-power methods now use the new peak meaning.

   Custom scanner subclasses must implement ``calcTransferFromCosine`` and ``calcTransferWithSigma`` for tracing. These return dimensionless transfers, without amplitude or the legacy intensity scaling. The energy model exposes corresponding Python methods ``compute_transfer_from_cosine`` and ``compute_transfer_with_sigma``. Existing public intensity methods remain amplitude-scaled.

Material files
--------------

HELIOS++ supports two ways of defining materials:

1. Reading material properties from `MTL material library files`_. Following the standard, these files and their materials are linked to mesh faces using the ``mtllib`` and ``usemtl`` statements in the .OBJ file.
2. Modifying materials using the ``Material`` interface of the Python API.

Further explanations can be found here:

- Python API: `Scene - Material properties <scene.ipynb#Material-properties>`_
- Legacy XML definitions: :ref:`materials_cli`

The default material, in case no material file is provided, looks like this:

.. code-block::

    newmtl default  
    Ka 0.0 0.0 0.0  
    Kd 0.0 0.0 0.0  
    Ks 0.0 0.0 0.0  
    Ns 10  
    helios_classification 0  
    helios_reflectance 50
    helios_isGround true

.. _MTL material library files: http://paulbourke.net/dataformats/mtl/


HELIOS-specific parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^

There are four HELIOS-specific parameters, that can be added to material files by inserting respective lines in the material definitions.

* ``helios_reflectance``: This parameter specifies the reflectance of the target surface, e.g. ``helios_reflectance 70.8``. (default: 50.0)
* ``helios_spectra``: Instead of setting reflectance values of the materials by hand, they can be retrieved from the ECOSTRESS spectral library (formally ASTER spectral library). For each material, this library provides a file specifying its reflectance for different wavelengths in range [0, 100]. HELIOS includes a small set of these spectra in the ``python/helios/data/spectra`` folder, such as asphalt, grass, wood or gray and red shingles. More spectra can be downloaded from NASA's JPL or users can add their own spectra (resulting from spectrometer measurements). The spectra files contained in any asset directory can be referenced by their name, e.g. ``helios_spectra grass``. Now HELIOS++ will select the appropriate reflectance for the material given the wavelength of the laser. If the selected wavelength is not specified in the library (as they are discrete values) the reflectance will be interpolated. If no spectra or reflectance is specified in the .mtl, the reflectance will be set to the default and a warning will be displayed.
* ``helios_classification``: This parameter can be used to assign a classification to different sceneparts, e.g., according the ASPRS Standard (see LAS Specification p. 11), e.g. ``helios_classification 2`` for the ground. This classification will appear as an attribute of the respective simulated point reflected from that scenepart. (default: 0)
* ``helios_isGround``: Boolean parameter, which can have either values "false"/"0" or "true"/"1". If set to "true", geometries with this material are considered ground. This information is used for the ``onGround`` parameters, which can be used to translate sceneparts and platforms. (default: "true")

.. _heliospp_intensity_model:

Intensity calculation from material properties
----------------------------------------------

HELIOS++ intensity is based on Phong's Bidirectional Reflectance Distribution Function (BRDF) reflectance model coupled with the lidar-radar equation. The recorded intensity (signal amplitude) is calculated in three main steps:

1. **BRDF Calculation**

   The BRDF is computed from the material reflectance :math:`\rho` (either set via ``helios_reflectance`` in the `.mtl` file or derived from a `helios_spectra` and the scanner's wavelength) and the specularity :math:`\text{spec}` (defined by the material parameters ``kd`` :math:`(k_d)`, ``ks`` :math:`(k_s)`, and the specular exponent ``Ns`` :math:`(N_s)`). The incidence angle :math:`\varphi` is determined from the ray-object intersection.

   .. math::
      \text{BRDF}_r = \rho \cdot BRDF(\varphi, \text{spec}, N_s)

   where :math:`BRDF` follows the formulation by :cite:t:`JutziGross.2022`:

   .. math::
      BRDF(\varphi, \text{spec}, N_s) = (1 - \text{spec}) \cdot \cos(\varphi) + \text{spec} \cdot \left| \cos(2\varphi^*) \right|^{N_s}

   with :math:`\varphi^* = \varphi - \pi/2` if :math:`\varphi > \pi/2`, otherwise :math:`\varphi^* = \varphi`.

   The specularity :math:`\text{spec}` is computed as:

   .. math::
      \text{spec} = \frac{k_s[0] + k_s[1] + k_s[2]}{k_d[0] + k_d[1] + k_d[2] + k_s[0] + k_s[1] + k_s[2]}

   Please note:

   - If ``ks`` is 0, the material is fully diffuse and the BRDF simplifies to :math:`\text{BRDF}_r = \rho \cdot \cos(\varphi)` (Lambertian reflectance).
   - If both ``kd`` and ``ks`` are 0, the material reflectance is direction-independent and the BRDF simplifies to :math:`\text{BRDF}_r = \rho`.


2. **Lidar Cross Section Calculation**

   The lidar cross section :math:`\sigma` is calculated using the illuminated target area :math:`A`, following :cite:p:`Wagner.2010`, Eq. 14. It is assumed that each sub-ray either fully hits the target or does not hit at all. Partial hits are treated as full hits in intensity simulation.

   .. math::
      \sigma_i = 4\pi \cdot \text{BRDF}_r \cdot A_i

3. **Received Intensity via Lidar-Radar Equation**

   For a subray with absolute power fraction :math:`s_i`, the dimensionless
   received/whole-beam peak-power transfer is:

   .. math::
      H_i = \frac{\pi s_i D_r^2 \eta_{\mathrm{sys}}\,\mathrm{BRDF}_r
                  e^{-2Ra_e}}{4R^2}, \qquad I_i = P_0 H_i.

   For an externally supplied cross section, the corresponding expression is :math:`H_i=s_iD_r^2\eta_{\mathrm{sys}}\sigma_i e^{-2Ra_e}/(16A_iR^2)`. Here :math:`P_0` is whole-beam peak optical power, :math:`A_i` is the subray's transverse target area, :math:`D_r` is receiver diameter, :math:`R` is target range, and :math:`\eta_{\mathrm{sys}}` is system efficiency. The BRDF response above already includes the incidence cosine. It is not applied a second time. The atmospheric extinction coefficient :math:`a_e` in inverse metres uses the Kruse relation:

   .. math::
      a_e = \frac{3.91}{1000 V_M} \left( \frac{\lambda_{\mu\mathrm{m}}}{0.55} \right)^{-q}

   with the exponent :math:`q` defined piecewise:

   .. math::
      q = 
      \begin{cases}
        1.6, & \text{if } V_M > 50\ \text{km} \\
        1.3, & \text{if } 6\ \text{km} < V_M \le 50\ \text{km} \\
        0.585 \cdot V_M^{0.33}, & \text{otherwise}
      \end{cases}

   Here :math:`V_M` is the numerical visibility in kilometres and :math:`\lambda_{\mu\mathrm{m}}` is the numerical wavelength in micrometres. The scanner stores wavelength in metres and converts it to micrometres for this relation. The factor of 1000 converts extinction per kilometre to extinction per metre, matching the range :math:`R` in metres in the two-way atmospheric transmission :math:`\exp(-2 R a_e)`.

   The approximation is applied for wavelengths from 0.5 to 2.0 micrometres, including both endpoints. Outside this range, HELIOS logs a warning and assumes zero extinction (unit atmospheric transmission).

References
----------

.. bibliography:: refs.bib
   :style: apa_recent
   :filter: docname in docnames
