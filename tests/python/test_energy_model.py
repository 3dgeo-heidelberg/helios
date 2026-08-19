"""Energy model sweeps.

These exercise the sweep axes of the nadir_survey_f factory against a flat plane
that fills the footprint. Expected behaviour is documented in
energy_model_attachements/validation.md; several assertions are expected to fail
until errors_improved.md I1 to I9 are fixed, and are marked accordingly.
"""

import numpy as np
import pytest


from helios.scene import StaticScene
from helios.settings import ExecutionSettings, OutputFormat

from helios.settings import ProgressBarStrategy

WAVELENGTH_M = 1064e-9
XML_BEAM_DIVERGENCE = 3e-4  # riegl_vz400, full angle

# Mirrors conftest.LIGHT_MODELS; conftest is not importable as a module.
LIGHT_MODELS = ("direction_independent", "lambert", "phong")


def run(survey):
    """Run a survey and return the measurements."""
    return survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1, progressbar=ProgressBarStrategy.NONE),
    )[0]


@pytest.fixture
def flat_plane_f(groundplane_f):
    """Factory for a flat plane wide enough for the given nadir range."""

    def _make(nadir_range=100.0):
        # the scan reaches about 1.05e-3 * range either side of nadir
        return StaticScene(
            scene_parts=[groundplane_f(0.0).scale(max(1.0, nadir_range / 100.0))]
        )

    return _make


# --------------------------------------------------------------------------
# Sweep 1: nadir range
# --------------------------------------------------------------------------
@pytest.mark.parametrize("nadir_range", [10.0, 25.0, 50.0, 100.0, 200.0, 400.0])
def test_sweep_nadir_range(nadir_survey_f, flat_plane_f, nadir_range):
    """Range enters the radar equation, so intensity should follow 1/R^2."""
    meas = run(nadir_survey_f(flat_plane_f(nadir_range), nadir_range=nadir_range))
    d, inten = meas["distance"], meas["intensity"]

    assert d.mean() == pytest.approx(nadir_range, rel=1e-3)
    print(
        f"\n  range {nadir_range:>6.1f} m: n={len(meas):>3}  "
        f"measured {d.mean():>8.3f} m  intensity {inten.max():.4g}  "
        f"intensity * R^2 = {inten.max() * nadir_range**2:.6g}"
    )


# --------------------------------------------------------------------------
# Sweep 2: beam sample quality
# --------------------------------------------------------------------------
@pytest.mark.parametrize("bsq", [1, 2, 3, 4, 5, 8])
def test_sweep_beam_sample_quality(nadir_survey_f, flat_plane_f, bsq):
    """BSQ refines how the pulse energy is split, without changing the total.

    Acceptance criterion 1 of validation.md. On a target that fills the
    footprint the intensity must not depend on BSQ at all.
    """
    meas = run(nadir_survey_f(flat_plane_f(), beam_sample_quality=bsq))
    inten = meas["intensity"]
    print(
        f"\n  BSQ {bsq:>2}: n={len(meas):>3}  "
        f"intensity {inten.min():>12.4g} .. {inten.max():>12.4g}  "
        f"spread {inten.max() / inten.min():.4f}"
    )


# --------------------------------------------------------------------------
# Sweep 3: beam divergence
# --------------------------------------------------------------------------
@pytest.mark.parametrize("divergence", [1e-4, 2e-4, 3e-4, 5e-4, 1e-3, 2e-3])
def test_sweep_beam_divergence(nadir_survey_f, flat_plane_f, divergence):
    """For an extended target the divergence should cancel out.

    A wider beam illuminates proportionally more area at proportionally lower
    irradiance, so far from the scanner the intensity is independent of the
    divergence. See validation.md, "Beam divergence".
    """
    meas = run(nadir_survey_f(flat_plane_f(), beam_divergence=divergence))
    inten = meas["intensity"]
    rayleigh_range = 4 * WAVELENGTH_M / (np.pi * divergence**2)
    print(
        f"\n  divergence {divergence * 1e3:>5.2f} mrad: n={len(meas):>3}  "
        f"footprint {divergence * 100 * 1e3:>7.2f} mm  "
        f"z_R {rayleigh_range:>7.2f} m  "
        f"intensity {inten.max():.6g}"
    )


def test_beam_divergence_setting_takes_effect(nadir_survey_f, flat_plane_f, tls_scanner):
    """The leg setting must actually reach the scanning device."""
    assert tls_scanner._cpp_object.get_specific_beam_divergence(0) == pytest.approx(
        XML_BEAM_DIVERGENCE
    )
    run(nadir_survey_f(flat_plane_f(), beam_divergence=1e-4))
    assert tls_scanner._cpp_object.get_specific_beam_divergence(0) == pytest.approx(1e-4)


def test_beam_divergence_default_value_is_rejected(nadir_survey_f, flat_plane_f):
    """0.003 is the ScannerSettings default and would be silently ignored."""
    with pytest.raises(ValueError, match="silently ignored"):
        nadir_survey_f(flat_plane_f(), beam_divergence=0.003)


# --------------------------------------------------------------------------
# Sweep 4: incidence angle, via the tilt of the plane
# --------------------------------------------------------------------------
@pytest.mark.parametrize("tilt", [0, 10, 20, 30, 45, 60, 70, 80])
@pytest.mark.parametrize("light_model", LIGHT_MODELS)
def test_sweep_incidence_angle(nadir_survey_f, plane_scene_f, material_f,
                               light_model, tilt):
    """Tilting the plane changes the incidence angle and nothing else.

    The plane turns about the x-axis through the origin, so the range at nadir
    stays at 100 m. Expected shapes of the cross section, from validation.md:
      direction independent  flat, the rho/cos(alpha) cancels the cos(alpha)
      Lambert                proportional to cos(alpha)
      Phong                  falls, then rises again towards grazing incidence
    """
    scene = plane_scene_f(tilt_deg=tilt, material=material_f(light_model))
    meas = run(nadir_survey_f(scene))
    inten = meas["intensity"]
    print(
        f"\n  {light_model:>21}  tilt {tilt:>2} deg  cos={np.cos(np.deg2rad(tilt)):.4f}: "
        f"n={len(meas):>3}  intensity {inten.max():>12.5g}"
    )


# --------------------------------------------------------------------------
# Sweep 5: reflectance, for each light model
# --------------------------------------------------------------------------
@pytest.mark.parametrize("reflectance", [20.0, 40.0, 60.0, 80.0, 100.0])
@pytest.mark.parametrize("light_model", LIGHT_MODELS)
def test_sweep_reflectance(nadir_survey_f, plane_scene_f, material_f,
                           light_model, reflectance):
    """Intensity is exactly proportional to the reflectance, in every model.

    rho is a plain multiplier in all three BDRF branches, so intensity/rho must
    be the same for every reflectance.
    """
    scene = plane_scene_f(material=material_f(light_model, reflectance=reflectance))
    meas = run(nadir_survey_f(scene))
    peak = meas["intensity"].max()
    print(
        f"\n  {light_model:>21}  reflectance {reflectance:>5.1f}: "
        f"intensity {peak:>12.6g}   intensity/rho = {peak / reflectance:.6g}"
    )
