from helios.utils import classonlymethod, get_asset_directories
from helios.validation import (
    Angle,
    AngleVelocity,
    AssetPath,
    Frequency,
    Length,
    Model,
    TimeIntervalNanoSeconds,
    Power,
    R3Vector,
    ScanFrequency,
    Visibility,
    units,
    UpdateableMixin,
    TimeInterval,
    validate_xml_file,
)
from helios.rotation import Rotation
from helios.settings import FullWaveformSettings
from pydantic import validate_call
from pydantic import Field
from typing import Annotated, Callable

import _helios
import math
import numpy as np


class ScannerSettingsBase(Model, UpdateableMixin, cpp_class=_helios.ScannerSettings):
    def _resolve_for_scanner(self, scanner):
        provided_fields = getattr(self, "_provided_fields", set())
        supported = list(scanner._cpp_object.supported_pulse_freqs_hz)
        if "pulse_frequency" not in provided_fields:
            if supported and self.pulse_frequency not in supported:
                self.pulse_frequency = supported[
                    0
                ]  # TODO add here log message after python log module implementation


class ScannerSettings(ScannerSettingsBase):
    """
    Class representing the settings for a scanner.
    These can be set for each leg of a survey.

    :param is_active: Whether the scanner is active during the leg. If false, no points will be recorded during this leg.
    :param head_rotation: The rotation speed of the scanner head in radians per second. For scanners with rotating head.
    :param rotation_start_angle: The start angle of the scanner head rotation in radians. For scanners with rotating head.
    :param rotation_stop_angle: The stop angle of the scanner head rotation in radians. For scanners with rotating head.
    :param pulse_frequency: The pulse frequency of the scanner in Hz.
    :param scan_angle: The scan angle of the scanner in radians. May be ignored for some scanner types (e.g. optics = risley).
    :param min_vertical_angle: The minimum vertical angle of the scanner in radians. To be used to define the scan angle for TLS scanners (with non-symmetrical field of view).
    :param max_vertical_angle: The maximum vertical angle of the scanner in radians. To be used to define the scan angle for TLS scanners (with non-symmetrical field of view).
    :param scan_frequency: The scan frequency of the scanner in Hz.
    :param beam_divergence_angle: The beam divergence angle of the scanner in radians.
    :param trajectory_time_interval: The time interval in seconds at which the trajectory points are written.
    :param vertical_resolution: The vertical resolution of the scanner in radians. To be used for TLS with rotating optics and rotating head.
    :param horizontal_resolution: The horizontal resolution of the scanner in radians. To be used for TLS with rotating optics and rotating head.
    :param max_duration: The maximum simulated duration of the leg in seconds.
    :param optics_warmup_phase: The simulated optics warmup phase in seconds before the first pulse.
    :type is_active: bool
    :type head_rotation: AngleVelocity
    :type rotation_start_angle: Angle
    :type rotation_stop_angle: Angle
    :type pulse_frequency: Frequency
    :type scan_angle: Angle
    :type min_vertical_angle: Angle
    :type max_vertical_angle: Angle
    :type scan_frequency: Frequency
    :type beam_divergence_angle: Angle
    :type trajectory_time_interval: TimeInterval
    :type vertical_resolution: Angle
    :type horizontal_resolution: Angle
    :type max_duration: TimeInterval
    :type optics_warmup_phase: TimeInterval
    """

    _track_explicit_fields = True
    is_active: bool = True
    head_rotation: AngleVelocity = 0
    rotation_start_angle: Angle = 0
    rotation_stop_angle: Angle = 0
    pulse_frequency: Frequency = 300000
    scan_angle: Angle = 0
    min_vertical_angle: Angle = np.nan
    max_vertical_angle: Angle = np.nan
    scan_frequency: ScanFrequency = 200
    beam_divergence_angle: Angle = 0.003 * units.rad
    trajectory_time_interval: TimeInterval = 0.01
    vertical_resolution: Angle = 0
    horizontal_resolution: Angle = 0
    max_duration: TimeInterval = 0
    optics_warmup_phase: TimeInterval = 0


# TODO: Requires expert input
class RotatingOpticsScannerSettings(ScannerSettingsBase):
    is_active: bool = True
    head_rotation: AngleVelocity = 0
    rotation_start_angle: Angle = 0
    rotation_stop_angle: Angle = 0


# TODO: Requires expert input
class OscillatingOpticsScannerSettings(ScannerSettingsBase):
    pass


# TODO: Requires expert input
class LineOpticsScannerSettings(ScannerSettingsBase):
    pass


# TODO: Requires expert input
class ConicOpticsScannerSettings(ScannerSettingsBase):
    pass


# TODO: Requires expert input
class RisleyOpticsScannerSettings(ScannerSettingsBase):
    pass


class BeamDeflector(Model, cpp_class=_helios.AbstractBeamDeflector):
    scan_angle_max: Angle = 0.0
    scan_frequency_min: ScanFrequency = 0.0
    scan_frequency_max: ScanFrequency = 0.0

    @classmethod
    def _from_cpp(cls, value):
        if cls is BeamDeflector:
            if isinstance(value, _helios.PolygonMirrorBeamDeflector):
                return RotatingOptics._from_cpp(value)
            if isinstance(value, _helios.OscillatingMirrorBeamDeflector):
                return OscillatingOptics._from_cpp(value)
            if isinstance(value, _helios.ConicBeamDeflector):
                return ConicOptics._from_cpp(value)
            if isinstance(value, _helios.FiberArrayBeamDeflector):
                return LineOptics._from_cpp(value)
            if isinstance(value, _helios.RisleyBeamDeflector):
                return RisleyOptics._from_cpp(value)
        return Model._from_cpp.__func__(cls, value)


class OscillatingOptics(
    BeamDeflector, cpp_class=_helios.OscillatingMirrorBeamDeflector
):
    scan_product: Annotated[int, Field(gt=0)] = 1_000_000


class ConicOptics(BeamDeflector, cpp_class=_helios.ConicBeamDeflector):
    pass


class LineOptics(BeamDeflector, cpp_class=_helios.FiberArrayBeamDeflector):
    number_of_fibers: Annotated[int, Field(gt=0)] = 1


class RotatingOptics(BeamDeflector, cpp_class=_helios.PolygonMirrorBeamDeflector):
    scan_angle_effective_max: Angle = 0.0


class Prism(Model, cpp_class=_helios.Prism):
    angle: Angle
    inclined_on_left: bool
    refractive_index: Annotated[float, Field(gt=0)]
    rotation_speed: AngleVelocity


class RisleyOptics(Model, cpp_class=_helios.RisleyBeamDeflector):
    prisms: tuple[Prism, ...]
    air_refractive_index: Annotated[float, Field(gt=0)] = 1.0
    incident_beam: R3Vector = (0.0, 1.0, 0.0)


Optics = OscillatingOptics | ConicOptics | LineOptics | RotatingOptics | RisleyOptics


class Detector(Model, cpp_class=_helios.FullWaveformPulseDetector):
    accuracy: Length = 0.0
    range_min: Length = 0.0
    range_max: Length = math.inf


class ScannerHead(Model, cpp_class=_helios.ScannerHead):
    rotation_axis: R3Vector = (0.0, 0.0, 1.0)
    rotation_speed_max: AngleVelocity = 0.0


class ScanningDevice(Model, cpp_class=_helios.ScanningDevice):
    id: str
    supported_pulse_frequencies: tuple[Frequency, ...]
    optics: Optics

    beam_origin_position: R3Vector = (0.0, 0.0, 0.0)
    beam_origin_attitude: Rotation = Rotation()
    beam_divergence: Angle = 0.0003
    pulse_length: TimeIntervalNanoSeconds = 4.0
    wavelength: Length = 1064.0 * units.nm
    average_power: Power = 4.0
    beam_quality_factor: Annotated[float, Field(gt=0)] = 1.0
    optical_efficiency: Annotated[float, Field(ge=0, le=1)] = 0.99
    receiver_diameter: Length = 0.15
    atmospheric_visibility: Visibility = 23.0
    received_energy_min: Power = 0.0001
    max_returns: Annotated[int, Field(ge=0)] = 0

    detector: Detector = Detector()
    head: ScannerHead = ScannerHead()
    full_waveform: FullWaveformSettings = FullWaveformSettings()

    def _post_set(self, field):
        if field == "full_waveform" and self.full_waveform.window_size is None:
            self.full_waveform.window_size = self.pulse_length / 4.0 * units.ns

    @classmethod
    def _from_cpp(cls, value):
        obj = Model._from_cpp.__func__(cls, value)
        obj._optics = BeamDeflector._from_cpp(value.optics)
        return obj


class Scanner(Model, cpp_class=_helios.Scanner):
    id: str
    name: str = ""

    def _never_serialize_shallow(self) -> bool:
        return True

    @classmethod
    def _from_cpp(cls, value):
        if cls is Scanner:
            if isinstance(value, _helios.SingleScanner):
                return SingleScanner._from_cpp(value)
            if isinstance(value, _helios.MultiScanner):
                return MultiScanner._from_cpp(value)
        return Model._from_cpp.__func__(cls, value)

    @classonlymethod
    @validate_call
    def from_xml(cls, scanner_file: AssetPath, scanner_id: str = ""):
        """Classmethod to load a scanner from an XML file. The XML file should conform to the schema defined in "xsd/scanner.xsd". The scanner_id parameter can be used to specify which scanner to load if the XML file contains multiple scanners. The method validates the XML file against the schema before loading the scanner.

        :param scanner_file: The path to the XML file containing the scanner definition.
        :param scanner_id: The ID of the scanner to load from the XML file.
        :type scanner_file: AssetPath
        :type scanner_id: str
        """

        # Validate the XML
        validate_xml_file(scanner_file, "xsd/scanner.xsd")

        _cpp_scanner = _helios.read_scanner_from_xml(
            str(scanner_file), [str(p) for p in get_asset_directories()], scanner_id
        )
        scanner = cls._from_cpp(_cpp_scanner)
        scanner._is_loaded_from_xml = True
        scanner._disable_yaml_serialization_for_descendants()
        scanner._set_constructor_provenance(
            "from_xml",
            "scanner_file",
            scanner_file=scanner_file,
            scanner_id=scanner_id,
        )
        return scanner


class SingleScanner(Scanner, cpp_class=_helios.SingleScanner):
    device: ScanningDevice


class MultiScanner(Scanner, cpp_class=_helios.MultiScanner):
    channels: tuple[ScanningDevice, ...]


#
# Predefined scanners
#

SCANNER_REGISTRY: dict[str, tuple[str, str]] = {
    # ALS Scanners
    "leica_als50": ("data/scanners_als.xml", "leica_als50"),
    "leica_als50_ii": ("data/scanners_als.xml", "leica_als50-ii"),
    "optech_2033": ("data/scanners_als.xml", "optech_2033"),
    "optech_3100": ("data/scanners_als.xml", "optech_3100"),
    "optech_galaxy": ("data/scanners_als.xml", "optech_galaxy"),
    "riegl_lms_q560": ("data/scanners_als.xml", "riegl_lms-q560"),
    "riegl_lms_q780": ("data/scanners_als.xml", "riegl_lms-q780"),
    "riegl_vq_780i": ("data/scanners_als.xml", "riegl_vq_780i"),
    "riegl_vux_1uav": ("data/scanners_als.xml", "riegl_vux-1uav"),
    "riegl_vux_1uav22": ("data/scanners_als.xml", "riegl_vux-1uav22"),
    "riegl_vux_1ha22": ("data/scanners_als.xml", "riegl_vux-1ha22"),
    "riegl_vq_880g": ("data/scanners_als.xml", "riegl_vq-880g"),
    "riegl_vq_1560i": ("data/scanners_als.xml", "riegl_vq-1560i"),
    "livox_mid70_als": ("data/scanners_als.xml", "livox_mid-70"),
    "livox_mid100_als": ("data/scanners_als.xml", "livox-mid-100"),
    "dji_zenmuse_l2_repetitive": ("data/scanners_als.xml", "dji-zenmuse-l2-repetitive"),
    "dji_zenmuse_l2_non_repetitive": (
        "data/scanners_als.xml",
        "dji-zenmuse-l2-non-repetitive",
    ),
    # TLS Scanners
    "riegl_vz_400": ("data/scanners_tls.xml", "riegl_vz400"),
    "riegl_vz_1000": ("data/scanners_tls.xml", "riegl_vz1000"),
    "riegl_vz_2000i": ("data/scanners_tls.xml", "riegl_vz2000i"),
    "riegl_vz_600i": ("data/scanners_tls.xml", "riegl_vz600i"),
    "riegl_vq_450": ("data/scanners_tls.xml", "riegl_vq-450"),
    "livox_mid40": ("data/scanners_tls.xml", "livox_mid-40"),
    "livox_mid100": ("data/scanners_tls.xml", "livox_mid-100"),
    "livox_mid70": ("data/scanners_tls.xml", "livox_mid-70"),
    "vlp16": ("data/scanners_tls.xml", "vlp16"),
    "velodyne_hdl_64e": ("data/scanners_tls.xml", "velodyne_hdl-64e"),
    "tractor_scanner": ("data/scanners_tls.xml", "tractorscanner"),
    "pano_scanner": ("data/scanners_tls.xml", "panoscanner"),
}


def list_scanners() -> list[str]:
    """List all predefined scanner names.

    :return: A list of all predefined scanner names that can be used to create scanners with `scanner_from_name()`.
    :rtype: list[str]
    """
    return list(SCANNER_REGISTRY.keys())


@validate_call
def scanner_from_name(scanner_name: str) -> Scanner:
    """Create a predefined scanner by its string name."""
    try:
        scanner_file, scanner_id = SCANNER_REGISTRY[scanner_name]
    except KeyError as exc:
        valid_names = ", ".join(list_scanners())
        raise ValueError(
            f"Unknown scanner '{scanner_name}'. Available scanners: {valid_names}"
        ) from exc
    return Scanner.from_xml(scanner_file, scanner_id=scanner_id)


def _make_predefined_scanner(scanner_name: str) -> Callable[[], Scanner]:
    def _scanner_factory():
        return scanner_from_name(scanner_name)

    _scanner_factory.__name__ = scanner_name
    _scanner_factory.__qualname__ = scanner_name
    _scanner_factory.__doc__ = f"Create predefined scanner '{scanner_name}'."
    return _scanner_factory


for _scanner_name in SCANNER_REGISTRY:
    globals()[_scanner_name] = _make_predefined_scanner(_scanner_name)

del _scanner_name
