from helios.utils import classonlymethod, get_asset_directories
from helios.validation import (
    Angle,
    AngleVelocity,
    AssetPath,
    Frequency,
    Length,
    Model,
    Power,
    R3Vector,
    ScanFrequency,
    units,
    UpdateableMixin,
    TimeInterval,
    validate_xml_file,
)
from helios.rotation import Rotation
from helios.settings import FullWaveformSettings
from pydantic import Field, NonNegativeInt, PositiveFloat, PositiveInt, validate_call
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
    scan_product: PositiveInt = 1_000_000


class ConicOptics(BeamDeflector, cpp_class=_helios.ConicBeamDeflector):
    pass


class LineOptics(BeamDeflector, cpp_class=_helios.FiberArrayBeamDeflector):
    number_of_fibers: PositiveInt = 1


class RotatingOptics(BeamDeflector, cpp_class=_helios.PolygonMirrorBeamDeflector):
    scan_angle_effective_max: Angle = 0.0


class Prism(Model, cpp_class=_helios.Prism):
    angle: Angle
    inclined_on_left: bool
    refractive_index: PositiveFloat
    rotation_speed: AngleVelocity


class RisleyOptics(Model, cpp_class=_helios.RisleyBeamDeflector):
    prisms: tuple[Prism, ...]
    air_refractive_index: PositiveFloat = 1.0
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
    pulse_length: TimeInterval = 4.0 * units.ns
    wavelength: Length = 1064.0 * units.nm
    average_power: Power = 4.0
    beam_quality_factor: PositiveFloat = 1.0
    optical_efficiency: Annotated[float, Field(ge=0, le=1)] = 0.99
    receiver_diameter: Length = 0.15
    atmospheric_visibility: Length = 23_000.0
    received_energy_min: Power = 0.0001
    max_returns: NonNegativeInt = 0

    detector: Detector = Detector()
    head: ScannerHead = ScannerHead()
    full_waveform: FullWaveformSettings = FullWaveformSettings()

    def _post_set(self, field):
        if field == "full_waveform" and self.full_waveform.window_size is None:
            self.full_waveform.window_size = self.pulse_length / 4.0

    @classmethod
    def _from_cpp(cls, value):
        obj = Model._from_cpp.__func__(cls, value)
        obj._optics = BeamDeflector._from_cpp(value.optics)
        return obj


class Scanner(Model, cpp_class=_helios.Scanner):
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

ScannerFactory = Callable[[], Scanner]
SCANNER_REGISTRY: dict[str, ScannerFactory] = {}


def registered_scanner(name: str):
    """Register a predefined scanner factory under name."""

    def decorator(factory: ScannerFactory) -> ScannerFactory:
        if name in SCANNER_REGISTRY:
            raise ValueError(f"Scanner '{name}' is already registered")
        SCANNER_REGISTRY[name] = factory
        return factory

    return decorator


def list_scanners() -> list[str]:
    """List all predefined scanner names.

    :return: A list of all predefined scanner names that can be used to create scanners with scanner_from_name().
    :rtype: list[str]
    """
    return list(SCANNER_REGISTRY.keys())


@validate_call
def scanner_from_name(scanner_name: str) -> Scanner:
    """Create a predefined scanner by its string name."""
    try:
        factory = SCANNER_REGISTRY[scanner_name]
    except KeyError as exc:
        valid_names = ", ".join(list_scanners())
        raise ValueError(
            f"Unknown scanner '{scanner_name}'. Available scanners: {valid_names}"
        ) from exc
    return factory()


@registered_scanner("leica_als50")
def leica_als50() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="leica_als50",
            supported_pulse_frequencies=(83000 * units.Hz,),
            optics=OscillatingOptics(
                scan_angle_max=0.6544984694978736 * units.rad,
                scan_frequency_min=25 * units.Hz,
                scan_frequency_max=70 * units.Hz,
                scan_product=1000000,
            ),
            beam_divergence=0.00033 * units.rad,
            pulse_length=1e-08 * units.s,
            detector=Detector(
                accuracy=0.05 * units.m,
                range_min=200 * units.m,
            ),
        )
    )


@registered_scanner("leica_als50_ii")
def leica_als50_ii() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="leica_als50-ii",
            supported_pulse_frequencies=(
                20000 * units.Hz,
                60000 * units.Hz,
                150000 * units.Hz,
            ),
            optics=OscillatingOptics(
                scan_angle_max=0.6544984694978736 * units.rad,
                scan_frequency_min=0 * units.Hz,
                scan_frequency_max=90 * units.Hz,
                scan_product=1000000,
            ),
            beam_divergence=0.00022 * units.rad,
            pulse_length=1e-08 * units.s,
            max_returns=4,
            detector=Detector(
                accuracy=0.05 * units.m,
                range_min=200 * units.m,
            ),
        )
    )


@registered_scanner("optech_2033")
def optech_2033() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="optech_2033",
            supported_pulse_frequencies=(33000 * units.Hz,),
            optics=OscillatingOptics(
                scan_angle_max=0.3490658503988659 * units.rad,
                scan_frequency_min=0 * units.Hz,
                scan_frequency_max=30 * units.Hz,
                scan_product=590,
            ),
            beam_origin_position=(0, 0.085, 0.06),
            beam_divergence=0.000424 * units.rad,
            pulse_length=8e-09 * units.s,
            wavelength=1.047e-06 * units.m,
            detector=Detector(
                accuracy=0.01 * units.m,
                range_min=1 * units.m,
            ),
        )
    )


@registered_scanner("optech_3100")
def optech_3100() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="optech_3100",
            supported_pulse_frequencies=(
                33000 * units.Hz,
                50000 * units.Hz,
                70000 * units.Hz,
                100000 * units.Hz,
            ),
            optics=OscillatingOptics(
                scan_angle_max=0.4363323129985824 * units.rad,
                scan_frequency_min=0 * units.Hz,
                scan_frequency_max=70 * units.Hz,
                scan_product=1000,
            ),
            beam_divergence=0.000424 * units.rad,
            pulse_length=8e-09 * units.s,
            max_returns=4,
            detector=Detector(
                accuracy=0.015 * units.m,
                range_min=80 * units.m,
            ),
        )
    )


@registered_scanner("optech_galaxy")
def optech_galaxy() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="optech_galaxy",
            supported_pulse_frequencies=(35000 * units.Hz, 550000 * units.Hz),
            optics=OscillatingOptics(
                scan_angle_max=0.5235987755982988 * units.rad,
                scan_frequency_min=0 * units.Hz,
                scan_frequency_max=120 * units.Hz,
                scan_product=2000,
            ),
            beam_divergence=0.000354 * units.rad,
            pulse_length=8e-09 * units.s,
            max_returns=8,
            detector=Detector(
                accuracy=0.03 * units.m,
                range_min=150 * units.m,
            ),
        )
    )


@registered_scanner("riegl_lms_q560")
def riegl_lms_q560() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_lms-q560",
            supported_pulse_frequencies=(
                50000 * units.Hz,
                100000 * units.Hz,
                180000 * units.Hz,
                200000 * units.Hz,
                240000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=0.7853981633974483 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=160 * units.Hz,
                scan_angle_effective_max=0.39269908169872414 * units.rad,
            ),
            beam_divergence=0.0005 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.02 * units.m,
                range_min=30 * units.m,
            ),
        )
    )


@registered_scanner("riegl_lms_q780")
def riegl_lms_q780() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_lms-q780",
            supported_pulse_frequencies=(
                100000 * units.Hz,
                200000 * units.Hz,
                300000 * units.Hz,
                400000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=0.7853981633974483 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=200 * units.Hz,
                scan_angle_effective_max=0.5235987755982988 * units.rad,
            ),
            beam_divergence=0.00025 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.02 * units.m,
                range_min=50 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vq_780i")
def riegl_vq_780i() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vq_780i",
            supported_pulse_frequencies=(
                150000 * units.Hz,
                250000 * units.Hz,
                350000 * units.Hz,
                500000 * units.Hz,
                700000 * units.Hz,
                1000000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=0.7853981633974483 * units.rad,
                scan_frequency_min=20 * units.Hz,
                scan_frequency_max=300 * units.Hz,
                scan_angle_effective_max=0.5235987755982988 * units.rad,
            ),
            beam_divergence=0.00025 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.02 * units.m,
                range_min=100 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vux_1uav")
def riegl_vux_1uav() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vux-1uav",
            supported_pulse_frequencies=(
                50000 * units.Hz,
                100000 * units.Hz,
                200000 * units.Hz,
                300000 * units.Hz,
                380000 * units.Hz,
                550000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=3.141592653589793 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=200 * units.Hz,
                scan_angle_effective_max=2.8797932657906435 * units.rad,
            ),
            beam_origin_position=(0, 0.085, 0.06),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg), ("z", 90 * units.deg))
            ),
            beam_divergence=0.0005 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.01 * units.m,
                range_min=3 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vux_1uav22")
def riegl_vux_1uav22() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vux-1uav22",
            supported_pulse_frequencies=(
                50000 * units.Hz,
                100000 * units.Hz,
                200000 * units.Hz,
                400000 * units.Hz,
                600000 * units.Hz,
                800000 * units.Hz,
                1200000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=3.141592653589793 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=200 * units.Hz,
                scan_angle_effective_max=3.141592653589793 * units.rad,
            ),
            beam_origin_position=(0, 0.085, 0.06),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg), ("z", 90 * units.deg))
            ),
            beam_divergence=0.0005 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.01 * units.m,
                range_min=3 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vux_1ha22")
def riegl_vux_1ha22() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vux-1ha22",
            supported_pulse_frequencies=(
                300000 * units.Hz,
                500000 * units.Hz,
                1000000 * units.Hz,
                1250000 * units.Hz,
                1500000 * units.Hz,
                1800000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=3.141592653589793 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=250 * units.Hz,
                scan_angle_effective_max=3.141592653589793 * units.rad,
            ),
            beam_origin_position=(0, 0.015, 0),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg), ("z", 90 * units.deg))
            ),
            beam_divergence=0.0005 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.005 * units.m,
                range_min=1 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vq_880g")
def riegl_vq_880g() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vq-880g",
            supported_pulse_frequencies=(
                200000 * units.Hz,
                400000 * units.Hz,
                550000 * units.Hz,
                700000 * units.Hz,
            ),
            optics=ConicOptics(
                scan_angle_max=0.3490658503988659 * units.rad,
                scan_frequency_min=30 * units.Hz,
                scan_frequency_max=80 * units.Hz,
            ),
            beam_origin_position=(0, 0, -0.286),
            beam_divergence=0.0007 * units.rad,
            pulse_length=2e-09 * units.s,
            wavelength=5.32e-07 * units.m,
            max_returns=9,
            detector=Detector(
                accuracy=0.025 * units.m,
                range_min=10 * units.m,
            ),
        )
    )


@registered_scanner("riegl_vq_1560i")
def riegl_vq_1560i() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=0.992546151641322, q1=0, q2=-0.12186934340514748, q3=0),
            5.32e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "1",
            Rotation(q0=0.992546151641322, q1=0, q2=0.12186934340514748, q3=0),
            1.064e-06 * units.m,
            (0, 1, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(
                    150000 * units.Hz,
                    250000 * units.Hz,
                    350000 * units.Hz,
                    500000 * units.Hz,
                    700000 * units.Hz,
                    1000000 * units.Hz,
                    2000000 * units.Hz,
                ),
                optics=RotatingOptics(
                    scan_angle_max=0.7853981633974483 * units.rad,
                    scan_frequency_min=40 * units.Hz,
                    scan_frequency_max=600 * units.Hz,
                    scan_angle_effective_max=0.5235987755982988 * units.rad,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.0007 * units.rad,
                pulse_length=4e-09 * units.s,
                wavelength=wavelength,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=100 * units.m,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("livox_mid70_als")
def livox_mid70_als() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="livox_mid-70",
            supported_pulse_frequencies=(100000 * units.Hz,),
            optics=RisleyOptics(
                prisms=(
                    Prism(
                        angle=0.5183627878423158 * units.rad,
                        inclined_on_left=False,
                        refractive_index=1.5095,
                        rotation_speed=-488.4129378759988 * units.rad / units.s,
                    ),
                    Prism(
                        angle=0.5183627878423158 * units.rad,
                        inclined_on_left=True,
                        refractive_index=1.5095,
                        rotation_speed=763.8258938386095 * units.rad / units.s,
                    ),
                ),
                air_refractive_index=1,
                incident_beam=(0, 1, 0),
            ),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg), ("z", 90 * units.deg))
            ),
            beam_divergence=0.00089 * units.rad,
            pulse_length=4e-09 * units.s,
            wavelength=9.05e-07 * units.m,
            max_returns=2,
            detector=Detector(
                accuracy=0.02 * units.m,
                range_min=2 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(1, 0, 0),
                rotation_speed_max=0 * units.rad / units.s,
            ),
        )
    )


@registered_scanner("livox_mid100_als")
def livox_mid100_als() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=0.9659258262890683, q1=0.25881904510252074, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        ("1", Rotation(q0=1, q1=0, q2=0, q3=0), 9.05e-07 * units.m, (0, 1, 0)),
        (
            "2",
            Rotation(q0=0.9659258262890683, q1=-0.25881904510252074, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(100000 * units.Hz,),
                optics=RisleyOptics(
                    prisms=(
                        Prism(
                            angle=0.3272160734931489 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=-763.8198200970015 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.3135204748527494 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=488.4736753360625 * units.rad / units.s,
                        ),
                    ),
                    air_refractive_index=1,
                    incident_beam=incident_beam,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.00089 * units.rad,
                pulse_length=4e-09 * units.s,
                wavelength=wavelength,
                max_returns=2,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=1 * units.m,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("dji_zenmuse_l2_repetitive")
def dji_zenmuse_l2_repetitive() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.02687759, 0.99963873, 0),
        ),
        (
            "1",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.01613028, 0.9998699, 0),
        ),
        (
            "2",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.00537738, 0.99998554, 0),
        ),
        (
            "3",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.00537738, 0.99998554, 0),
        ),
        (
            "4",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.01613028, 0.9998699, 0),
        ),
        (
            "5",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.02687759, 0.99963873, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(40000 * units.Hz,),
                optics=RisleyOptics(
                    prisms=(
                        Prism(
                            angle=0.48262734594065715 * units.rad,
                            inclined_on_left=False,
                            refractive_index=1.51,
                            rotation_speed=314.1592653589793 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.5173765022820139 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=-314.1592653589793 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.04853551210285991 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=-942.4777960769379 * units.rad / units.s,
                        ),
                    ),
                    air_refractive_index=1,
                    incident_beam=incident_beam,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.00034 * units.rad,
                pulse_length=4e-09 * units.s,
                wavelength=wavelength,
                max_returns=5,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=2 * units.m,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("dji_zenmuse_l2_non_repetitive")
def dji_zenmuse_l2_non_repetitive() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.02687759, 0.99963873, 0),
        ),
        (
            "1",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.01613028, 0.9998699, 0),
        ),
        (
            "2",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (-0.00537738, 0.99998554, 0),
        ),
        (
            "3",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.00537738, 0.99998554, 0),
        ),
        (
            "4",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.01613028, 0.9998699, 0),
        ),
        (
            "5",
            Rotation(q0=1, q1=0, q2=0, q3=0),
            9.05e-07 * units.m,
            (0.02687759, 0.99963873, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(40000 * units.Hz,),
                optics=RisleyOptics(
                    prisms=(
                        Prism(
                            angle=0.48262734594065715 * units.rad,
                            inclined_on_left=False,
                            refractive_index=1.51,
                            rotation_speed=66.15125986957884 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.5173765022820139 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=-324.55542376823865 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.04853551210285991 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=66.15144836513805 * units.rad / units.s,
                        ),
                    ),
                    air_refractive_index=1,
                    incident_beam=incident_beam,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.00034 * units.rad,
                pulse_length=4e-09 * units.s,
                wavelength=wavelength,
                max_returns=5,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=2 * units.m,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("riegl_vz_400")
def riegl_vz_400() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vz400",
            supported_pulse_frequencies=(100000 * units.Hz, 300000 * units.Hz),
            optics=RotatingOptics(
                scan_angle_max=2.0943951023931953 * units.rad,
                scan_frequency_min=3 * units.Hz,
                scan_frequency_max=120 * units.Hz,
                scan_angle_effective_max=0.8726646259971648 * units.rad,
            ),
            beam_origin_position=(0, 0, 0.2),
            beam_divergence=0.0003 * units.rad,
            pulse_length=5e-09 * units.s,
            detector=Detector(
                accuracy=0.005 * units.m,
                range_min=1.5 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=1.0471975511965976 * units.rad / units.s,
            ),
        )
    )


@registered_scanner("riegl_vz_1000")
def riegl_vz_1000() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vz1000",
            supported_pulse_frequencies=(
                70000 * units.Hz,
                100000 * units.Hz,
                150000 * units.Hz,
                300000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=2.0943951023931953 * units.rad,
                scan_frequency_min=3 * units.Hz,
                scan_frequency_max=120 * units.Hz,
                scan_angle_effective_max=0.8726646259971648 * units.rad,
            ),
            beam_divergence=0.0003 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.008 * units.m,
                range_min=2.5 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=1.0471975511965976 * units.rad / units.s,
            ),
        )
    )


@registered_scanner("riegl_vz_2000i")
def riegl_vz_2000i() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vz2000i",
            supported_pulse_frequencies=(
                50000 * units.Hz,
                100000 * units.Hz,
                300000 * units.Hz,
                600000 * units.Hz,
                1200000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=2.0943951023931953 * units.rad,
                scan_frequency_min=3 * units.Hz,
                scan_frequency_max=240 * units.Hz,
                scan_angle_effective_max=0.8726646259971648 * units.rad,
            ),
            beam_origin_position=(0, 0, 0.2),
            beam_divergence=0.00027 * units.rad,
            pulse_length=5e-09 * units.s,
            detector=Detector(
                accuracy=0.005 * units.m,
                range_min=1.5 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=2.6179938779914944 * units.rad / units.s,
            ),
        )
    )


@registered_scanner("riegl_vz_600i")
def riegl_vz_600i() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vz600i",
            supported_pulse_frequencies=(
                140000 * units.Hz,
                600000 * units.Hz,
                1200000 * units.Hz,
                2200000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=0.9162978572970231 * units.rad,
                scan_frequency_min=4 * units.Hz,
                scan_frequency_max=420 * units.Hz,
                scan_angle_effective_max=0.9162978572970231 * units.rad,
            ),
            beam_origin_position=(0, 0, 0.2),
            beam_divergence=0.00035 * units.rad,
            pulse_length=5e-09 * units.s,
            detector=Detector(
                accuracy=0.005 * units.m,
                range_min=0.5 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=6.283185307179586 * units.rad / units.s,
            ),
        )
    )


@registered_scanner("riegl_vq_450")
def riegl_vq_450() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="riegl_vq-450",
            supported_pulse_frequencies=(
                150000 * units.Hz,
                200000 * units.Hz,
                300000 * units.Hz,
                380000 * units.Hz,
                550000 * units.Hz,
                550000 * units.Hz,
            ),
            optics=RotatingOptics(
                scan_angle_max=6.283185307179586 * units.rad,
                scan_frequency_min=10 * units.Hz,
                scan_frequency_max=200 * units.Hz,
                scan_angle_effective_max=6.283185307179586 * units.rad,
            ),
            beam_origin_position=(0, 0.14, 0.126),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg), ("z", 90 * units.deg))
            ),
            beam_divergence=0.0003 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0.008 * units.m,
                range_min=1.5 * units.m,
            ),
        )
    )


@registered_scanner("livox_mid40")
def livox_mid40() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="livox_mid-40",
            supported_pulse_frequencies=(100000 * units.Hz,),
            optics=RisleyOptics(
                prisms=(
                    Prism(
                        angle=0.3272160734931489 * units.rad,
                        inclined_on_left=True,
                        refractive_index=1.51,
                        rotation_speed=-763.8198200970015 * units.rad / units.s,
                    ),
                    Prism(
                        angle=0.3135204748527494 * units.rad,
                        inclined_on_left=True,
                        refractive_index=1.51,
                        rotation_speed=488.4736753360625 * units.rad / units.s,
                    ),
                ),
                air_refractive_index=1,
                incident_beam=(0, 1, 0),
            ),
            beam_divergence=0.00089 * units.rad,
            pulse_length=4e-09 * units.s,
            wavelength=9.05e-07 * units.m,
            max_returns=2,
            detector=Detector(
                accuracy=0.01 * units.m,
                range_min=1 * units.m,
            ),
        )
    )


@registered_scanner("livox_mid100")
def livox_mid100() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=0.9659258262890683, q1=0, q2=0, q3=0.25881904510252074),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        ("1", Rotation(q0=1, q1=0, q2=0, q3=0), 9.05e-07 * units.m, (0, 1, 0)),
        (
            "2",
            Rotation(q0=0.9659258262890683, q1=0, q2=0, q3=-0.25881904510252074),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(100000 * units.Hz,),
                optics=RisleyOptics(
                    prisms=(
                        Prism(
                            angle=0.3272160734931489 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=-763.8198200970015 * units.rad / units.s,
                        ),
                        Prism(
                            angle=0.3135204748527494 * units.rad,
                            inclined_on_left=True,
                            refractive_index=1.51,
                            rotation_speed=488.4736753360625 * units.rad / units.s,
                        ),
                    ),
                    air_refractive_index=1,
                    incident_beam=incident_beam,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.00089 * units.rad,
                pulse_length=4e-09 * units.s,
                wavelength=wavelength,
                max_returns=2,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=1 * units.m,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("livox_mid70")
def livox_mid70() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="livox_mid-70",
            supported_pulse_frequencies=(100000 * units.Hz,),
            optics=RisleyOptics(
                prisms=(
                    Prism(
                        angle=0.5183627878423158 * units.rad,
                        inclined_on_left=False,
                        refractive_index=1.5095,
                        rotation_speed=-488.4129378759988 * units.rad / units.s,
                    ),
                    Prism(
                        angle=0.5183627878423158 * units.rad,
                        inclined_on_left=True,
                        refractive_index=1.5095,
                        rotation_speed=763.8258938386095 * units.rad / units.s,
                    ),
                ),
                air_refractive_index=1,
                incident_beam=(0, 1, 0),
            ),
            beam_divergence=0.00089 * units.rad,
            pulse_length=4e-09 * units.s,
            wavelength=9.05e-07 * units.m,
            max_returns=2,
            detector=Detector(
                accuracy=0.02 * units.m,
                range_min=2 * units.m,
            ),
        )
    )


@registered_scanner("vlp16")
def vlp16() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=0.9914448613738104, q1=0.13052619222005157, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "1",
            Rotation(q0=0.9999619230641713, q1=-0.008726535498373935, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "2",
            Rotation(q0=0.9935718556765875, q1=0.11320321376790672, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "3",
            Rotation(q0=0.9996573249755573, q1=-0.026176948307873153, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "4",
            Rotation(q0=0.9953961983671789, q1=0.09584575252022398, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "5",
            Rotation(q0=0.9990482215818578, q1=-0.043619387365336, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "6",
            Rotation(q0=0.996917333733128, q1=0.07845909572784494, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "7",
            Rotation(q0=0.9981347984218669, q1=-0.06104853953485687, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "8",
            Rotation(q0=0.9981347984218669, q1=0.06104853953485687, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "9",
            Rotation(q0=0.996917333733128, q1=-0.07845909572784494, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "10",
            Rotation(q0=0.9990482215818578, q1=0.043619387365336, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "11",
            Rotation(q0=0.9953961983671789, q1=-0.09584575252022398, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "12",
            Rotation(q0=0.9996573249755573, q1=0.026176948307873153, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "13",
            Rotation(q0=0.9935718556765875, q1=-0.11320321376790672, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "14",
            Rotation(q0=0.9999619230641713, q1=0.008726535498373935, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "15",
            Rotation(q0=0.9914448613738104, q1=-0.13052619222005157, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(18750 * units.Hz,),
                optics=RotatingOptics(
                    scan_angle_max=0.017453292519943295 * units.rad,
                    scan_frequency_min=0 * units.Hz,
                    scan_frequency_max=0 * units.Hz,
                    scan_angle_effective_max=0.017453292519943295 * units.rad,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.0007 * units.rad,
                pulse_length=5e-09 * units.s,
                wavelength=wavelength,
                max_returns=2,
                detector=Detector(
                    accuracy=0.03 * units.m,
                    range_min=0.09144 * units.m,
                    range_max=100 * units.m,
                ),
                head=ScannerHead(
                    rotation_axis=(0, 0, 1),
                    rotation_speed_max=125.66370614359172 * units.rad / units.s,
                ),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("velodyne_hdl_64e")
def velodyne_hdl_64e() -> Scanner:
    channel_specs = (
        (
            "0",
            Rotation(q0=0.9766722783341679, q1=0.2147353271670632, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "1",
            Rotation(q0=0.9774619749435719, q1=0.21111155235896517, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "2",
            Rotation(q0=0.9782400368169809, q1=0.20747633688763586, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "3",
            Rotation(q0=0.9790028003834026, q1=0.20384679747657422, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "4",
            Rotation(q0=0.9797538445727765, q1=0.20020590412089176, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "5",
            Rotation(q0=0.9804896330291901, q1=0.19657080027889237, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "6",
            Rotation(q0=0.98121193449322, q1=0.19293299253386653, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "7",
            Rotation(q0=0.9819223908809145, q1=0.18928396204303352, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "8",
            Rotation(q0=0.9826176569475028, q1=0.18564089057909533, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "9",
            Rotation(q0=0.9833009949208249, q1=0.18198668464400342, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "10",
            Rotation(q0=0.9839691872454873, q1=0.1783385503794824, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "11",
            Rotation(q0=0.9846238447152292, q1=0.17468796300317962, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "12",
            Rotation(q0=0.9852664508491874, q1=0.17102637466497936, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "13",
            Rotation(q0=0.9858939798813239, q1=0.16737102626728376, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "14",
            Rotation(q0=0.986509376213469, q1=0.16370476670797443, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "15",
            Rotation(q0=0.9871097421992678, q1=0.16004485888429829, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "16",
            Rotation(q0=0.9876965301306678, q1=0.15638274958523668, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "17",
            Rotation(q0=0.9882710646187888, q1=0.15270986489826321, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "18",
            Rotation(q0=0.9888306404187694, q1=0.14904349891560598, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "19",
            Rotation(q0=0.9893778830920186, q1=0.14536644884001276, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "20",
            Rotation(q0=0.9899102158995233, q1=0.14169602837680095, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "21",
            Rotation(q0=0.9904289321310923, q1=0.13802365883305714, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "22",
            Rotation(q0=0.9909351970334708, q1=0.1343407431877478, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "23",
            Rotation(q0=0.9914266268149909, q1=0.13066462276472776, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "24",
            Rotation(q0=0.9919055272920757, q1=0.12697804899835702, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "25",
            Rotation(q0=0.9923696435207983, q1=0.12329838043706698, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "26",
            Rotation(q0=0.9928201093432615, q1=0.11961701586160008, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "27",
            Rotation(q0=0.9932579302403823, q1=0.11592533810428036, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "28",
            Rotation(q0=0.9936810446949559, q1=0.11224072974611785, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "29",
            Rotation(q0=0.9940914379840141, q1=0.10854590239559919, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "30",
            Rotation(q0=0.9944871777355946, q1=0.10485825346386382, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "31",
            Rotation(q0=0.9948692379534605, q1=0.10116916216862128, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "32",
            Rotation(q0=0.9952384640062902, q1=0.09746999416435916, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "33",
            Rotation(q0=0.99559311736158, q1=0.09377816730055694, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "34",
            Rotation(q0=0.9959340759705525, q1=0.09008505048387246, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "35",
            Rotation(q0=0.9962620136233985, q1=0.08638286989358077, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "36",
            Rotation(q0=0.9965758284605366, q1=0.08268384442075193, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "37",
            Rotation(q0=0.9968759092179038, q1=0.07898367945960456, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "38",
            Rotation(q0=0.9971622517600065, q1=0.07528242600310811, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "39",
            Rotation(q0=0.997434852140681, q1=0.07158013505923284, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "40",
            Rotation(q0=0.9976937066031479, q1=0.06787685765024688, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "41",
            Rotation(q0=0.9979388115800644, q1=0.06417264481201317, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "42",
            Rotation(q0=0.9981701636935727, q1=0.06046754759328597, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "43",
            Rotation(q0=0.9983877597553471, q1=0.0567616170550075, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "44",
            Rotation(q0=0.9985915967666377, q1=0.05305490426960412, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "45",
            Rotation(q0=0.9987816719183118, q1=0.049347460320282595, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "46",
            Rotation(q0=0.9989579825908924, q1=0.045639336300326024, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "47",
            Rotation(q0=0.9991205263545949, q1=0.04193058331238976, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "48",
            Rotation(q0=0.9992692676146443, q1=0.038222124494753366, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "49",
            Rotation(q0=0.99940427426763, q1=0.034512267030605406, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "50",
            Rotation(q0=0.9995255078612717, q1=0.03080193394361477, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "51",
            Rotation(q0=0.9996329667248167, q1=0.027091176366880562, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "52",
            Rotation(q0=0.9997266493773455, q1=0.023380045439351873, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "53",
            Rotation(q0=0.9998065545277921, q1=0.019668592305123064, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "54",
            Rotation(q0=0.9998726810749622, q1=0.0159568681127289, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "55",
            Rotation(q0=0.999925028107548, q1=0.012244924014439678, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "56",
            Rotation(q0=0.9999635963933823, q1=0.008532636638984828, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "57",
            Rotation(q0=0.9999883817745766, q1=0.004820406192807788, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "58",
            Rotation(q0=0.9999993860466841, q1=0.0011081093153153482, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "59",
            Rotation(q0=0.999996609285308, q1=-0.0026041155671631764, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "60",
            Rotation(q0=0.999980051398164, q1=-0.006316391828033058, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "61",
            Rotation(q0=0.9999497133918914, q1=-0.010028493778932555, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "62",
            Rotation(q0=0.9999055942582262, q1=-0.013740544789181384, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
        (
            "63",
            Rotation(q0=0.9998476951563913, q1=-0.01745240643728351, q2=0, q3=0),
            9.05e-07 * units.m,
            (0, 1, 0),
        ),
    )
    return MultiScanner(
        channels=tuple(
            ScanningDevice(
                id=channel_id,
                supported_pulse_frequencies=(20833 * units.Hz,),
                optics=RotatingOptics(
                    scan_angle_max=0.017453292519943295 * units.rad,
                    scan_frequency_min=0 * units.Hz,
                    scan_frequency_max=0 * units.Hz,
                    scan_angle_effective_max=0.017453292519943295 * units.rad,
                ),
                beam_origin_attitude=beam_origin_attitude,
                beam_divergence=0.0034 * units.rad,
                pulse_length=5e-09 * units.s,
                wavelength=wavelength,
                max_returns=1,
                detector=Detector(
                    accuracy=0.02 * units.m,
                    range_min=0.9144 * units.m,
                    range_max=120 * units.m,
                ),
                head=ScannerHead(
                    rotation_axis=(0, 0, 1),
                    rotation_speed_max=94.2477796076938 * units.rad / units.s,
                ),
                full_waveform=FullWaveformSettings(beam_sample_quality=1),
            )
            for channel_id, beam_origin_attitude, wavelength, incident_beam in channel_specs
        )
    )


@registered_scanner("tractor_scanner")
def tractor_scanner() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="tractorscanner",
            supported_pulse_frequencies=(
                1000 * units.Hz,
                50000 * units.Hz,
                100000 * units.Hz,
                200000 * units.Hz,
            ),
            optics=OscillatingOptics(
                scan_angle_max=0.5235987755982988 * units.rad,
                scan_frequency_min=100 * units.Hz,
                scan_frequency_max=50 * units.Hz,
                scan_product=1000000,
            ),
            beam_divergence=0.00035 * units.rad,
            pulse_length=4e-09 * units.s,
            detector=Detector(
                accuracy=0 * units.m,
                range_min=0 * units.m,
            ),
        )
    )


@registered_scanner("pano_scanner")
def pano_scanner() -> Scanner:
    return SingleScanner(
        device=ScanningDevice(
            id="panoscanner",
            supported_pulse_frequencies=(100000 * units.Hz, 300000 * units.Hz),
            optics=RotatingOptics(
                scan_angle_max=3.141592653589793 * units.rad,
                scan_frequency_min=0 * units.Hz,
                scan_frequency_max=12000 * units.Hz,
                scan_angle_effective_max=2.6179938779914944 * units.rad,
            ),
            beam_origin_position=(0, 0, 0.2),
            beam_origin_attitude=Rotation.from_axis_angles(
                rotations=(("x", 90 * units.deg),)
            ),
            beam_divergence=0.0003 * units.rad,
            pulse_length=5e-09 * units.s,
            detector=Detector(
                accuracy=0.005 * units.m,
                range_min=0.01 * units.m,
            ),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=8.726646259971648 * units.rad / units.s,
            ),
        )
    )
