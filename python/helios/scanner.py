from helios.utils import classonlymethod, get_asset_directories
from helios.validation import (
    Angle,
    AngleVelocity,
    AssetPath,
    Frequency,
    Length,
    Model,
    units,
    UpdateableMixin,
    TimeInterval,
    validate_xml_file,
)
from pydantic import validate_call
from typing import Callable

import _helios
import numpy as np


class ScannerSettingsBase(Model, UpdateableMixin, cpp_class=_helios.ScannerSettings):
    pass


class ScannerSettings(ScannerSettingsBase):
    is_active: bool = True
    head_rotation: AngleVelocity = 0
    rotation_start_angle: Angle = 0
    rotation_stop_angle: Angle = 0
    pulse_frequency: Frequency = 300000
    scan_angle: Angle = 0
    min_vertical_angle: Angle = np.nan
    max_vertical_angle: Angle = np.nan
    scan_frequency: Frequency = 200
    beam_divergence_angle: Angle = 0.003 * units.rad
    trajectory_time_interval: TimeInterval = 0.01
    vertical_resolution: Angle = 0
    horizontal_resolution: Angle = 0


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


class Scanner(Model, cpp_class=_helios.Scanner):
    @classonlymethod
    @validate_call
    def from_xml(cls, scanner_file: AssetPath, scanner_id: str = ""):
        """Classmethod to load a scanner from an XML file. The XML file should conform to the schema defined in "xsd/scanner.xsd". The scanner_id parameter can be used to specify which scanner to load if the XML file contains multiple scanners. The method validates the XML file against the schema before loading the scanner."""

        # Validate the XML
        validate_xml_file(scanner_file, "xsd/scanner.xsd")

        _cpp_scanner = _helios.read_scanner_from_xml(
            str(scanner_file), [str(p) for p in get_asset_directories()], scanner_id
        )
        scanner = cls._from_cpp(_cpp_scanner)
        scanner._is_loaded_from_xml = True
        return scanner


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
    "livox_mid70": ("data/scanners_als.xml", "livox_mid-70"),
    "livox_mid100": ("data/scanners_als.xml", "livox-mid-100"),
    "livox_mid100a": ("data/scanners_als.xml", "livox-mid-100a"),
    "livox_mid100b": ("data/scanners_als.xml", "livox-mid-100b"),
    "livox_mid100c": ("data/scanners_als.xml", "livox-mid-100c"),
    # TLS Scanners
    "riegl_vz_400": ("data/scanners_tls.xml", "riegl_vz400"),
    "riegl_vz_1000": ("data/scanners_tls.xml", "riegl_vz1000"),
    "riegl_vz_2000i": ("data/scanners_tls.xml", "riegl_vz2000i"),
    "riegl_vz_600i": ("data/scanners_tls.xml", "riegl_vz600i"),
    "riegl_vq_450": ("data/scanners_tls.xml", "riegl_vq-450"),
    "livox_mid70_tls": ("data/scanners_tls.xml", "livox_mid-70"),
    "vlp16": ("data/scanners_tls.xml", "vlp16"),
    "velodyne_hdl_64e": ("data/scanners_tls.xml", "velodyne_hdl-64e"),
    "tractor_scanner": ("data/scanners_tls.xml", "tractorscanner"),
    "pano_scanner": ("data/scanners_tls.xml", "panoscanner"),
}


def list_scanners() -> list[str]:
    """List all predefined scanner names."""
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
