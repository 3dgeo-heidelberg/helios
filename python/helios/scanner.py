from helios.util import get_asset_directories
from helios.validation import ValidatedCppModel, ValidatedCppManagedProperty, UpdateableMixin
from pathlib import Path

import _helios
import numpy as np


class ScannerSettingsBase(ValidatedCppModel, UpdateableMixin, cpp_class=_helios.ScannerSettings):
    pass


class ScannerSettings(ScannerSettingsBase):
    is_active: bool = ValidatedCppManagedProperty("is_active", default=True)
    head_rotation: float = ValidatedCppManagedProperty("head_rotation", default=0)
    rotation_start_angle: float = ValidatedCppManagedProperty("rotation_start_angle", default=0)
    rotation_stop_angle: float = ValidatedCppManagedProperty("rotation_stop_angle", default=0)
    pulse_frequency: int = ValidatedCppManagedProperty("pulse_frequency", default=300000)
    scan_angle: float = ValidatedCppManagedProperty("scan_angle", default=0.349066)
    min_vertical_angle: float = ValidatedCppManagedProperty("min_vertical_angle", default=np.nan)
    max_vertical_angle: float = ValidatedCppManagedProperty("max_vertical_angle", default=np.nan)
    scan_frequency: float = ValidatedCppManagedProperty("scan_frequency", default=200)
    beam_divergence_angle: float = ValidatedCppManagedProperty("beam_divergence_angle", default=0.0003)
    trajectory_time_interval: float = ValidatedCppManagedProperty(
        "trajectory_time_interval", default=0.01
    )
    vertical_resolution: float = ValidatedCppManagedProperty("vertical_resolution", default=0)
    horizontal_resolution: float = ValidatedCppManagedProperty("horizontal_resolution", default=0)


# TODO: Requires expert input
class RotatingOpticsScannerSettings(ScannerSettingsBase):
    is_active: bool = ValidatedCppManagedProperty("is_active", default=True)
    head_rotation: float = ValidatedCppManagedProperty("head_rotation", default=0)
    rotation_start_angle: float = ValidatedCppManagedProperty("rotation_start_angle", default=0)
    rotation_stop_angle: float = ValidatedCppManagedProperty("rotation_stop_angle", default=0)


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


class Scanner(ValidatedCppModel, cpp_class=_helios.Scanner):
    @classmethod
    def from_xml(cls, scanner_file: Path, scanner_id: str = ""):

        _cpp_scanner = _helios.read_scanner_from_xml(
            scanner_file, [str(p) for p in get_asset_directories()], scanner_id
        )
        return cls.__new__(cls, _cpp_object=_cpp_scanner)


#
# Predefined scanners
#

# ALS Scanners


def leica_als50():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")


def leica_als50_ii():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50-ii")


def optech_2033():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="optech_2033")


def optech_3100():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="optech_3100")


def optech_galaxy():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="optech_galaxy")


def riegl_lms_q560():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_lms-q560")


def riegl_lms_q780():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_lms-q780")


def riegl_vq_780i():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vq_780i")


def riegl_vux_1uav():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vux-1uav")


def riegl_vux_1uav22():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vux-1uav22")


def riegl_vux_1ha22():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vux-1ha22")


def riegl_vq_880g():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vq-880g")


def riegl_vq_1560i():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="riegl_vq-1560i")


def livox_mid70():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="livox_mid-70")


def livox_mid100():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="livox-mid-100")


def livox_mid100a():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="livox-mid-100a")


def livox_mid100b():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="livox-mid-100b")


def livox_mid100c():
    return Scanner.from_xml("data/scanners_als.xml", scanner_id="livox-mid-100c")


# TLS Scanners


def riegl_vz_400():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="riegl_vz400")


def riegl_vz_1000():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="riegl_vz1000")


def riegl_vq_450():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="riegl_vq-450")


def livox_mid70_tls():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="livox_mid-70")


def vlp16():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="vlp16")


def velodyne_hdl_64e():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="velodyne_hdl-64e")


def tractor_scanner():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="tractorscanner")


def pano_scanner():
    return Scanner.from_xml("data/scanners_tls.xml", scanner_id="panoscanner")
