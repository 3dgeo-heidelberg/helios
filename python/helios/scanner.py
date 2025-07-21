from helios.utils import get_asset_directories
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
    scan_angle: Angle = 0.349066
    min_vertical_angle: Angle = np.nan
    max_vertical_angle: Angle = np.nan
    scan_frequency: Frequency = 200
    beam_divergence_angle: Angle = 0.003 * units.rad
    trajectory_time_interval: TimeInterval = 0.01
    vertical_resolution: Length = 0
    horizontal_resolution: Length = 0


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
    @classmethod
    @validate_call
    def from_xml(cls, scanner_file: AssetPath, scanner_id: str = ""):

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
