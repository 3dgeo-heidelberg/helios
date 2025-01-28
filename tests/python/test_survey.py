from helios.platform import Platform
from helios.scanner import Scanner
from helios.scene import Scene
from helios.survey import *

import pytest


def test_construct_survey_from_xml():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    assert survey.name == "toyblocks_als"
    assert len(survey.legs) == 6
    assert isinstance(survey.platform, Platform)
    assert isinstance(survey.scanner, Scanner)
    assert isinstance(survey.scene, Scene)


def test_add_leg_parameters():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    platform_settings = PlatformSettings(x=5)
    scanner_settings = ScannerSettings(pulse_frequency=1000)
    survey.add_leg(
        platform_settings=platform_settings,
        scanner_settings=scanner_settings,
        y=5,
        head_rotation=12,
    )
    assert survey.legs[-1].platform_settings.x == 5
    assert survey.legs[-1].platform_settings.y == 5
    assert survey.legs[-1].scanner_settings.pulse_frequency == 1000
    assert survey.legs[-1].scanner_settings.head_rotation == 12

    with pytest.raises(ValueError):
        survey.add_leg(foobar=12)
