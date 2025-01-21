from helios.platform import Platform
from helios.scanner import Scanner
from helios.scene import Scene
from helios.survey import *


def test_construct_survey_from_xml():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    assert survey.name == "toyblocks_als"
    assert len(survey.legs) == 6
    assert isinstance(survey.platform, Platform)
    assert isinstance(survey.scanner, Scanner)
    assert isinstance(survey.scene, Scene)
