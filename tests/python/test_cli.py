from pathlib import Path
from click.testing import CliRunner
from unittest.mock import patch
import pytest
import helios
from helios.__main__ import cli


def test_cli_help():
    runner = CliRunner()
    result = runner.invoke(cli, ["--help"])
    assert result.exit_code == 0


def test_cli_version():
    runner = CliRunner()
    result = runner.invoke(cli, ["--version"])
    assert result.exit_code == 0
    assert helios.__version__ in result.stdout


def test_cli_call():
    runner = CliRunner()
    result = runner.invoke(
        cli, ["--dryrun", "data/surveys/demo/box_survey_static_puck.xml"]
    )
    assert result.exit_code == 0
    # Missing argument:
    result = runner.invoke(cli, ["--dryrun"])
    assert result.exit_code != 0


def test_cli_uses_explicit_seed():
    runner = CliRunner()
    with patch("helios.__main__.set_rng_seed") as set_rng_seed:
        result = runner.invoke(
            cli,
            [
                "--dryrun",
                "--seed",
                "12345",
                "data/surveys/demo/box_survey_static_puck.xml",
            ],
        )
    assert result.exit_code == 0
    set_rng_seed.assert_called_once_with(12345)


def test_cli_uses_random_seed_when_not_specified():
    runner = CliRunner()
    with patch("helios.__main__.set_rng_seed") as set_rng_seed:
        result = runner.invoke(
            cli,
            [
                "--dryrun",
                "data/surveys/demo/box_survey_static_puck.xml",
            ],
        )
    assert result.exit_code == 0
    set_rng_seed.assert_called_once_with()


def test_cli_loads_survey_from_yaml(tmp_path):
    survey_file = tmp_path / "survey.yaml"
    survey_file.touch()
    runner = CliRunner()
    with patch("helios.__main__.Survey") as survey_cls:
        result = runner.invoke(cli, ["--dryrun", str(survey_file)])
    assert result.exit_code == 0
    survey_cls.from_yaml.assert_called_once_with(survey_file)
    survey_cls.from_xml.assert_not_called()


def test_cli_loads_survey_from_xml():
    runner = CliRunner()
    with patch("helios.__main__.Survey") as survey_cls:
        result = runner.invoke(
            cli, ["--dryrun", "data/surveys/demo/box_survey_static_puck.xml"]
        )
    assert result.exit_code == 0
    survey_cls.from_xml.assert_called_once_with(
        Path("data/surveys/demo/box_survey_static_puck.xml")
    )
    survey_cls.from_yaml.assert_not_called()


def test_cli_incorrect_file_extension(tmp_path):
    survey_file = tmp_path / "survey.txt"
    survey_file.touch()
    runner = CliRunner()
    result = runner.invoke(cli, ["--dryrun", str(survey_file)])
    assert result.exit_code != 0
