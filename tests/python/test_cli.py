from click.testing import CliRunner
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
