import subprocess
import sys

from importlib import resources


def _get_executable():
    """Locate the compiled Helios executable."""
    return resources.files("_pyhelios") / "pyhelios" / "bin" / "helios++"


def helios_exec(args):
    executable = _get_executable()
    return subprocess.call([executable] + args)


def helios_entrypoint():
    raise SystemExit(helios_exec(sys.argv[1:]))


if __name__ == "__main__":
    helios_entrypoint()
