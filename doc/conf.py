import os
import pathlib
import re
import subprocess
import sys

from importlib import metadata as importlib_metadata


ROOT = pathlib.Path(__file__).resolve().parents[1]
PYTHON_DIR = ROOT / "python"

sys.path.insert(0, str(PYTHON_DIR))


def _read_local_version():
    version_file = PYTHON_DIR / "helios" / "_version.py"
    if not version_file.exists():
        return "unknown"

    content = version_file.read_text(encoding="utf-8")
    match = re.search(r"__version__\s*=\s*version\s*=\s*['\"]([^'\"]+)['\"]", content)
    if match is None:
        return "unknown"
    return match.group(1)


project = "Helios++"
copyright = "2026, HELIOS++ dev team"
author = "HELIOS++ dev team"
release = _read_local_version()

extensions = [
    "sphinx.ext.autodoc",
    "breathe",
    "nbsphinx",
    "nbsphinx_link",
    "sphinx_mdinclude",
    "sphinx_rtd_theme",
]

templates_path = []
exclude_patterns = ["_build"]
html_theme = "sphinx_rtd_theme"
html_static_path = []

autodoc_mock_imports = ["_helios"]
nbsphinx_execute = "never"

breathe_projects = {}
breathe_default_project = "helios"


if os.environ.get("READTHEDOCS", "False") == "True":
    # Use a stable path in the checkout root so RTD artifact folders stay clean.
    builddir = ROOT / "build-cmake-rtd"
    builddir.mkdir(exist_ok=True)
    subprocess.check_call(
        ["cmake", "-DHELIOS_DOCS_ONLY=ON", str(ROOT)],
        cwd=builddir,
    )
    subprocess.check_call(
        ["cmake", "--build", ".", "--target", "doxygen"], cwd=builddir
    )
    breathe_projects["helios"] = str(builddir / "doc" / "xml")
