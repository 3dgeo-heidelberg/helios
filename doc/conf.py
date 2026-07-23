import os
import pathlib
import re
import subprocess
import sys

from pybtex.plugin import register_plugin
from pybtex.style.formatting import toplevel
from pybtex.style.formatting.plain import Style as PlainStyle
from pybtex.style.formatting.unsrt import pages
from pybtex.style.sorting import BaseSortingStyle
from pybtex.style.template import (
    field,
    join,
    names,
    optional,
    optional_field,
    sentence,
    tag,
    words,
)

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


project = "HELIOS++"
copyright = "2026, HELIOS++ dev team"
author = "HELIOS++ dev team"
release = _read_local_version()

extensions = [
    "sphinx.ext.autodoc",
    "autoapi.extension",
    "breathe",
    "nbsphinx",
    "nbsphinx_link",
    "sphinx_mdinclude",
    "sphinx_rtd_theme",
    "sphinxcontrib.bibtex",
]

autoapi_dirs = ["../python/helios"]
autoapi_add_toctree_entry = False
autoapi_generate_api_docs = False
autoapi_keep_files = True

bibtex_bibfiles = ["refs.bib", "refs_using_helios.bib"]
bibtex_reference_style = "author_year"
bibtex_foot_reference_style = "author_year"

suppress_warnings = ["bibtex.duplicate_label"]

templates_path = ["_templates"]
exclude_patterns = ["_build"]
html_theme = "sphinx_rtd_theme"
html_favicon = "3dgeo.ico"
html_static_path = ["_static"]
html_css_files = ["theme_overrides.css"]

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


class ApaStyle(PlainStyle):
    default_name_style = "lastfirst"
    default_label_style = "alpha"
    default_sorting_style = "author_year_title"

    def __init__(self, **kwargs):
        kwargs.setdefault("abbreviate_names", True)
        super().__init__(**kwargs)

    def format_names(self, role, as_sentence=True):
        formatted_names = names(role, sep=", ", sep2=", & ", last_sep=", & ")
        if as_sentence:
            return sentence[formatted_names]
        return formatted_names

    def get_article_template(self, e):
        volume_and_issue = join[
            tag("em")[field("volume")],
            optional["(", field("number"), ")"],
        ]
        return toplevel[
            self.format_names("author"),
            join["(", field("year"), ")."],
            self.format_title(e, "title"),
            sentence[tag("em")[field("journal")], optional[volume_and_issue], pages],
            sentence[optional_field("note")],
            self.format_web_refs(e),
        ]

    def get_book_template(self, e):
        return toplevel[
            self.format_author_or_editor(e),
            join["(", field("year"), ")."],
            self.format_btitle(e, "title"),
            self.format_volume_and_series(e),
            sentence[
                field("publisher"),
                optional_field("address"),
                self.format_edition(e),
            ],
            optional[sentence[self.format_isbn(e)]],
            sentence[optional_field("note")],
            self.format_web_refs(e),
        ]

    def get_inproceedings_template(self, e):
        return toplevel[
            self.format_names("author"),
            join["(", field("year"), ")."],
            self.format_title(e, "title"),
            words[
                "In",
                sentence[
                    optional[self.format_editor(e, as_sentence=False)],
                    self.format_btitle(e, "booktitle", as_sentence=False),
                    self.format_volume_and_series(e, as_sentence=False),
                    optional[pages],
                ],
            ],
            sentence[
                optional_field("publisher"),
                optional_field("address"),
                self.format_edition(e),
            ],
            sentence[optional_field("note")],
            self.format_web_refs(e),
        ]

    def get_incollection_template(self, e):
        return toplevel[
            self.format_names("author"),
            join["(", field("year"), ")."],
            self.format_title(e, "title"),
            words[
                "In",
                sentence[
                    optional[self.format_editor(e, as_sentence=False)],
                    self.format_btitle(e, "booktitle", as_sentence=False),
                    self.format_volume_and_series(e, as_sentence=False),
                    self.format_chapter_and_pages(e),
                ],
            ],
            sentence[
                optional_field("publisher"),
                optional_field("address"),
                self.format_edition(e),
            ],
            sentence[optional_field("note")],
            self.format_web_refs(e),
        ]

    def get_misc_template(self, e):
        return toplevel[
            optional[sentence[self.format_names("author")]],
            join["(", field("year"), ")."],
            optional[self.format_title(e, "title")],
            sentence[
                optional[
                    words[
                        optional_field("archiveprefix"),
                        "preprint",
                        field("eprint"),
                        optional["[", field("primaryclass"), "]"],
                    ]
                ],
                optional_field("howpublished"),
                optional_field("publisher"),
            ],
            sentence[optional_field("note")],
            self.format_web_refs(e),
        ]


register_plugin("pybtex.style.formatting", "apa", ApaStyle)


class YearDescendingSortingStyle(BaseSortingStyle):
    def sorting_key(self, entry):
        year = entry.fields.get("year", "")
        try:
            year_value = int(year)
            missing_year = 0
        except ValueError:
            year_value = 0
            missing_year = 1

        if entry.type in ("book", "inbook"):
            author_key = ApaStyle().sorting_style.author_editor_key(entry)
        elif "author" in entry.persons:
            author_key = ApaStyle().sorting_style.persons_key(entry.persons["author"])
        else:
            author_key = ""

        return (missing_year, -year_value, author_key, entry.fields.get("title", ""))


class ApaRecentStyle(ApaStyle):
    default_sorting_style = "year_desc"


register_plugin("pybtex.style.sorting", "year_desc", YearDescendingSortingStyle)
register_plugin("pybtex.style.formatting", "apa_recent", ApaRecentStyle)
