# Documentation (Local Build)

This folder contains the Sphinx sources for the HELIOS++ documentation. The
local build uses Doxygen for the C++ API XML and Sphinx for the HTML output.

## Prerequisites

- Install Doxygen via your OS
- `python -m pip install -r doc/requirements-rtd.txt`

## Build

From the repo root, configure a docs-only build and generate the HTML:

```bash
cmake -S . -B build-docs -DHELIOS_DOCS_ONLY=ON
cmake --build build-docs --target sphinx-doc
```

The HTML output is written to:

```
build-docs/doc/sphinx
```

Alternatively, if you are not interested in local Doxygen builds, you can also
call Sphinx directly and ignore the Doxygen-related error message:

```
sphinx-build -b html <src-foc-folder> <target-output-directory>
```

## Serve

To preview locally, serve the HTML folder with a simple HTTP server:

```bash
cd build-docs/doc/sphinx
python -m http.server -d build-docs/doc/sphinx 8000
```

Then open `http://localhost:8000` in a browser.

## Rebuild

After editing `.rst` or notebook links, re-run:

```bash
cmake --build build-docs --target sphinx-doc
```
