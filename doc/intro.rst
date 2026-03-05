Introduction
============

HELIOS++ is a general-purpose Python package for simulation of terrestrial, mobile and airborne laser scanning surveys written in C++11. 
It is developed and maintained by the `3DGeo Research Group`_ at Heidelberg University.

This documentation contains the API references and notebook-based examples for Helios++.
For project usage details, installation instructions, and extended background material,
see the repository ``README.md``.

.. _3DGeo Research Group: https://uni-heidelberg.de/3dgeo

Installation
------------

Conda installation
^^^^^^^^^^^^^^^^^^

The recommended way to install HELIOS++ is via the  `conda package manager`_.

The following software is required for installation of HELIOS++:
* a Conda installation. We recommend `mamba`_, `micromamba`_, or `miniconda`_.

HELIOS++ can then be installed with:

```bash
conda install -c conda-forge helios
```

.. _conda package manager: https://docs.conda.io/en/latest/
.. _mamba: https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html
.. _micromamba: https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html
.. _miniconda: https://docs.anaconda.com/free/miniconda/

Standalone Installer
^^^^^^^^^^^^^^^^^^^^

You can also install HELIOS++ via the standalone installers available for Windows, Linux and MacOS. They will not only install HELIOS++ but also add shortcuts for a) a H++ terminal session and b) a H++ Jupyter session.

Download the correct installer for your operating system from the `release page`_ and run it (under Windows, this is a setup wizard, under Linux and MacOS, it is a shell script).

.. _release page: https://github.com/3dgeo-heidelberg/helios/releases

Development installation
^^^^^^^^^^^^^^^^^^^^^^^^

If you intend to contribute to the development of HELIOS++, we recommend a locally compiled version using these instructions:

```bash
git clone https://github.com/3dgeo-heidelberg/helios.git
cd helios
conda env create -f environment-dev.yml
conda activate helios-dev

# On Linux, the following line is recommended, to go with a Conda-provided compiler.
# We had issues with incompatible system compilers before.
conda install -c conda-forge gcc gxx

python -m pip install --no-build-isolation --config-settings=build-dir="build" -v -e .
```

This will install the HELIOS++ Python package in editable mode and expose the
CMake build directory used as `build` (adapt as needed). Additional CMake variables
can be passed with e.g. `--config-settings=cmake.define.BUILD_TESTING="ON"`.

