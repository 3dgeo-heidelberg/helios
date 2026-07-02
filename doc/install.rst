Installation
============

Conda installation
^^^^^^^^^^^^^^^^^^

The recommended way to install HELIOS is via the  `conda package manager`_.
As conda installer, we recommend `mamba`_, `micromamba`_, or `miniconda`_.

HELIOS can then be installed with:

.. code-block:: bash

    conda install -c conda-forge helios

.. _conda package manager: https://docs.conda.io/en/latest/
.. _mamba: https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html
.. _micromamba: https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html
.. _miniconda: https://docs.anaconda.com/free/miniconda/

Standalone Installer
^^^^^^^^^^^^^^^^^^^^

You can also install HELIOS via the standalone installers available for Windows, Linux and MacOS. They will not only install HELIOS but also add shortcuts for a) a HELIOS terminal session and b) a HELIOS Jupyter session.

Download the correct installer for your operating system from the `release page`_ and run it (under Windows, this is a setup wizard, under Linux and MacOS, it is a shell script).

.. _release page: https://github.com/3dgeo-heidelberg/helios/releases

Development installation
^^^^^^^^^^^^^^^^^^^^^^^^

If you intend to contribute to the development of HELIOS, we recommend a locally compiled version using these instructions:

.. code-block:: bash

    git clone https://github.com/3dgeo-heidelberg/helios.git
    cd helios
    conda env create -f environment-dev.yml
    conda activate helios-dev

    # On Linux, the following line is recommended, to go with a Conda-provided compiler.
    # We had issues with incompatible system compilers before.
    conda install -c conda-forge gcc gxx

    python -m pip install --no-build-isolation --config-settings=build-dir="build" -v -e .

This will install the HELIOS Python package in editable mode and expose the
CMake build directory used as `build` (adapt as needed). Additional CMake variables
can be passed with e.g. ``--config-settings=cmake.define.BUILD_TESTING="ON"``.
