Introduction
============

HELIOS (\ **H**\ eidelberg **LI**\ DAR **O**\ perations **S**\ imulator) is a general-purpose Python package for simulation of terrestrial, mobile and airborne laser scanning surveys written in C++11. 
It is developed and maintained by the `3DGeo Research Group`_ at Heidelberg University.

This documentation contains the full HELIOS documentation, including notebook-based usage tutorials,
detailed explanations, and the complete API reference for both the Python and C++ interfaces.

.. _3DGeo Research Group: https://uni-heidelberg.de/3dgeo

Official website: https://uni-heidelberg.de/helios
GitHub repository: https://github.com/3dgeo-heidelberg/helios

.. figure:: /img/vls.png
   :alt: Virtual laser scanning
   :width: 100%
   :align: center

   Concept of virtual laser scanning (modified from `Winiwarter et al., 2022`_).

.. _Winiwarter et al., 2022: https://doi.org/10.1016/j.rse.2021.112772


How to cite HELIOS
--------------------

If you use HELIOS in your research, please cite

  Höfle B., Weiser H., Kapitan D., Kempf D. (2026). HELIOS (version 3.0.0). DOI: 10.5281/zenodo.4452870 URL: https://github.com/3dgeo-heidelberg/helios


.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   install

.. toctree::
   :maxdepth: 2
   :caption: Tutorials

   python
   notebooks

.. toctree::
   :maxdepth: 2
   :caption: Explanation

   scanners_platforms
   intensity_fwf
   error_randomness
   cli

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   pythonapi
   cppapi
   
.. toctree::
   :maxdepth: 2
   :caption: Research

   research_using_helios
