.. Pattern Manager documentation master file, created by
   sphinx-quickstart on Tue Jun  4 13:26:07 2019.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Pattern Manager's documentation!
===========================================

This ROS node enables arrangement, management, and runtime iteration through various types of patterns (for e.g. palletization processes).

.. toctree::
   :maxdepth: 2

Currently supported pattern types include:

- linear
- rectangular
- circualr
- scatter

Example arrangement of groups and patterns:

.. code-block:: bash

	g0                          # Root (group)
	├── g1                      # Group
	│   ├── p1                  # Pattern
	│   │   ├── tf1             # Transform
	│   │   ├── tf2
	│   │   ├── tf3
	│   │   └── ...
	│   └── ...
	├── g2
	│   ├── g3
	│   │   └── p1
	│   │       ├── tf1
	│   │       └── ...
	│   └── ...
	└── ...

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Class diagram
=============

.. figure:: ../rsc/pm.svg


