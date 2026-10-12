Installation
============

.. contents::
   :local:
   :depth: 2

Clone the repository
--------------------

.. code-block:: console

   $ git clone https://github.com/MRRP-lab/arm-demos.git
   $ cd arm-demos

.. note::

   If you are on Windows, first install Ubuntu 22.04 with WSL and then clone the repository onto your WSL filesystem.

   .. code-block:: doscon

      > wsl --install Ubuntu-22.04

   After installation is complete and you are logged in to the WSL system, run the clone commands from above.


Install dependencies
---------------------

Use the instructions from https://pixi.sh/latest/ to install ``pixi``.
Once ``pixi`` has been installed, close the terminal session and start it again, which will ensure ``pixi`` is on the PATH.

Install the required dependencies into an isolated development environment:

.. code-block:: console

   $ pixi install

You will get a warning. This is expected:

.. code-block:: console

    WARN Could not find activation scripts: ./install/setup.sh

This is ``pixi`` trying to include our packages in the shell environment.
This will not work until we build our source code.
This is done in the next step.

Build the source code
---------------------

.. code-block:: console

   $ pixi run build

This will build and create an `editable install <https://pip.pypa.io/en/stable/topics/local-project-installs/#editable-installs>`__ of the packages from the ``src`` folder.
This command should be re-run if metadata is changed or a new file is added to the ``src`` folder.
Updates to existing files should not need a rebuild to take effect.

.. note::

   This will build the project's source code into the ``install`` folder.
   It will also create the missing ``install/setup.sh`` script causing the previous warning.
   The ``setup.sh`` script is used to make the packages in the ``install`` folder available for use in the current shell.
   Pixi will automatically handle calling this script.

Download the MediaPipe Model
----------------------------

This project uses hand tracking that depends on a model that must be downloaded separately.
A task is available to perform the download:

.. code-block:: console

   $ pixi run download-mediapipe-model

Alternatively, you can `download the model yourself <https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker#models>`__.
The file should be named ``hand_landmarker.task`` in the project root directory.

Next steps
----------

See :doc:`Camera Connection <Camera Connection>`.
