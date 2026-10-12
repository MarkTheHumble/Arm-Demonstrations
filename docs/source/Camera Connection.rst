Camera Connection
=================

.. contents::
   :local:
   :depth: 2

Start by reading through the `OAK USB Deployment Guide <https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/>`__.
Pixi tasks are included with this project to help with the setup.

Install OAK Viewer into the Environment
---------------------------------------

.. code-block:: console

   $ pixi run install-oak

.. note::

   Pixi only supports Conda and PyPI dependencies.
   This calls a custom script to download the OAK Viewer ``.deb`` package and prepare a custom environment for it.
   Pixi will include this custom environment in its own ``oak-viewer`` environment.
   This should let you run OAK Viewer without relying on system-wide dependencies, which could change with updates.

Connect the Camera
------------------

Plug the camera into the host computer using a USB3 USB-C cable and capable port.
You may need to use two cables and a `Y-adapter <https://docs.luxonis.com/hardware/products/Y-adapter>`__ to meet the power requirements.

Verify the camera is connected properly and functioning by running OAK Viewer:

.. code-block:: console

   $ pixi run oak-viewer

.. note::

   If you are using WSL, you will need to use `usbipd-win <https://learn.microsoft.com/en-us/windows/wsl/connect-usb>`__ to make the device available to WSL.
   A script is included to automatically reconnect OAK cameras that use multiple PIDs.

   First, install ``usbipd-win``:

   .. code-block:: console

      $ winget.exe install --interactive --exact dorssel.usbipd-win

   Then run the helper script:

   .. code-block:: console

      $ pixi run connect-oak-wsl
   
   This will launch an elevated PowerShell process (using `WSL Interop <https://wsl.dev/technical-documentation/interop/>`__) to poll for OAK devices and connect them to WSL.
   This script will need to be left running while using the camera.

   Now run OAK Viewer:

   .. code-block:: console

      $ pixi run oak-viewer-wsl

   It may take some time or multiple attempts for it to successfully connect to the camera.

Next steps
----------

The camera should now be set up. See :doc:`Development <Development>` for how to run the demos.
