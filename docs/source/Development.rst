Development
===========

There are a variety of ways to interact with this project.
Here are some recommended methods:

.. contents::
   :local:
   :depth: 2

Terminal
--------

When opening a new terminal session, make sure to activate the development environment:

.. code-block:: console

   $ pixi shell

This will make all the necessary dependencies available in your shell.

Visual Studio Code
------------------

Install `Pixi Code <https://marketplace.visualstudio.com/items?itemName=renan-r-santos.pixi-code?>`__.
This will automatically set your Python interpreter and run ``pixi shell`` at the start of new terminal sessions.


.. note::

   If you don't want to use an extension, some configuration can still be done manually to simplify the development experience.
   Edit your `settings.json <https://code.visualstudio.com/docs/configure/settings#_settings-json-file>`__ and add a terminal profile that automatically runs ``pixi shell`` on startup:

   .. code-block:: json

      {
         "terminal.integrated.profiles.linux": { "pixi": { "path": "bash", "args": ["-ilc", "pixi shell"] } }
      }

   Additionally, you'll have to `set your Python interpreter <https://code.visualstudio.com/docs/python/environments#_select-an-environment>`__ manually.
   The Python executable can be found at ``.pixi/envs/default/bin/python`` relative to your project root directory.

Next steps
----------

See :doc:`Usage <Usage>`.
