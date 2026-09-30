CLI interface
=============

You can use the CLI tool to test and configure the OnRobot 2FG7 gripper.

Usage
-----

.. code-block:: console

   onrobot_api [-i] <address> [port]
   onrobot_api [-f force] <address> [port]
   onrobot_api [-p position] <address> [port]

Options
-------

.. code-block:: console

   -i            Print device info
   -f force      Set gripper force
   -p position   Set gripper position

Example
-------

- Device info ::code:`onrobot_cli -i 192.168.1.1`

- Set gripper force ::code:`onrobot_cli -f 20.0 192.168.1.1`

- Set position ::code:`onrobot_cli -p 0.02 192.168.1.1`
