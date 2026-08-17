Installation
============

Using CMake
-----------

1. Install CMake and a C++ compiler that supports C++17.
2. Create a build directory and navigate into it:

   .. code-block:: bash

      mkdir build
      cd build

3. Run CMake to configure the project:

   .. code-block:: bash

      cmake ..

4. Build the project:

   .. code-block:: bash

      cmake --build .

5. Optionally, install the library and tools:

   .. code-block:: bash

      sudo cmake --install .

Using Colcon
------------

The repository is a valid Colcon package. To build it put the package in your `Colcon workspace`_ and `build the workspace`_.

.. _Colcon workspace: https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html
.. _build the workspace: https://colcon.readthedocs.io/en/released/user/quick-start.html

