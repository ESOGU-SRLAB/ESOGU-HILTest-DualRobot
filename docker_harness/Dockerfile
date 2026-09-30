FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# python3-pytest / python3-pytest-timeout: STLC Manager sends pytest files;
# entrypoint.sh runs them with pytest (see there for why plain python3 is wrong).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        python3-pytest \
        python3-pytest-timeout \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /harness_ws/src

COPY pymoveit2_sim             ./pymoveit2_sim
COPY pymoveit2_real            ./pymoveit2_real
COPY pymoveit2_kawasaki_sim    ./pymoveit2_kawasaki_sim
COPY pymoveit2_kawasaki_real   ./pymoveit2_kawasaki_real

WORKDIR /harness_ws

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select \
        pymoveit2_sim \
        pymoveit2_real \
        pymoveit2_kawasaki_sim \
        pymoveit2_kawasaki_real

# STLC-generated code does `from sim_robot_goal import SimCollisionAwareRobotController`.
# That file is an example that CMake installs as a program under lib/pymoveit2_sim,
# which is not on the Python path, so the import failed with ModuleNotFoundError.
# Only the UR package's dir is added: pymoveit2_kawasaki_sim ships a different
# sim_robot_goal.py with the same class name, and two on the path would shadow each other.
# setup.bash (sourced in entrypoint.sh) prepends its own paths and keeps this one.
ENV PYTHONPATH=/harness_ws/install/pymoveit2_sim/lib/pymoveit2_sim

# Loaded by pytest because the generated file is mounted next to it in /harness_ws.
COPY docker_harness/conftest.py /harness_ws/conftest.py

# Post-test homing, run by entrypoint.sh after every test (trusted code, not STLC's).
COPY docker_harness/reset_home.py /harness_ws/harness_tools/reset_home.py

COPY docker_harness/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Required run flags (without them discovery works but no data flows):
#   --network host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
