FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
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

COPY docker_harness/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
