# syntax=docker/dockerfile:1.6

ARG ISAAC_SIM_BASE_IMAGE=nvcr.io/nvidia/isaac-sim:5.0.0
FROM ${ISAAC_SIM_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=humble
ARG ISAACLAB_REF=v2.2.0
ARG WORKSPACE_DIR=/workspace

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    WORKSPACE_DIR=${WORKSPACE_DIR} \
    ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    LIVESTREAM=2 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOCALHOST_ONLY=0 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    iproute2 \
    lsb-release \
    procps \
    software-properties-common \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor --batch --yes -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-tf2-ros \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN (rosdep init 2>/dev/null || true) && rosdep update

RUN if [ -f /opt/IsaacLab/source/isaaclab/pyproject.toml ]; then \
        echo "Reusing IsaacLab already present in base image"; \
    else \
        rm -rf /opt/IsaacLab \
        && mkdir -p /opt/IsaacLab \
        && curl -fsSL "https://codeload.github.com/isaac-sim/IsaacLab/tar.gz/refs/tags/${ISAACLAB_REF}" \
            | tar -xz --strip-components=1 -C /opt/IsaacLab; \
    fi \
    && ln -sfn /isaac-sim /opt/IsaacLab/_isaac_sim

RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation flatdict==4.0.1
RUN /isaac-sim/python.sh -m pip install --no-cache-dir h5py
RUN /isaac-sim/python.sh -m pip install --no-cache-dir \
        prettytable==3.3.0 \
        wcwidth==0.6.0 \
        hidapi==0.15.0 \
        cloudpickle==3.1.2 \
        farama-notifications==0.0.4
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-deps gymnasium==0.29.0
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation \
        -e /opt/IsaacLab/source/isaaclab \
    && /isaac-sim/python.sh -c "import isaaclab; print('Isaac Lab core package ready')"
RUN /isaac-sim/python.sh -m pip install --no-cache-dir rsl-rl-lib==2.3.3
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation \
        -e /opt/IsaacLab/source/isaaclab_tasks \
        -e /opt/IsaacLab/source/isaaclab_rl

RUN mkdir -p /workspace/checkpoints /workspace/scene_models /workspace/rosclaw_ws/src

COPY --from=booster_assets . /workspace/booster_assets
COPY --from=booster_train . /workspace/booster_train

RUN /isaac-sim/python.sh -m pip install --no-cache-dir -e /workspace/booster_assets \
    && /isaac-sim/python.sh -m pip install --no-cache-dir -e /workspace/booster_train/source/booster_train

COPY --from=rosclaw_ros2 rosclaw_msgs /workspace/rosclaw_ws/src/rosclaw_msgs
COPY --from=rosclaw_ros2 rosclaw_discovery /workspace/rosclaw_ws/src/rosclaw_discovery
COPY --from=rosclaw_ros2 rosclaw_estop /workspace/rosclaw_ws/src/rosclaw_estop
COPY --from=rosclaw_ros2 rosclaw_k1_bridge /workspace/rosclaw_ws/src/rosclaw_k1_bridge
COPY --from=rosclaw_ros2 rosclaw_bringup /workspace/rosclaw_ws/src/rosclaw_bringup

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install \
        --from-paths /workspace/rosclaw_ws/src \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        -y -r \
    && cd /workspace/rosclaw_ws \
    && colcon build --symlink-install --packages-select \
        rosclaw_msgs \
        rosclaw_discovery \
        rosclaw_estop \
        rosclaw_k1_bridge \
        rosclaw_bringup

RUN mkdir -p /workspace/rosclaw-example-booster-k1
WORKDIR /workspace/rosclaw-example-booster-k1

COPY tf2_ros.py /workspace/rosclaw-example-booster-k1/tf2_ros.py
COPY overrides/control.py /workspace/booster_train/source/booster_train/booster_train/demo/control.py
COPY overrides/motion_replay.py /workspace/booster_train/source/booster_train/booster_train/demo/motion_replay.py
COPY overrides/velocity_env.py /workspace/booster_train/source/booster_train/booster_train/demo/velocity_env.py
COPY docker/entrypoint-k1.sh /entrypoint-k1.sh
COPY docker/start-k1-teleop.sh /start-k1-teleop.sh

RUN chmod +x /entrypoint-k1.sh /start-k1-teleop.sh

ENTRYPOINT ["/entrypoint-k1.sh"]
