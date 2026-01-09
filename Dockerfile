ARG ROS_DISTRO=jazzy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DOMAIN_ID=0

# --- Defining base 
FROM ros:${ROS_DISTRO}-ros-base AS base

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG ROS_DOMAIN_ID

ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
ENV ROS_LOCALHOST_ONLY=0

# Install rosdep and initialize
RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
	python3-rosdep \
	&& rm -rf /var/lib/apt/lists/* \
	&& if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
	&& rosdep fix-permissions

# Create a non-root user to run the application
RUN set -eux; \
	if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi; \
	if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
	if getent passwd "${USER_UID}" >/dev/null; then \
	useradd --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
	else \
	useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
	fi; \
	fi; \
	mkdir -p /pcl_fusion_ws; \
	chown -R "${USERNAME}:${USER_GID}" /pcl_fusion_ws

# Run rosdep update as the non-root user
RUN su -s /bin/bash - "${USERNAME}" -c "rosdep update"

USER ${USERNAME}

# --- Build stage
FROM base AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

ARG USERNAME
ARG USER_GID

COPY . .

USER root
RUN chown -R "${USERNAME}:${USER_GID}" /pcl_fusion_ws
RUN rosdep install --from-paths src --ignore-src -r -y

USER ${USERNAME}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
	&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Runtime stage
FROM base AS runtime

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY --from=build /pcl_fusion_ws/install /pcl_fusion_ws/install
COPY --from=build /pcl_fusion_ws/build /pcl_fusion_ws/build

# Create the directory used for bind-mounting an override params file.
USER root
RUN mkdir -p /pcl_fusion_ws/install/fusion/share/fusion/config/mnt \
	&& chown -R ${USERNAME}:${USER_GID} /pcl_fusion_ws/install/fusion/share/fusion/config
USER ${USERNAME}

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /pcl_fusion_ws/install/setup.bash && ros2 launch fusion fusion.launch.py"]