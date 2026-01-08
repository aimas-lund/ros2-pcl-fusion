ARG ROS_DISTRO=jazzy

# --- Defining base 
FROM ros:${ROS_DISTRO}-ros-base AS base

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
	python3-rosdep \
	&& rm -rf /var/lib/apt/lists/* \
	&& if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
	&& rosdep fix-permissions \
	&& rosdep update


# --- Build stage
FROM base AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY . .

RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
	&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Runtime stage
FROM base AS runtime

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY --from=build /pcl_fusion_ws/install /pcl_fusion_ws/install
COPY --from=build /pcl_fusion_ws/build /pcl_fusion_ws/build

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /pcl_fusion_ws/install/setup.bash && ros2 launch fusion fusion.launch.py"]