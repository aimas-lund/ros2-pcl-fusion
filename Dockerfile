# --- Build stage
FROM ros:jazzy AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY . .

RUN source /opt/ros/jazzy/setup.bash \
	&& colcon build --symlink-install

# --- Runtime stage
FROM ros:jazzy AS runtime

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY --from=build /pcl_fusion_ws/install ./install
COPY --from=build /pcl_fusion_ws/build ./build

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /pcl_fusion_ws/install/setup.bash && ros2 run fusion fusion_node_exe"]