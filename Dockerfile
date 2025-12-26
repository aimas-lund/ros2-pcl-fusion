# --- Build stage
FROM ros:jazzy AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY . .

RUN source /opt/ros/jazzy/setup.bash \
	&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Runtime stage
FROM ros:jazzy AS runtime

SHELL ["/bin/bash", "-c"]

WORKDIR /pcl_fusion_ws

COPY --from=build /pcl_fusion_ws/install /pcl_fusion_ws/install
COPY --from=build /pcl_fusion_ws/build /pcl_fusion_ws/build

RUN ls -al /pcl_fusion_ws

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /pcl_fusion_ws/install/setup.bash && ros2 launch fusion fusion.launch.py"]