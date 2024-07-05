FROM ros:humble

# Copy over the source code
WORKDIR /ws
COPY . /ws/src/docking_robot

# Pull VCS repos
RUN vcs import src/docking_robot < src/docking_robot/docking_robot.repos

# Install rosdeps
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    HUSARION_ROS_BUILD_TYPE=simulation \
    MAKEFLAGS="-j 8" \
    colcon build --symlink-install --parallel-workers 2

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && rm -rf /tmp/*

# Create the entrypoint
RUN echo '#!/bin/sh\n\
ulimit -Sn 524288\n\
ulimit -Hn 524288\n\
. /ws/install/setup.sh\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "docking_bringup", "docking.launch.py"]
