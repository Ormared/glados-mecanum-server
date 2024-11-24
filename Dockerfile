FROM althack/ros2:jazzy-dev AS base

RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   ros-jazzy-navigation2 \
   ros-jazzy-nav2-bringup \
   ros-jazzy-twist-mux \
   ros-jazzy-robot-localization \
   ros-jazzy-slam-toolbox \
   ros-jazzy-foxglove-bridge \
   net-tools \
   ufw \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

WORKDIR /workspace
COPY src src

# RUN rosdep update
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "export GZ_VERSION=harmonic" >> /home/ros/.bashrc
# RUN rosdep install --from-paths src --rosdistro jazzy - 
SHELL [ "/bin/bash", "-c" ]
# RUN colcon build --packages-ignore glados_gazebo



# FROM base AS builder

FROM althack/ros2:jazzy-base AS final
COPY install /workspace/install
COPY src /workspace/src
# COPY entrypoint.sh /workspace/

ENV ROS_DISCOVERY_SERVER=10.0.1.2:11811

RUN apt-get update \
  && apt-get -y install --no-install-recommends \
  ros-jazzy-foxglove-bridge \
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog
WORKDIR /workspace

#CMD ["ros2", "launch", "glados_bringup", "glados_onrobot.launch.py"]
#ENTRYPOINT ["entrypoint.sh"]
#CMD ["/bin/bash", "-c"]
# CMD ["ros2", "launch", "glados_bringup", "glados_onrobot.launch.py"]
ENTRYPOINT ["/bin/bash", "-c", "source install/setup.bash && ros2 launch glados_bringup glados_onrobot.launch.py"]
# ENTRYPOINT ["/bin/bash", "-c", "source install/setup.bash && bash"]






