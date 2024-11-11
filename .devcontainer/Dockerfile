FROM althack/ros2:jazzy-gazebo

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME
# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends libxkbcommon-x11-0 libxcb-xinerama0 x11-apps \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# RUN mkdir -p /var/run/user && chmod 755 /var/run/user
RUN sudo chown ros:ros /var/run/user

# USER ros

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# RUN sudo chmod 0700 /var/run/user/