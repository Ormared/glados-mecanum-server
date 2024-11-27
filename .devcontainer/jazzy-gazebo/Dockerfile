FROM althack/ros2:jazzy-gazebo

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   libxkbcommon-x11-0 \
   libxcb-xinerama0 \
   libxcb-xinput0 \
   x11-apps \
   iputils-ping \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

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
   ros-jazzy-cv-bridge \
   ros-jazzy-image-transport 

RUN apt-get -y install --no-install-recommends \
   libopencv-dev \
   python3-opencv \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# RUN git clone https://github.com/Slamtec/rplidar_sdk \
#       && cd rplidar_sdk \
#       && make

# RUN mkdir -p /var/run/user && chmod 755 /var/run/user
RUN sudo chown ros:ros /var/run/user

ENV GZ_VERSION=harmonic
# USER ros

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
# RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

# sudo apt install ros-jazzy-navigation2
# sudo apt install ros-jazzy-nav2-bringup

################
# uncomment below for nvidia support
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
# RUN apt-get update \
#  && apt-get install -y -qq --no-install-recommends \
#   libglvnd0 \
#   libgl1 \
#   libglx0 \
#   libegl1 \
#   libxext6 \
#   libx11-6 \
# && rm -rf /var/lib/apt/lists/*

# # Env vars for the nvidia-container-runtime.
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# RUN sudo chmod 0700 /var/run/user/
