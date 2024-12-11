# FROM althack/ros2:jazzy-gazebo
FROM ormared/ros2:jazzy-cuda-gazebo

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME
# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   libxkbcommon-x11-0 \
   libxcb-xinerama0 \
   libxcb-xkb1 \
   libxcb-render-util0 \
   libx11-xcb1 \
   xvfb \
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
   ros-jazzy-teleop-twist-keyboard \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

   # RUN mkdir -p /var/run/user && chmod 755 /var/run/user
RUN sudo chown ros:ros /var/run/user

# USER ros



### For Windows, uncomment the following lines to install WSLg dependencies and utilize vGPU acceleration 
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install \
#           vainfo \
#           mesa-va-drivers \
#           mesa-utils \
#    # Clean up
#         && apt-get autoremove -y \
#         && apt-get clean -y \
#         && rm -rf /var/lib/apt/lists/*
# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
# CMD vainfo --display drm --device /dev/dri/card0
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
ENV GZ_VERSION=harmonic
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

################
# uncomment below for nvidia support
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
#   libnvidia-gl \
#   libnvidia-ml \ 
  libx11-dev \
  libxmu-dev \ 
  libxi-dev \
  libglu1-mesa \
  libglu1-mesa-dev \
  libfreeimage-dev \
  zlib1g \
  freeglut3-dev \
  mesa-utils \
&& rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ENV QT_X11_NO_MITSHM=1
# RUN sudo chmod 0700 /var/run/user/