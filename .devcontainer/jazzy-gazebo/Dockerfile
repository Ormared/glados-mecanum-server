FROM althack/ros2:jazzy-gazebo

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME
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
   ros-jazzy-sdformat-urdf \
   net-tools \
   ufw \
   ros-jazzy-cv-bridge \
   ros-jazzy-image-transport 

RUN apt-get -y install --no-install-recommends \
   libopencv-dev \
   python3-opencv \
   ros-jazzy-teleop-twist-keyboard 

RUN apt-get install -y \
   unzip \
   wget \ 
   curl \
   apt-transport-https \
   gnupg \
   lsb-release \
   # packages for realsense
   libglfw3-dev \
   libssl-dev \
   libusb-1.0-0-dev \
   libudev-dev \
   pkg-config \
   libgtk-3-dev \
   cmake \
   build-essential \
   libglfw3-dev \
   libgl1-mesa-dev \
   libglu1-mesa-dev \
   at \
   rsync \
   v4l-utils \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# COPY ../../librealsense2_noble_x86_debians_beta.zip /tmp
RUN wget https://github.com/IntelRealSense/librealsense/releases/download/v2.56.3/librealsense2_noble_x86_debians_beta.zip -P /tmp
# COPY ../../pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl /tmp
RUN wget https://github.com/IntelRealSense/librealsense/releases/download/v2.56.3/pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl -P /tmp

# Install the library
RUN unzip /tmp/librealsense2_noble_x86_debians_beta.zip -d /tmp/librealsense
RUN dpkg -i /tmp/librealsense/*.deb
RUN apt --fix-broken install -y


# WHEELS FOR USING REALSENSE WITH PYTHON
# RUN pip install /tmp/pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl \
   # && rm -rf /tmp/*


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
