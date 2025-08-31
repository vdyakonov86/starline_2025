FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# create a non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# set up sudo privileges
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# add user to video group to allow access to webcam
RUN usermod --append --groups video $USERNAME

# Установка локали
RUN apt update && sudo apt install locales \ 
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && LANG=en_US.UTF-8

# Необходимые системные утилиты
RUN apt-get install -y \
    vim git wget curl zip unzip x11-apps \
    python3-pip python3-colcon-common-extensions python3-argcomplete bash-completion \
    cmake g++

RUN pip3 install setuptools rosbags

# ROS пакеты
RUN apt install -y ros-humble-rosbag2-storage-mcap ros-humble-kobuki-ros-interfaces

# PCL
RUN apt install libpcl-dev pcl-tools

# FLANN
# Fix issue: https://github.com/flann-lib/flann/issues/369
COPY fix_flann.sh /tmp/fix_flann.sh

RUN wget -O flann.zip https://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip \
    && unzip flann.zip \
    && cd flann-1.8.4-src \
    && chmod +x /tmp/fix_flann.sh && /tmp/fix_flann.sh \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install

# VTK
RUN wget https://vtk.org/files/release/9.5/VTK-9.5.1.tar.gz 
RUN mkdir -p vtk/source \
    && tar -xzvf VTK-9.5.1.tar.gz -C vtk/source \
    && cd vtk/source/VTK-9.5.1 \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install

#Eigen 
# download and unpack sources, there is no need to install
RUN wget -O eigen.zip https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip \
    && unzip /eigen.zip \
    && cp -r /eigen-3.4.0/Eigen /usr/local/include \
    && rm /eigen.zip \
    && rm -rf /eigen-3.4.0

RUN wget https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.15.1/source.tar.gz
RUN tar -xzvf source.tar.gz \
    && cd pcl && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4 \
    && make -j4 install

# Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd ./Livox-SDK2/ \
    && mkdir build \
    && cd ./build \
    && cmake .. && make -j$(nproc) \
    && make install


# Livox ROS Driver 2
# WORKDIR /
# RUN mkdir ws_livox && git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2

    
COPY scripts/.bashrc /home/${USERNAME}/bashrc
RUN cat /home/${USERNAME}/bashrc >> /home/${USERNAME}/.bashrc && rm /home/${USERNAME}/bashrc

# Remove folder in order to ensure to run apt-get update before installing new package 
RUN rm -rf /var/lib/apt/lists/*

# switch from root to user
USER $USERNAME
WORKDIR /starline

CMD ["bash"]