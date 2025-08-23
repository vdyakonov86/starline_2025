FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# create a non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# RUN useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

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

# Необходимые пакеты
RUN apt install python3-pip python3-colcon-common-extensions
RUN pip3 install setuptools --break-system-packages

COPY scripts/.bashrc /home/${USERNAME}/bashrc
RUN cat /home/${USERNAME}/bashrc >> /home/${USERNAME}/.bashrc && rm /home/${USERNAME}/bashrc

# Remove folder in order to ensure to run apt-get update before installing new package 
RUN rm -rf /var/lib/apt/lists/*

# switch from root to user
USER $USERNAME
WORKDIR /ws

CMD ["bash"]