# This is the development environment
FROM athackst/ros2:eloquent-dev

# install gazebo
RUN sudo apt-get update
RUN sudo apt install -y ros-eloquent-gazebo-ros-pkgs

# install PointCloudLibrary (pcl)
RUN sudo apt install -y libpcl-dev
RUN sudo apt install -y ros-eloquent-pcl-conversions

# install rviz2
RUN sudo apt install -y ros-eloquent-rviz2

# Install python UI libs
RUN pip3 install kivy pygame

# This Dockerfile adds a non-root 'vscode' user with sudo access. However, for Linux,
# this user's GID/UID must match your local user UID/GID to avoid permission issues
# with bind mounts. Update USER_UID / USER_GID if yours is not 1000. See
# https://aka.ms/vscode-remote/containers/non-root-user for details.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# TODO: FIX ignition config
# ~/.ignition/fuel/config.yaml
# url: https://fuel.ignitionrobotics.org/
# https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo

CMD exec /bin/bash -c "trap : TERM INT; sleep infinity & wait"
