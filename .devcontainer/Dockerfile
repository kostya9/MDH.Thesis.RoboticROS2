# This is the development environment
FROM athackst/ros2:dashing-dev

# Uncomment this line and add custom packages and dependencies you want to install here.
# sudo apt-get update && sudo apt-get install -y \
# put dependent packages here
# && rm -rf /var/lib/apt/lists/*

# install gazebo
# RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo apt-get update
RUN sudo apt install -y ros-dashing-gazebo-ros-pkgs

# install PointCloudLibrary (pcl)
RUN sudo apt install -y libpcl-dev
RUN sudo apt install -y ros-dashing-pcl-conversions

# install rviz2
RUN sudo apt install -y ros-dashing-rviz2


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

# The entrypoint for the container to source the environment
# COPY entrypoint.sh /setup/entrypoint.sh
ENTRYPOINT [ "/setup/entrypoint.sh" ]

CMD exec /bin/bash -c "trap : TERM INT; sleep infinity & wait"
